/* SPDX-License-Identifier: MIT */
/* PPE return queue split between core 0 (pop) and core 1 (consume).
 *
 * Two-hart interleaving model: core 0, core 1 and the host/PPE run as
 * threads, but a seeded scheduler lets exactly one of them run between two
 * shared-memory accesses, so every run is deterministic. Stores of the two
 * harts to the split queue go through a store buffer that commits in random
 * order except across barrier(), which models the weak ordering a missing
 * fence would expose.
 *
 * Checked: every hardware entry is processed exactly once and in order; an
 * entry that cannot be delivered (Linux ring full) waits at the tail; the
 * Linux stop sequence (q[13]=0 only with an empty queue and core 0 idle);
 * a restart sees no stale entries; a bad token sets q[3]=4 and stops pops.
 */
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "../protocol.h"

enum { CORE0, CORE1, HOST, ACTORS };

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static int turn = HOST;
static int alive[ACTORS] = { 1, 1, 1 };
static unsigned weight[ACTORS] = { 4, 4, 2 };
static __thread int self = HOST;
static uint64_t rng = 0x7552531a2026ULL;
static int stop_cores;

static uint32_t rnd(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return (uint32_t)(rng >> 11);
}

/* Per-hart store buffers for the split queue words. */
#define SB 64
struct pending {
	volatile uint32_t *p;
	uint32_t v;
};
static struct pending sb[2][SB];
static int sbn[2];

static void commit(int c, int i)
{
	/* Stores to one word stay in program order. */
	for (int j = 0; j < i; j++)
		if (sb[c][j].p == sb[c][i].p) {
			i = j;
			break;
		}
	*sb[c][i].p = sb[c][i].v;
	memmove(&sb[c][i], &sb[c][i + 1], (sbn[c] - i - 1) * sizeof(sb[c][0]));
	sbn[c]--;
}

static void drain(int c)
{
	while (sbn[c])
		commit(c, 0);
}

static void yield(void)
{
	unsigned total = 0, r;
	int next;

	for (int c = 0; c < 2; c++)
		if (sbn[c] && !(rnd() % 3))
			commit(c, rnd() % sbn[c]);
	for (int a = 0; a < ACTORS; a++)
		total += alive[a] ? weight[a] : 0;
	if (!total)
		return;
	r = rnd() % total;
	for (next = 0; next < ACTORS; next++) {
		unsigned w = alive[next] ? weight[next] : 0;

		if (r < w)
			break;
		r -= w;
	}
	if (next == self)
		return;
	turn = next;
	pthread_cond_broadcast(&cv);
	while (turn != self)
		pthread_cond_wait(&cv, &mu);
}

/* What hart c would read (its own buffered store first). */
static uint32_t view(int c, volatile uint32_t *p)
{
	for (int i = sbn[c] - 1; i >= 0; i--)
		if (sb[c][i].p == p)
			return sb[c][i].v;
	return *p;
}

static uint32_t split_ld(volatile uint32_t *p)
{
	yield();
	return self < 2 ? view(self, p) : *p;
}

static void split_st(volatile uint32_t *p, uint32_t v)
{
	yield();
	if (self >= 2) {
		*p = v;
		return;
	}
	if (sbn[self] == SB)
		commit(self, 0);
	sb[self][sbn[self]].p = p;
	sb[self][sbn[self]].v = v;
	sbn[self]++;
}

static void barrier(void)
{
	yield();
	if (self < 2)
		drain(self);
}

/* PPE return queue model: registers fe4 (count), fe0 (status), fdc
 * (metadata); writing 0x80000000 to fe0 acknowledges the front entry.
 */
#define FIFO 128
static uint32_t fifo_st[FIFO], fifo_md[FIFO];
static uint32_t fifo_r, fifo_w;
static int presented;

static uint32_t read32(uint32_t address)
{
	yield();
	assert(self == CORE0);
	if (address == 0x1fb50fe4)
		return (fifo_w - fifo_r) & 65535;
	if (address == 0x1fb50fe0) {
		if (fifo_w == fifo_r)
			return 0;
		presented = 1;
		return fifo_st[fifo_r % FIFO];
	}
	assert(address == 0x1fb50fdc && presented);
	return fifo_md[fifo_r % FIFO];
}

static void ppe_w32(uint32_t address, uint32_t value);
#define SPLIT_LD(p) split_ld(p)
#define SPLIT_ST(p, v) split_st(p, v)
#define PPE_W32(a, v) ppe_w32(a, v)

/* Core 1 buffer ownership, reduced to what the split code relies on. */
static uint32_t states[AN7552_RX_BUFFER_COUNT];
static uint32_t done[8192], ndone, refused, host_full;

static uint32_t st_get(uint32_t token)
{
	return states[token];
}

static void native_rx_release(uint32_t token)
{
	yield();
	assert(self == CORE1 && states[token] == 2);
	states[token] = 0;
	done[ndone++] = token;
}

static int native_rx_deliver(uint32_t token, uint32_t info)
{
	yield();
	assert(self == CORE1);
	if (host_full) {
		refused++;
		return 0;
	}
	assert(states[token] == 2);
	assert(info == (((token * 7) & 0x1fffff) | (1U << 29)));
	states[token] = 0;
	done[ndone++] = token;
	return 1;
}

#include "../ppe-split.c.inc"

static volatile uint32_t *ppe_q(void)
{
	return (void *)(uintptr_t)(AN7552_NPU_BASE + 0x6c0);
}

static void ppe_w32(uint32_t address, uint32_t value)
{
	volatile uint32_t *slot;
	uint32_t head;

	yield();
	assert(self == CORE0 && address == 0x1fb50fe0 && value == 0x80000000);
	assert(presented && fifo_w != fifo_r);
	/* Acknowledged only after the entry is stored under a published head. */
	head = view(CORE0, &ppe_split_ctrl()[PS_HEAD]);
	slot = ppe_split_slot(head - 1);
	assert(view(CORE0, &slot[0]) == fifo_st[fifo_r % FIFO]);
	assert(view(CORE0, &slot[1]) == fifo_md[fifo_r % FIFO]);
	fifo_r++;
	presented = 0;
}

/* Receive side on core 1: queue frames to the PPE in batches; q[12] is
 * published at the end of a batch, after the doorbells.
 */
static uint32_t inppe[64], inppe_r, inppe_w, to_queue, next_token;
static uint32_t pushed[8192], npushed;

static void core1_step(void)
{
	volatile uint32_t *q = ppe_q();

	if (q[0] == 2 && !q[3] && to_queue && (rnd() & 1)) {
		uint32_t room = 63 - (inppe_w - inppe_r), k = 1 + rnd() % 6;

		if (k > to_queue)
			k = to_queue;
		if (k > room)
			k = room;
		for (uint32_t i = 0; i < k; i++) {
			assert(!states[next_token]);
			states[next_token] = 2;
			inppe[inppe_w++ % 64] = next_token++;
		}
		to_queue -= k;
		yield();
		q[12] += k;
	}
	ppe_split_service(q);
}

static void *core_main(void *arg)
{
	self = (int)(intptr_t)arg;
	pthread_mutex_lock(&mu);
	while (turn != self)
		pthread_cond_wait(&cv, &mu);
	while (!stop_cores) {
		if (self == CORE0)
			native_ppe_pop();
		else
			core1_step();
		yield();
	}
	drain(self);
	alive[self] = 0;
	turn = HOST;
	pthread_cond_broadcast(&cv);
	pthread_mutex_unlock(&mu);
	return NULL;
}

static void hw_push(uint32_t token, int release)
{
	fifo_st[fifo_w % FIFO] =
		0x80000000 | (release ? 0x40000000 : 0) | token;
	fifo_md[fifo_w % FIFO] = (token * 7) & 0x1fffff;
	fifo_w++;
	pushed[npushed++] = token;
}

/* The PPE returns forwarded frames (release) and some for Linux. */
static void hw_step(void)
{
	for (uint32_t n = rnd() % 4; n && inppe_r != inppe_w &&
				     fifo_w - fifo_r < FIFO;
	     n--) {
		uint32_t token = inppe[inppe_r++ % 64];

		hw_push(token, token % 3 != 0);
	}
}

static int toggle_full;

static void host_step(void)
{
	hw_step();
	if (toggle_full && !(rnd() % 40))
		host_full = !host_full;
	yield();
}

static void run(unsigned steps)
{
	while (steps--)
		host_step();
}

static void set_weights(unsigned c0, unsigned c1, unsigned host)
{
	weight[CORE0] = c0;
	weight[CORE1] = c1;
	weight[HOST] = host;
}

/* Linux native_ppe_start(): q cleared, producer index, mode 2. */
static void ppe_start(void)
{
	volatile uint32_t *q = ppe_q();

	for (int i = 0; i < 16; i++)
		q[i] = 0;
	q[10] = 7;
	q[0] = 2;
}

/* Committed (memory) state of the split queue. */
static uint32_t mem(uint32_t word)
{
	return ppe_split_ctrl()[word];
}

static uint32_t session_pushed, session_done;

static void session_begin(void)
{
	session_pushed = npushed;
	session_done = ndone;
}

/* Processed entries of this session are a prefix of the returned ones. */
static void check_order(void)
{
	assert(ndone - session_done <= npushed - session_pushed);
	for (uint32_t i = session_done; i < ndone; i++)
		assert(done[i] == pushed[session_pushed + i - session_done]);
}

static void wait_for(volatile uint32_t *word, uint32_t value, unsigned limit,
		     int must)
{
	unsigned n = 0;

	while (*word != value && n++ < limit)
		host_step();
	assert(!must || *word == value);
}

/* Linux native_ppe_stop() after q[12] reached 0 or timed out: q[0]=0 and
 * wait for q[13]==0. When it reads 0, core 0 must be idle and the queue
 * empty, and nothing may be published afterwards.
 */
static void stop_and_check(void)
{
	volatile uint32_t *q = ppe_q();
	uint32_t head;

	q[0] = 0;
	wait_for(&q[13], 0, 400000, 1);
	head = mem(PS_HEAD);
	assert(head == mem(PS_TAIL));
	assert(!mem(PS_ACTIVE) && !mem(PS_ENABLE));
	run(3000);
	assert(!q[13] && mem(PS_HEAD) == head && mem(PS_TAIL) == head);
}

int main(void)
{
	volatile uint32_t *q = ppe_q();
	pthread_t threads[2];
	uint32_t dropped, popped, head;

	assert(mmap((void *)(uintptr_t)AN7552_NPU_BASE, 4096,
		    PROT_READ | PROT_WRITE,
		    MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1,
		    0) == (void *)(uintptr_t)AN7552_NPU_BASE);
	(void)native_ppe_pop;
	ppe_split_boot(0);
	ppe_split_boot(1);
	pthread_mutex_lock(&mu);
	for (int c = 0; c < 2; c++)
		assert(!pthread_create(&threads[c], NULL, core_main,
				       (void *)(intptr_t)c));

	/* A: 3000 frames, varying speeds, Linux ring full now and then. */
	ppe_start();
	session_begin();
	next_token = 1;
	to_queue = 3000;
	toggle_full = 1;
	for (unsigned round = 0; ndone - session_done < 3000; round++) {
		static const unsigned w[][3] = {
			{ 4, 4, 2 }, { 6, 1, 3 }, { 6, 0, 3 }, { 1, 6, 2 },
			{ 2, 2, 2 },
		};

		assert(round < 400);
		set_weights(w[round % 5][0], w[round % 5][1], w[round % 5][2]);
		run(3000);
		/* A stale slot read behind a published head shows up as an
		 * already released token: q[3]=4.
		 */
		assert(!q[3]);
		check_order();
	}
	set_weights(4, 4, 2);
	toggle_full = 0;
	host_full = 0;
	wait_for(&q[12], 0, 100000, 1);
	check_order();
	assert(!q[3] && q[2] == 3000 && q[9] == 3000 &&
	       q[14] + q[15] == 3000 && q[15] == 1000);
	assert(refused > 0 && mem(PS_FULL) > 0 &&
	       ppe_split_ctrl()[PS_PEAK] == PPE_SPLIT_ENTRIES);
	printf("A: 3000 entries in order, %u deliveries refused and retried, "
	       "%u full-queue stops\n", refused, mem(PS_FULL));

	/* B: normal stop with frames in flight. */
	session_begin();
	to_queue = 200;
	run(1500);
	q[0] = 3;
	wait_for(&q[12], 0, 400000, 1);
	to_queue = 0;
	check_order();
	assert(ndone - session_done == npushed - session_pushed);
	for (uint32_t t = 0; t < next_token; t++)
		assert(!states[t]);
	stop_and_check();
	assert(!mem(PS_DROPPED));
	printf("B: drained stop, %u entries, q[13]=0 with an empty queue\n",
	       ndone - session_done);

	/* C: drain times out while core 1 is starved: entries left behind. */
	ppe_start();
	session_begin();
	to_queue = 400;
	host_full = 1;
	run(2000);
	set_weights(6, 0, 3);
	run(4000);
	assert(view(CORE0, &ppe_split_ctrl()[PS_HEAD]) != mem(PS_TAIL));
	q[0] = 3;
	wait_for(&q[12], 0, 2000, 0);
	assert(q[12]);
	set_weights(4, 4, 2);
	stop_and_check();
	check_order();
	host_full = 0;
	dropped = mem(PS_DROPPED);
	assert(dropped > 0);
	popped = fifo_r;
	run(3000);
	assert(fifo_r == popped);
	printf("C: timed-out drain, %u stale entries dropped before q[13]=0\n",
	       dropped);

	/* D: restart after C: Linux resets the hardware ring and the buffer
	 * states (rro_setup), no stale entry may surface.
	 */
	fifo_r = fifo_w;
	inppe_r = inppe_w;
	for (uint32_t t = 0; t < AN7552_RX_BUFFER_COUNT; t++)
		states[t] = 0;
	ppe_start();
	session_begin();
	next_token = 4000;
	to_queue = 500;
	for (unsigned n = 0; ndone - session_done < 500; n++) {
		assert(n < 200);
		run(3000);
	}
	check_order();
	for (uint32_t i = session_done; i < ndone; i++)
		assert(done[i] >= 4000);
	assert(mem(PS_DROPPED) == dropped);
	printf("D: restart, 500 new entries, no stale ones\n");

	/* E: a bad token stops the split path with q[3]=4. */
	session_begin();
	next_token = 4600;
	to_queue = 300;
	run(2000);
	hw_push(6000, 1);
	to_queue = 100;
	wait_for(&q[3], 4, 400000, 1);
	run(500);
	popped = fifo_r;
	head = view(CORE0, &ppe_split_ctrl()[PS_HEAD]);
	run(5000);
	assert(fifo_r == popped && mem(PS_HEAD) == head);
	check_order();
	assert(pushed[session_pushed + ndone - session_done] == 6000);
	assert(q[7] == (0xc0000000 | 6000));
	printf("E: bad token -> q[3]=4, pops stopped\n");

	stop_cores = 1;
	while (alive[CORE0] || alive[CORE1])
		yield();
	pthread_mutex_unlock(&mu);
	for (int c = 0; c < 2; c++)
		pthread_join(threads[c], NULL);
	puts("PASS: PPE split: exactly-once in-order pops, full ring retry, stop/restart handshake, bad token stop");
	return 0;
}

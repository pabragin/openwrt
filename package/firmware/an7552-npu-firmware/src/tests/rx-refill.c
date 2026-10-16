/* SPDX-License-Identifier: MIT */
/* Refill gate: fewer reads of the radio DMA index, same ring ownership. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "../protocol.h"
static uint32_t ring_reads;
static void barrier(void)
{
	__sync_synchronize();
}
static uint32_t read32(uint32_t address)
{
	if (address == 0x200d450c || address == 0x200d850c)
		ring_reads++;
	return *(volatile uint32_t *)(uintptr_t)address;
}
__attribute__((unused)) static uint32_t packet_word(volatile uint8_t *p)
{
	return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}
#include "../pn.c.inc"
#include "../rx-security.c.inc"
#include "../rx-parse.h"
__attribute__((unused)) static uint32_t packet_mac_low(volatile uint8_t *p)
{
	return packet_word(p);
}
__attribute__((unused)) static uint32_t packet_mac_high(volatile uint8_t *p)
{
	return p[4] | (uint32_t)p[5] << 8;
}
/* The SRAM state mirror and gate counters are mapped here. */
#define RX_HEADER_ALIAS(host) ((volatile uint8_t *)rx_words(host))
#include "../rro.c.inc"
/* Core 0 pops PPE return entries concurrently on hardware; these tests run
 * one pop inside every core 1 service call (tests/ppe-split.c interleaves
 * the two harts).
 */
#define PPE_SPLIT_CORE0_HOOK() native_ppe_pop()
#include "../ppe.c.inc"

#define N AN7552_RX_RING_COUNT
static volatile uint32_t *consumer_reg = (void *)(uintptr_t)0x200d450c;
static volatile uint32_t *q, *data;

static void map(uint32_t address, size_t length)
{
	assert(mmap((void *)(uintptr_t)address, length, PROT_READ | PROT_WRITE,
		    MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1,
		    0) == (void *)(uintptr_t)address);
}

static uint32_t room(void)
{
	return (*consumer_reg + N - data[6] - 1) % N;
}

static uint32_t refill_checked(void)
{
	uint32_t before = room(), producer = data[6], reads = ring_reads;
	uint32_t posted;

	native_rx_refill(0);
	posted = (data[6] + N - producer) % N;
	/* Never post into slots the radio still owns. */
	assert(posted <= before);
	assert(posted <= RX_REFILL_BUDGET);
	/* Posting requires a fresh read of the consumer index. */
	assert(!posted || ring_reads == reads + 1);
	assert(!q[6]);
	return posted;
}

/* The radio consumes k slots, taking their tokens; completions report
 * consumed tokens later, in order, and release them.
 */
static uint32_t fifo[N], fifo_head, fifo_tail;
static void radio_consume(uint32_t k)
{
	for (uint32_t i = 0; i < k; i++) {
		volatile uint32_t *d = rx_words(AN7552_RX_DATA_RING_HOST +
						*consumer_reg * 16);

		fifo[fifo_tail] = (d[2] >> 16) - AN7552_RX_TOKEN_BASE;
		fifo_tail = (fifo_tail + 1) % N;
		*consumer_reg = (*consumer_reg + 1) % N;
	}
}
static void complete(uint32_t k)
{
	for (uint32_t i = 0; i < k; i++) {
		uint32_t token = fifo[fifo_head];

		assert(fifo_head != fifo_tail);
		assert(token < AN7552_RX_BUFFER_COUNT && st_get(token) == 1);
		rx_refill_note(0);
		native_rx_release(token);
		fifo_head = (fifo_head + 1) % N;
	}
}

static uint32_t rng = 7;
static uint32_t rnd(uint32_t n)
{
	rng = rng * 1103515245 + 12345;
	return (rng >> 8) % n;
}

int main(void)
{
	volatile uint32_t *free_ids, *states, *ring;
	uint32_t token = 0, reads, consumed_pending = 0;

	map(0x3e900000, 0x4000);
	map(0x442c0000, 0x40000);
	map(0x200d4000, 0x5000);
	(void)native_pn_service;
	(void)native_ppe_submit;
	(void)native_rx_deliver;
	(void)native_rro_receive;
	q = (void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_RRO_CONTROL);
	data = (void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_RX_DATA_CONTROL);
	free_ids = rx_words(AN7552_RX_FREE_HOST);
	states = rx_words(AN7552_RX_STATE_HOST);
	ring = rx_words(AN7552_RX_DATA_RING_HOST);
	/* Linux setup: band 0 ring full with tokens 0..N-2, rest free. */
	for (; token < N - 1; token++) {
		ring[token * 4 + 0] = AN7552_RX_BUFFER_HOST + token * 2048;
		ring[token * 4 + 2] = (AN7552_RX_TOKEN_BASE + token) << 16;
		states[token] = 1;
	}
	for (uint32_t i = 0; i < AN7552_RX_BUFFER_COUNT; i++)
		free_ids[i] = i;
	q[20] = token;
	q[21] = 0;
	q[22] = AN7552_RX_BUFFER_COUNT - token;
	data[4] = 0x200d4500;
	data[6] = N - 1;
	data[8] = 0;
	data[11] = 1800;
	data[4 + 16] = 0x200d8500;
	data[6 + 16] = N - 1;
	memset((void *)(uintptr_t)(RX_STATE_SRAM + AN7552_RX_BUFFER_COUNT),
	       0x5a, 16);	/* SRAM is not cleared at boot */
	st_load();

	/* Full ring, no completions: no PCIe reads until the idle limit. */
	reads = ring_reads;
	for (uint32_t i = 0; i < RX_REFILL_IDLE - 1; i++)
		assert(!refill_checked());
	assert(ring_reads == reads);
	assert(!refill_checked() && ring_reads == reads + 1);

	/* Below the completion threshold nothing is read. */
	radio_consume(5);
	complete(5);
	reads = ring_reads;
	assert(!refill_checked() && ring_reads == reads);
	radio_consume(RX_REFILL_MIN - 5);
	complete(RX_REFILL_MIN - 5);
	assert(refill_checked() == RX_REFILL_MIN && ring_reads == reads + 1);

	/* Completions beyond one budget are carried to the next calls. */
	radio_consume(100);
	complete(100);
	reads = ring_reads;
	assert(refill_checked() == 32 && refill_checked() == 32 &&
	       refill_checked() == 32);
	assert(ring_reads == reads + 3);
	assert(!refill_checked() && ring_reads == reads + 3);	/* 4 owed */
	for (uint32_t i = 0; i < RX_REFILL_IDLE; i++)
		refill_checked();
	assert(room() == 0);

	/* Random radio: consumption runs ahead of completions (reordering
	 * holds buffers). The ring is never overrun and always recovers.
	 */
	for (uint32_t step = 0; step < 200000; step++) {
		uint32_t r = rnd(4), k;

		/* Alternate bursts with a near-full steady state. */
		uint32_t burst = (step / 20000) & 1 ? N : 9;

		if (r == 0) {
			k = rnd((N - 1 - room()) + 1);
			k = k < burst ? k : rnd(burst);
			radio_consume(k);
			consumed_pending += k;
		} else if (r == 1 && consumed_pending) {
			k = rnd(consumed_pending) + 1;
			complete(k);
			consumed_pending -= k;
		} else {
			refill_checked();
		}
	}
	complete(consumed_pending);
	for (uint32_t i = 0; i < 4 * RX_REFILL_IDLE && room(); i++)
		refill_checked();
	assert(room() == 0);
	/* Every token is in exactly one place: ring or free list. */
	{
		uint32_t posted = 0;

		for (uint32_t t = 0; t < AN7552_RX_BUFFER_COUNT; t++)
			posted += st_get(t) == 1;
		assert(posted == N - 1 &&
		       q[22] == AN7552_RX_BUFFER_COUNT - (N - 1));
	}
	printf("PASS: RX refill gate bounds ring reads and never overruns the ring (%u reads)\n",
	       ring_reads);
	return 0;
}

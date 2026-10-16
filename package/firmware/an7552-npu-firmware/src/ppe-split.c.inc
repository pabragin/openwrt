/* SPDX-License-Identifier: MIT */
/* PPE return queue split between the two harts.
 *
 * In mode 2 the PPE returns every forwarded frame so that the NPU can
 * recycle its receive buffer, and the frames it does not forward so that the
 * NPU can hand them to Linux. Popping one return entry costs three register
 * reads and an acknowledge write. Core 0 (lightly loaded under upload) pops
 * the entries into a single-producer single-consumer queue; core 1 consumes
 * the queue and keeps everything that touches buffer ownership: token
 * validation, st_set, free_ids/RRO q[21]/q[22], the Linux receive ring and
 * the PPE q[2]/q[9]/q[12]/q[14]/q[15] counters.
 *
 * Why the acknowledge may precede release by core 1: the acknowledge only
 * frees the PPE return-queue slot. The buffer itself is named by the token,
 * and a token returned by the PPE is in receive state 2, which only core 1
 * leaves (native_rx_release() / native_rx_deliver() -> state 0). The PPE gets
 * a buffer only through a descriptor written by native_ppe_queue(), which
 * core 1 issues for a token that went through the free list, refill and a
 * new receive (states 0 -> 1 -> 2). So after the return entry the PPE holds
 * no reference to the buffer; until core 1 consumes the entry the buffer is
 * owned by the NPU and nobody can reuse or overwrite it. The old firmware
 * relied on the same fact when it acknowledged a release-only entry.
 *
 * Placement (shared SRAM at AN7552_NPU_BASE, the block Linux sees at
 * 0x1e903000): both harts and Linux already exchange state there (core 0
 * writes the TDMA words of the core 1 block, Linux polls words of both), so
 * it is one physical memory seen by both harts. Linux zeroes 0x000-0x7ff once
 * at firmware boot and otherwise touches only 0x000-0x07f (core blocks),
 * 0x180-0x18b (TDMA), 0x1e0-0x1f7 (park, ABI), 0x200-0x27f (native TX),
 * 0x300-0x30f (forward), 0x380-0x38b (station), 0x400-0x47f (TXFREE),
 * 0x500-0x57f (RRO), 0x600-0x67f (RX data), 0x6c0-0x6ff (PPE), 0x700-0x727
 * (PN); the firmware also uses 0x280-0x2ff (profile) and 0x780-0x7bf (parse
 * counters). Free and used here:
 *   0x080-0x17f  32 entries x {status, metadata}
 *   0x680-0x6bf  control words (debugfs status row "680:")
 */
#define PPE_SPLIT_ENTRIES 32U
#define PPE_SPLIT_RING (AN7552_NPU_BASE + 0x080U)
#define PPE_SPLIT_CTRL (AN7552_NPU_BASE + 0x680U)
#define PPE_SPLIT_MAGIC 0x7552531aU
/* Control words. Each word has exactly one writing core. */
#define PS_HEAD 0	/* core 0: entries published (free running) */
#define PS_ACTIVE 1	/* core 0: 1 while it may pop */
#define PS_FULL 2	/* core 0: pop calls stopped by a full queue */
#define PS_MAGIC 3	/* core 0: PPE_SPLIT_MAGIC once running */
#define PS_TAIL 4	/* core 1: entries consumed (free running) */
#define PS_ENABLE 5	/* core 1: core 0 may pop */
#define PS_DROPPED 6	/* core 1: stale entries dropped at stop or start */
#define PS_PEAK 7	/* core 1: highest occupancy seen by the consumer */
#define PS_STOPS 8	/* core 1: completed stop handshakes */

/* Host tests route the shared accesses through a weak-memory model. */
#ifndef SPLIT_LD
#define SPLIT_LD(p) (*(p))
#define SPLIT_ST(p, v) (*(p) = (v))
#endif
#ifndef PPE_W32
#define PPE_W32(a, v) (*(volatile uint32_t *)(uintptr_t)(a) = (v))
#endif
#ifndef PPE_SPLIT_CORE0_HOOK
#define PPE_SPLIT_CORE0_HOOK() do { } while (0)
#endif

static inline volatile uint32_t *ppe_split_ctrl(void)
{
	return (volatile uint32_t *)(uintptr_t)PPE_SPLIT_CTRL;
}

static inline volatile uint32_t *ppe_split_slot(uint32_t index)
{
	return (volatile uint32_t *)(uintptr_t)PPE_SPLIT_RING +
	       (index % PPE_SPLIT_ENTRIES) * 2;
}

/* Each core clears its own words before announcing AN7552_MAGIC; Linux
 * enables the PPE only after both cores did.
 */
__attribute__((unused)) static void ppe_split_boot(uint32_t core)
{
	volatile uint32_t *s = ppe_split_ctrl();

	if (!core) {
		s[PS_HEAD] = 0;
		s[PS_ACTIVE] = 0;
		s[PS_FULL] = 0;
		s[PS_MAGIC] = PPE_SPLIT_MAGIC;
		return;
	}
	s[PS_TAIL] = 0;
	s[PS_ENABLE] = 0;
	s[PS_DROPPED] = 0;
	s[PS_PEAK] = 0;
	s[PS_STOPS] = 0;
}

/* Core 0, once per loop iteration. Never keeps an entry across calls, so
 * parking between calls cannot lose one.
 */
__attribute__((unused)) static void native_ppe_pop(void)
{
	volatile uint32_t *q = (void *)(uintptr_t)(AN7552_NPU_BASE + 0x6c0);
	volatile uint32_t *s = ppe_split_ctrl(), *slot;
	uint32_t head, tail, outstanding, status, metadata;

	if (SPLIT_LD(&q[0]) < 2 || SPLIT_LD(&q[3]) ||
	    !SPLIT_LD(&s[PS_ENABLE])) {
		if (SPLIT_LD(&s[PS_ACTIVE])) {
			/* Every published head before core 1 may see us idle. */
			barrier();
			SPLIT_ST(&s[PS_ACTIVE], 0);
		}
		return;
	}
	if (!SPLIT_LD(&s[PS_ACTIVE])) {
		/* Dekker with ppe_split_quiet(): store ACTIVE, fence, load
		 * ENABLE. Core 1 stores ENABLE=0, fences, loads ACTIVE, so at
		 * least one of us sees the other.
		 */
		SPLIT_ST(&s[PS_ACTIVE], 1);
		barrier();
		if (!SPLIT_LD(&s[PS_ENABLE])) {
			SPLIT_ST(&s[PS_ACTIVE], 0);
			return;
		}
	}
	head = SPLIT_LD(&s[PS_HEAD]);
	tail = SPLIT_LD(&s[PS_TAIL]);
	/* Frames the PPE still holds = queued (q[12], kept by core 1) minus
	 * those already in this queue. A stale view only delays a pop: core 1
	 * validates every entry against its own q[12] after its batch flush.
	 */
	outstanding = SPLIT_LD(&q[12]);
	for (uint32_t budget = 0; budget < 64; budget++) {
		if (outstanding <= head - tail)
			break;
		if (head - tail >= PPE_SPLIT_ENTRIES) {
			tail = SPLIT_LD(&s[PS_TAIL]);
			if (head - tail >= PPE_SPLIT_ENTRIES) {
				SPLIT_ST(&s[PS_FULL], SPLIT_LD(&s[PS_FULL]) + 1);
				break;
			}
		}
		/* Register order as in the single-core pop: fe4 before every
		 * entry (st2p), then status with its valid bit, then metadata.
		 */
		if (!(read32(0x1fb50fe4) & 65535))
			break;
		status = read32(0x1fb50fe0);
		if (!(status & 0x80000000))
			break;
		metadata = read32(0x1fb50fdc);
		slot = ppe_split_slot(head);
		SPLIT_ST(&slot[0], status);
		SPLIT_ST(&slot[1], metadata);
		/* The entry is visible before the head that covers it. */
		barrier();
		SPLIT_ST(&s[PS_HEAD], ++head);
		PPE_W32(0x1fb50fe0, 0x80000000);
	}
}

/* Core 1: process queued entries exactly as the single-core pop did. An
 * entry that cannot be delivered (Linux ring full) stays at the tail.
 */
static void ppe_split_consume(volatile uint32_t *q, uint32_t mode)
{
	volatile uint32_t *s = ppe_split_ctrl(), *slot;
	uint32_t head = SPLIT_LD(&s[PS_HEAD]), tail = SPLIT_LD(&s[PS_TAIL]);
	uint32_t start = tail, pending, released = 0, delivered = 0;
	uint32_t status = 0, metadata = 0, token;

	if (head == tail)
		return;
	/* Entries were published before the head that covers them. */
	barrier();
	if (head - tail > s[PS_PEAK])
		s[PS_PEAK] = head - tail;
	pending = q[12];
	for (uint32_t budget = 0; budget < 64 && tail != head; budget++) {
		slot = ppe_split_slot(tail);
		status = SPLIT_LD(&slot[0]);
		metadata = SPLIT_LD(&slot[1]);
		token = status & 65535;
		if (token >= AN7552_RX_BUFFER_COUNT ||
		    (st_get(token) & 255) != 2 || !pending) {
			q[3] = 4;
			break;
		}
		if (mode == 3 || (status & 0x40000000)) {
			native_rx_release(token);
			released++;
		} else {
			if (!native_rx_deliver(token, (metadata & 0x1fffff) |
							      (1U << 29)))
				break;
			delivered++;
		}
		tail++;
		pending--;
	}
	/* Last entry looked at, as the single-core pop left them. */
	q[7] = status;
	q[8] = metadata;
	if (tail == start)
		return;
	q[2] += tail - start;
	q[9] += tail - start;
	q[12] = pending;
	q[14] += released;
	q[15] += delivered;
	/* Slot reads complete before the slots are handed back. */
	barrier();
	SPLIT_ST(&s[PS_TAIL], tail);
}

/* Core 1: stop core 0 popping and empty the queue. Returns 1 once core 0
 * is idle and the queue is empty. Leftovers can only exist after a failed
 * drain (q[3] set or Linux timed out waiting for q[12]); they are dropped
 * like the single-core firmware abandoned unacknowledged hardware entries,
 * and rro_setup() re-initialises every buffer state before the next start.
 */
static int ppe_split_quiet(void)
{
	volatile uint32_t *s = ppe_split_ctrl();
	uint32_t head, tail;

	if (SPLIT_LD(&s[PS_ENABLE]))
		SPLIT_ST(&s[PS_ENABLE], 0);
	/* Store ENABLE before loading ACTIVE (Dekker with native_ppe_pop()). */
	barrier();
	PPE_SPLIT_CORE0_HOOK();
	if (SPLIT_LD(&s[PS_ACTIVE]))
		return 0;
	/* Core 0 fenced its last head before clearing ACTIVE. */
	barrier();
	head = SPLIT_LD(&s[PS_HEAD]);
	tail = SPLIT_LD(&s[PS_TAIL]);
	if (head != tail) {
		s[PS_DROPPED] += head - tail;
		SPLIT_ST(&s[PS_TAIL], head);
		barrier();
	}
	return 1;
}

/* Core 1, first mode >= 2 call after a start: drop anything stale, then let
 * core 0 pop. Core 0 pops only after it saw ENABLE=1, so with ACTIVE=0 the
 * head cannot move until then.
 */
static int ppe_split_start(void)
{
	volatile uint32_t *s = ppe_split_ctrl();
	uint32_t head, tail;

	if (SPLIT_LD(&s[PS_ACTIVE]))
		return 0;
	barrier();
	head = SPLIT_LD(&s[PS_HEAD]);
	tail = SPLIT_LD(&s[PS_TAIL]);
	if (head != tail) {
		s[PS_DROPPED] += head - tail;
		SPLIT_ST(&s[PS_TAIL], head);
	}
	barrier();
	SPLIT_ST(&s[PS_ENABLE], 1);
	return 1;
}

/* Core 1 part of native_ppe_service(). Returns 0 only in mode 1, once core 0
 * is quiet, so that the single-submission path runs unchanged.
 *
 * Linux stop (native_ppe_stop): q[0]=3 and wait for q[12]==0, then q[0]=0
 * and wait for q[13]==0. In mode 3 core 0 keeps popping and core 1 releases
 * everything; q[12] reaches 0 only when core 1 consumed every entry. q[13]=0
 * is written only after core 0 confirmed it stopped and the queue is empty.
 */
static int ppe_split_service(volatile uint32_t *q)
{
	uint32_t mode = q[0];

	if (!mode) {
		if (ppe_split_quiet()) {
			if (q[13])
				ppe_split_ctrl()[PS_STOPS]++;
			barrier();
			q[13] = 0;
		}
		return 1;
	}
	q[13] = 1;
	if (mode < 2)
		return !ppe_split_quiet();
	if (!SPLIT_LD(&ppe_split_ctrl()[PS_ENABLE]) && !ppe_split_start())
		return 1;
	PPE_SPLIT_CORE0_HOOK();
	if (!q[3])
		ppe_split_consume(q, mode);
	return 1;
}

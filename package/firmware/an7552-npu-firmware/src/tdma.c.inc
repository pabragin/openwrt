/* SPDX-License-Identifier: MIT */
#define TDMA_BATCH 32U

/* Direct transmit buffers: the 2048 ring buffers armed by Linux plus the
 * unused tail of the packet workspace up to the TXWI area.
 */
#define TX_POOL_BUFFERS 3040U
#define TX_BUFFER_HOST(id) (AN7552_TDMA_BUFFER_HOST + (id) * AN7552_TDMA_BUFFER_SIZE)
#define TX_BUFFER_NPU(id) (AN7552_TDMA_BUFFER_NPU + (id) * AN7552_TDMA_BUFFER_SIZE)

static int tdma_buffer_valid(uint32_t address)
{
	uint32_t offset = address - AN7552_TDMA_BUFFER_HOST;

	return address >= AN7552_TDMA_BUFFER_HOST &&
	       !(offset & (AN7552_TDMA_BUFFER_SIZE - 1)) &&
	       offset / AN7552_TDMA_BUFFER_SIZE < TX_POOL_BUFFERS;
}

/* Core 0 consumes received frames and, with forwarding enabled, also owns
 * slot refill (it swaps in a free buffer from its pool), so the TDMA ring has
 * a single owner and a slot is free again as soon as its frame is queued to
 * the radio rather than when the radio completes it. Without forwarding,
 * slots are re-armed in place. Status words stay in the core 1 state block.
 */
static void tdma_receive(volatile uint32_t *state, uint32_t *consumer,
			 uint32_t *reclaim, uint32_t band)
{
	volatile uint32_t *enable =
		(void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_TDMA_CONTROL);
	volatile uint32_t *forward =
		(void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_FORWARD_CONTROL);
	volatile uint32_t *work = 0;
	volatile uint32_t *descriptor;
	uint32_t control, length, last = 0, returned = 0, forwarding;
	uint32_t registers = 0x1fb50900 + band * 16;

	if (!*enable) {
		*consumer = 0;
		*reclaim = 0;
		state[12] &= ~(1U << band);
		return;
	}
	if (!(state[12] & (1U << band))) {
		*consumer = read32(registers + 12) % AN7552_TDMA_COUNT;
		*reclaim = *consumer;
	}
	state[12] |= 1U << band;
	forwarding = forward[0] && (forward[1] == 2 || forward[1] == band);
	for (uint32_t n = 0; n < TDMA_BATCH; n++) {
		if (forwarding) {
			work = (void *)(uintptr_t)(AN7552_FORWARD_WORK_NPU +
						   (band * AN7552_TDMA_COUNT +
						    *consumer) *
							   16);
			if (work[0])
				break;
		}
		descriptor =
			(void *)(uintptr_t)(AN7552_TDMA_RING_NPU +
					    (band * AN7552_TDMA_COUNT +
					     *consumer) *
						    32);
		control = descriptor[1];
		if (!(control & 0x80000000))
			break;
		barrier();
		length = control & 0xffff;
		if (!length || length > AN7552_TDMA_BUFFER_SIZE ||
		    !tdma_buffer_valid(descriptor[2])) {
			state[14]++;
			*enable = 0;
			break;
		}
		state[5]++;
		state[7] = length;
		if (forwarding) {
			work[1] = length;
			work[2] = descriptor[4];
			work[3] = descriptor[6] >> 24;
			barrier();
			work[0] = 1;
			forward[8]++;
			*consumer = (*consumer + 1) % AN7552_TDMA_COUNT;
			continue;
		}
		state[8] = descriptor[4];
		state[9] = descriptor[5];
		state[10] = descriptor[6];
		state[11] = descriptor[7];
		descriptor[1] = AN7552_TDMA_BUFFER_SIZE;
		last = *consumer;
		returned = 1;
		*consumer = (*consumer + 1) % AN7552_TDMA_COUNT;
	}
	if (returned) {
		barrier();
		*(volatile uint32_t *)(uintptr_t)(registers + 8) = last;
	}
}

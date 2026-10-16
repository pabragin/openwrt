/* SPDX-License-Identifier: MIT */
#include <stdint.h>
#include "protocol.h"
#include "firmware-abi.h"

static inline void barrier(void)
{
	__asm__ volatile("fence iorw, iorw" ::: "memory");
}

static uint32_t read32(uint32_t address)
{
	return *(volatile uint32_t *)(uintptr_t)address;
}

#include "tdma.c.inc"
#include "station.c.inc"
#include "tx-pool.c.inc"
#include "native-tx.c.inc"
#include "txfree.c.inc"
#include "pn.c.inc"
#include "rx-security.c.inc"
#include "rx-parse.h"
#include "rro.c.inc"
#include "ppe.c.inc"

void firmware_main(uint32_t core)
{
	volatile uint32_t *state =
		(void *)(uintptr_t)(AN7552_NPU_BASE + core * AN7552_STRIDE);
	volatile uint32_t *tdma_state =
		(void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_STRIDE);
	uint32_t heartbeat = 0;
	uint32_t consumer[2] = { 0 }, reclaim[2] = { 0 };
	uint32_t direct_consumer[2] = { 0 }, direct_active[2] = { 0 };

	state[1] = 0;
	state[2] = 0;
	state[3] = 0;
	state[4] = 0;
	*(volatile uint32_t *)(uintptr_t)(AN7552_NPU_BASE +
					  AN7552_FW_RUNTIME_ABI + core * 4) =
		AN7552_FW_ABI_VERSION;
	if (!core)
		tx_pool_boot();
	barrier();
	state[0] = AN7552_MAGIC;
	for (;;) {
		if (read32(AN7552_NPU_BASE + AN7552_PARK_CONTROL)) {
			barrier();
			*(volatile uint32_t *)(uintptr_t)(AN7552_NPU_BASE +
							  AN7552_PARK_ACK +
							  core * 4) =
				AN7552_PARK_MAGIC;
			barrier();
			for (;;)
				__asm__ volatile("wfi");
		}
		state[1] = ++heartbeat;
		if (core) {
			native_pn_service();
			native_ppe_service();
			if (read32(AN7552_NPU_BASE + AN7552_STATION_CONTROL)) {
				barrier();
				*(volatile uint32_t
					  *)(uintptr_t)(AN7552_NPU_BASE +
							AN7552_STATION_CONTROL +
							8) = 1;
				continue;
			}
			*(volatile uint32_t
				  *)(uintptr_t)(AN7552_NPU_BASE +
						AN7552_STATION_CONTROL + 8) = 0;
			/* Core 1 owns the whole Wi-Fi -> Ethernet direction. */
			native_rro_receive();
			continue;
		}
		native_txfree(0);
		native_txfree(1);
		if (read32(AN7552_NPU_BASE + AN7552_STATION_CONTROL)) {
			barrier();
			*(volatile uint32_t
				  *)(uintptr_t)(AN7552_NPU_BASE +
						AN7552_STATION_CONTROL + 4) = 1;
			continue;
		}
		*(volatile uint32_t *)(uintptr_t)(AN7552_NPU_BASE +
						  AN7552_STATION_CONTROL + 4) =
			0;
		native_transmit(0);
		native_transmit(1);
		/* Core 0 owns the whole Ethernet -> Wi-Fi direction: TDMA ingress,
		 * direct transmit and completion reclaim. The TDMA status words keep
		 * their Linux-visible home in the core 1 block (offsets 5..14); core 1
		 * only writes words 0..4 there, so each word has a single writer.
		 */
		for (uint32_t band = 0; band < 2; band++) {
			tdma_receive(tdma_state, &consumer[band],
				     &reclaim[band], band);
			barrier();
			*(volatile uint32_t *)(uintptr_t)(AN7552_NPU_BASE +
							  AN7552_TDMA_CONTROL +
							  4 + band * 4) =
				consumer[band];
			native_forward(&direct_consumer[band],
				       &direct_active[band], band);
		}
	}
}

/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "../protocol.h"
static void barrier(void)
{
	__sync_synchronize();
}
static uint32_t read32(uint32_t address)
{
	return *(volatile uint32_t *)(uintptr_t)address;
}
#include "../tdma.c.inc"
#include "../tx-pool.c.inc"
#include "../txfree.c.inc"
int main(void)
{
	(void)native_txfree;
	(void)tdma_receive;
	(void)tx_pool_arm;
	(void)tx_pool_stop;
	uint32_t q[16] = { 0 };
	void *npu = mmap((void *)(uintptr_t)AN7552_NPU_BASE, 4096,
			 PROT_READ | PROT_WRITE,
			 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	void *pool = mmap((void *)(uintptr_t)TX_POOL_ADDR, 0xc000,
			  PROT_READ | PROT_WRITE,
			  MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	void *txwi = mmap((void *)(uintptr_t)AN7552_TXWI_NPU, 0x40000,
			  PROT_READ | PROT_WRITE,
			  MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	volatile uint32_t *forward =
		(void *)(uintptr_t)(AN7552_NPU_BASE + AN7552_FORWARD_CONTROL);
	struct tx_pool *p;
	uint32_t packet[8] = { (6U << 27) | (2U << 16), 5U << 16, 1U << 31,
			       1U << 30, 5 | (2048U << 15) };

	assert(npu == (void *)(uintptr_t)AN7552_NPU_BASE);
	assert(pool == (void *)(uintptr_t)TX_POOL_ADDR);
	assert(txwi == (void *)(uintptr_t)AN7552_TXWI_NPU);
	tx_pool_boot();
	tx_pool_start(1, 0);
	p = tx_pool();
	assert(p->free_tokens == TX_POOL_TOKENS && p->free_buffers == 992);
	/* Token 5 is queued to the radio with buffer 2100 on band 1. */
	p->token_buffer[5] = TX_TOKEN_VALID | TX_TOKEN_BAND | 2100;
	p->inflight[1] = 1;
	p->free_tokens--;
	p->free_buffers--;
	assert(txfree_walk((void *)packet, 20, q, 0) == 0);
	packet[4] = 6 | (2048U << 15);
	assert(txfree_walk((void *)packet, 20, q, 0) == 5);
	packet[4] = 5 | (2048U << 15);
	assert(!txfree_walk((void *)packet, 20, q, 1));
	tx_pool_publish(p);
	assert(q[8] == 1 && q[9] == 1 && forward[9] == 1);
	assert(!tx_pool_token_busy(5) && p->inflight[1] == 0);
	assert(p->free_tokens == TX_POOL_TOKENS && p->free_buffers == 992);
	assert(p->buffer[991] == 2100 && p->token[2047] == 5);
	assert(txfree_walk((void *)packet, 20, q, 1) == 5);
	assert(txfree_walk((void *)packet, 20, q, 0) == 5);
	p->token_buffer[5] = TX_TOKEN_VALID | 7;
	for (uint32_t length = 0; length < 20; length++)
		assert(txfree_walk((void *)packet, length, q, 0));
	packet[0] = (6U << 27) | (1U << 16);
	assert(txfree_walk((void *)packet, 20, q, 0) == 4);
	packet[4] = 5 | (32767U << 15);
	assert(!txfree_walk((void *)packet, 20, q, 0));
	packet[4] = 16384 | (32767U << 15);
	assert(txfree_walk((void *)packet, 20, q, 0) == 4);
	packet[4] = 5 | (32767U << 15);
	packet[1] = 4U << 16;
	assert(txfree_walk((void *)packet, 20, q, 0) == 2);
	packet[1] = 7U << 16;
	packet[3] = 1U << 31;
	assert(!txfree_walk((void *)packet, 20, q, 0));
	puts("PASS: TXFREE bounds, counts, versions, host/direct tokens and double completion");
	return 0;
}

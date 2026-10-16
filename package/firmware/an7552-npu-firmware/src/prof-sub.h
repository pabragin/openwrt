/* SPDX-License-Identifier: MIT */
/* Optional sub-section profile (make SUBPROF=1, implies PROF=1).
 *
 * Laps and event counters live in local SRAM at PSUB_SRAM (free space after
 * the receive buffer states and the refill gate; Linux never touches it).
 * prof.h folds them into 64-bit sums and publishes them in the spare words
 * of the 280:/2c0: debugfs lines.
 *
 *   SRAM word  owner   meaning                         published as
 *   0          core 1  native_rx_refill x2              2c0 [4]
 *   1          core 1  RX descriptor read + checks      2c0 [9]
 *   2          core 1  saved[0..3] + security view      2c0 [10]
 *   3          core 1  parse + identity + state         2c0 [11]
 *   4          core 1  fast-path recheck                2c0 [12]
 *   5          core 1  PPE queue or host delivery       2c0 [13]
 *   6          core 1  batch setup, flush, retries      2c0 [14]
 *   8          core 0  band 1: radio ring full breaks   280 [13]
 *   9          core 0  band 1: token/in-flight breaks   280 [14]
 *   10..14     core 0  upload front half (pipeline)     4c0 [2..6]
 *
 * In pipeline mode (upsplit.c.inc) core 1 words 0, 5 and 6 keep their
 * meaning; 1..4 are the back half's parts of the same work:
 *   1  record read, buffer state check, refill note
 *   2  saved[1..5] writes for frames queued to the PPE
 *   3  station identity check (moved here from core 0's parse)
 *   4  key word + epoch/cookie check (the old fast recheck)
 * Core 0 words: 10 RX descriptor read, 11 security view, 12 parse +
 * classify, 13 record write + head publish, 14 batch setup/flush/EMI.
 */
#ifndef AN7552_PROF_SUB_H
#define AN7552_PROF_SUB_H

#ifdef NPU_SUBPROF
#define PSUB_SRAM 0x3e902000U
#define PSUB_WORDS 16U

static inline uint32_t psub_cycle(void)
{
	uint32_t c;

	__asm__ volatile("csrr %0, mcycle" : "=r"(c) : : "memory");
	return c;
}

static inline volatile uint32_t *psub_words(void)
{
	return (volatile uint32_t *)(uintptr_t)PSUB_SRAM;
}

#define PSUB_START(t) uint32_t t = psub_cycle()
#define PSUB_LAP(t, slot)                                                    \
	do {                                                                 \
		uint32_t psub_n_ = psub_cycle();                             \
		psub_words()[slot] += psub_n_ - (t);                         \
		(t) = psub_n_;                                               \
	} while (0)
#define PSUB_RESET(t) ((t) = psub_cycle())
#define PSUB_COUNT(slot) (psub_words()[slot]++)
#else
#define PSUB_RESET(t) do { } while (0)
#define PSUB_START(t) do { } while (0)
#define PSUB_LAP(t, slot) do { } while (0)
#define PSUB_COUNT(slot) do { } while (0)
#endif

#endif

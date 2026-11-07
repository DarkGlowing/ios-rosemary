#pragma once
#include <stdint.h>

/*
 * Collect the panic log a previous boot's XNU kernel left in DRAM and copy it
 * to the start of expdb and of boot_b (LBA+0 record, LBA+8 log for each).
 *
 * xnu patch 0088 makes osfmk/arm64/rosemary_early.c mirror its console text
 * into a static __DATA buffer:
 *   [0..7]   "RXNULOG1"
 *   [8..11]  length, little endian
 *   [16..]   the text
 *
 * A canary is written to DRAM on every boot and read back on the next one, so
 * a miss can be told apart from the machine simply losing memory across reset.
 *
 * Returns the log length when something was stored (>0), -1 on a miss.
 */
int xnulog_collect(uintptr_t phys_base, uintptr_t mem_size);

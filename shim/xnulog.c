/*
 * Collect the XNU early-panic log that the previous boot left in DRAM.
 *
 * This is deliberately the xnu12 behaviour, which is the only variant proven
 * to move data on this device:
 *   - write a boot record at LBA+0 of each target and the log at LBA+8;
 *   - targets are the START of expdb and the START of boot_b (both were read
 *     back successfully once);
 *   - on success reboot immediately with the hardware reset so LK boots us
 *     again and the cycle stays short (a long wait lets the slot fail over to
 *     Android, which wipes everything we care about);
 *   - on a miss, just boot XNU as usual.
 *
 * The canary answers the separate question of whether DRAM survives the reset
 * at all: it is written at phys_base+512MiB (outside the payload) and read
 * back one boot later.
 *
 * Runs with the MMU off, exactly like the rest of the loader path: the buffer
 * address is physical, the UFS descriptors are physical and the caches are
 * off, so no cache maintenance is needed around the DMA.
 */
#include <stdint.h>
#include "console.h"
#include "platform_mt6785.h"
#include "ufs.h"
#include "xnulog.h"

#define LOG_MAGIC		"RXNULOG1"
#define LOG_MAGIC_U64		0x31474f4c554e5852ULL
#define LOG_MAX			(32u * 1024u)
#define LOG_HDR			16u	/* magic, then length in LE32, then text */
#define LOG_SCAN_BYTES		(64u * 1024u * 1024u)
#define LOG_CHUNK		4096u	/* one UFS driver block */

/* _rosemary_log is a static 32 KiB buffer in the kernel's __DATA:
 * VA 0xfffffe0007c4b000, virt_base 0xfffffe0006000000 -> +0x1c4b000. */
#define LOG_EXPECTED_OFF	0x1c4b000u

/* Written outside the ~40MiB payload, read back on the next boot. */
#define CANARY_OFF		0x20000000u
#define CANARY_MAGIC		0x314e4143584d4948ULL	/* "HIMXCAN1" */

static const uint32_t target_lbas[] = {
	776u,			/* expdb */
	578304u,		/* boot_b, partition start */
};
#define NTARGETS (sizeof(target_lbas) / sizeof(target_lbas[0]))

static char rec[4096] __attribute__((aligned(4096)));

static uint32_t
rd32(const volatile uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static const volatile uint8_t *
find_log(uintptr_t phys_base, uintptr_t mem_size)
{
	uintptr_t span = mem_size < LOG_SCAN_BYTES ? mem_size : LOG_SCAN_BYTES;

	for (uintptr_t a = phys_base; a + LOG_HDR < phys_base + span; a += 8) {
		/* Caches are off here, so every byte costs a bus transaction:
		 * reject on a single 64-bit load first. */
		if (*(const volatile uint64_t *)a != LOG_MAGIC_U64) {
			continue;
		}
		uint32_t len = rd32((const volatile uint8_t *)a + 8);
		if (len > 0 && len < LOG_MAX) {
			return (const volatile uint8_t *)a;
		}
	}
	return 0;
}

static uint32_t
canary_touch(uintptr_t phys_base)
{
	volatile uint64_t *c = (volatile uint64_t *)(phys_base + CANARY_OFF);

	if (c[0] == CANARY_MAGIC) {
		uint32_t n = (uint32_t)c[1];

		c[1] = (uint64_t)(n + 1);
		c[2] = (uint64_t)(~((n + 1) & 0xffffffffu) & 0xffffffffu);
		return (c[2] == (uint64_t)(~((n + 1) & 0xffffffffu) & 0xffffffffu)) ? n : 1;
	}
	c[0] = CANARY_MAGIC;
	c[1] = 1;
	c[2] = (uint64_t)(~1u & 0xffffffffu);
	return 0;
}

static int
write_boot_record(uint32_t lba, int found, uint32_t len, uint32_t canary,
    uintptr_t phys_base, uintptr_t span)
{
	static const char hdr[] = "SHIMBOOT4\n";
	const volatile uint8_t *buf = (const volatile uint8_t *)(phys_base + LOG_EXPECTED_OFF);

	for (unsigned int i = 0; i < sizeof(rec); i++) {
		rec[i] = 0;
	}
	unsigned int n = 0;
	for (unsigned int i = 0; i < sizeof(hdr) - 1; i++) {
		rec[n++] = hdr[i];
	}
#define PUTS(s) do { for (const char *q = (s); *q; q++) rec[n++] = *q; } while (0)
#define PUTX(tag, v) do { \
	PUTS(tag); \
	for (int sh = 60; sh >= 0; sh -= 4) \
		rec[n++] = "0123456789abcdef"[((uint64_t)(v) >> sh) & 0xf]; \
} while (0)
	PUTX("canary=", (uint64_t)canary);
	PUTX(" found=", (uint64_t)found);
	PUTX(" len=", (uint64_t)len);
	PUTS("\n");
	PUTX("phys_base=", phys_base);
	PUTX(" span=", span);
	PUTS("\n  buf:");
	for (unsigned int i = 0; i < 32; i++) {
		uint8_t b = buf[i];
		rec[n++] = "0123456789abcdef"[b >> 4];
		rec[n++] = "0123456789abcdef"[b & 0xf];
		if ((i & 3) == 3)
			rec[n++] = ' ';
	}
	PUTS("\n");
#undef PUTX
#undef PUTS
	return ufs_write_raw(lba, rec, 4096);
}

int
xnulog_collect(uintptr_t phys_base, uintptr_t mem_size)
{
	uintptr_t span = mem_size < LOG_SCAN_BYTES ? mem_size : LOG_SCAN_BYTES;
	uint32_t canary = canary_touch(phys_base);
	const volatile uint8_t *p = find_log(phys_base, mem_size);
	uint32_t len = p ? rd32(p + 8) : 0;
	int stored = 0;

	for (unsigned int t = 0; t < NTARGETS; t++) {
		if (write_boot_record(target_lbas[t], p != 0, len, canary, phys_base, span)) {
			continue;
		}
		if (p) {
			uint32_t total = (LOG_HDR + len + LOG_CHUNK - 1) & ~(LOG_CHUNK - 1);

			for (uint32_t off = 0; off < total; off += LOG_CHUNK) {
				if (ufs_write_raw(target_lbas[t] + 8 + off / LOG_CHUNK,
				    (const void *)(p + off), LOG_CHUNK)) {
					break;
				}
			}
			stored++;
		}
	}

	con_hex2("CANARY", "prev=", canary, "found=", (uint32_t)(p != 0));
	if (p) {
		/* Retire the log so the next boot boots XNU instead of dumping again. */
		volatile uint8_t *q = (volatile uint8_t *)p;

		for (unsigned int i = 0; i < 8; i++) {
			q[i] = 0;
		}
		con_hex2("XNULOG", "len=", len, "copies=", (uint32_t)stored);
		return (int)len;
	}
	con_str("XNULOG", "no xnu log in dram");
	return -1;
}

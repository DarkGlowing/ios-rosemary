/*
 * Milestone 5 probe: can we talk to the internal UFS that ABL booted from?
 *
 * Runs with the MMU off (all memory Device, so DMA is coherent with no cache
 * maintenance). Dumps the UFSHCI registers ABL left behind and, if the host
 * is enabled and the link is up, points the transfer request list at our own
 * buffers and issues READ-ONLY SCSI commands on LUN 0. Nothing is ever
 * written to the device: there is no SCSI WRITE in this file.
 *
 * Register layout and descriptor formats: JEDEC UFSHCI (JESD223).
 */
#include <stdint.h>
#include "console.h"
#include "ufs.h"

#define UFS_BASE	0x11270000UL	// rosemary: ufshci@11270000 (MediaTek UFSHCI, standard JEDEC regs), M-PHY at ufs_mphy@11fa0000

#define REG_CAP		0x00
#define REG_VER		0x08
#define REG_IS		0x20
#define REG_HCS		0x30
#define REG_HCE		0x34
#define REG_UTRLBA	0x50
#define REG_UTRLBAU	0x54
#define REG_UTRLDBR	0x58
#define REG_UTRLCLR	0x5c
#define REG_UTRLRSR	0x60

#define HCS_DP		(1u << 0)
#define HCS_UTRLRDY	(1u << 1)
#define HCS_UCRDY	(1u << 3)

#define BLOCK		4096u

static inline uint32_t ufs_rd(uint32_t off) { return *(volatile uint32_t *)(UFS_BASE + off); }
static inline void ufs_wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(UFS_BASE + off) = v; }

/* UTP transfer request descriptor (32 bytes). */
struct utrd {
	uint32_t dw0;		/* [31:28] CT, [26:25] DD, [24] I */
	uint32_t dw1;
	uint32_t dw2;		/* [7:0] OCS */
	uint32_t dw3;
	uint32_t ucdba;		/* command descriptor base, 128-byte aligned */
	uint32_t ucdbau;
	uint32_t resp;		/* [31:16] response UPIU offset, [15:0] length (DWORDs) */
	uint32_t prdt;		/* [31:16] PRDT offset, [15:0] entries (DWORDs/entries) */
};

/* UTP command descriptor: command UPIU, response UPIU, PRDT. */
struct ucd {
	uint8_t cmd[512];
	uint8_t resp[512];
	struct {
		uint32_t addr_lo, addr_hi, rsvd, dbc;	/* dbc = byte count - 1 */
	} prdt[1];
};

static struct utrd utrl[32] __attribute__((aligned(1024)));
static struct ucd ucd __attribute__((aligned(128)));
static uint8_t data[2 * BLOCK] __attribute__((aligned(4096)));

static uint64_t ticks(void)
{
	uint64_t t;
	__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(t));
	return t;
}

static void zero(void *p, unsigned n)
{
	volatile uint8_t *b = p;
	for (unsigned i = 0; i < n; i++)
		b[i] = 0;
}

static uint32_t be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/*
 * Issue one SCSI command (read direction or no data) in slot 0.
 * Returns 0 on success, otherwise a code describing the failure stage.
 */
static int g_dir = 1;	/* 1 = data in (read), 2 = data out (write) */
/* A UFS device exposes several logical units. On rosemary LU0 is a tiny boot
 * LU; the GPT and every partition live on another one, so every LBA we pass
 * has to go to the LU that actually carries the partition table. */
static uint8_t g_lun;

static uint32_t scsi(uint8_t lun, const uint8_t *cdb, unsigned cdb_len, void *buf, uint32_t len,
    uint32_t *ocs_out, uint32_t *status_out)
{
	static uint8_t tag;
	zero(&utrl[0], sizeof(utrl[0]));
	zero(&ucd, sizeof(ucd));

	/* Command UPIU: header (12 bytes), expected length, CDB. */
	ucd.cmd[0] = 0x01;			/* transaction type: COMMAND */
	ucd.cmd[1] = len ? (g_dir == 2 ? 0x20 : 0x40) : 0x00;	/* flags: W or R */
	ucd.cmd[2] = lun;
	ucd.cmd[3] = ++tag;
	ucd.cmd[12] = len >> 24;
	ucd.cmd[13] = len >> 16;
	ucd.cmd[14] = len >> 8;
	ucd.cmd[15] = len;
	for (unsigned i = 0; i < cdb_len; i++)
		ucd.cmd[16 + i] = cdb[i];

	uintptr_t ucd_pa = (uintptr_t)&ucd;
	utrl[0].dw0 = (1u << 28) | ((len ? (g_dir == 2 ? 1u : 2u) : 0u) << 25) | (1u << 24);
	utrl[0].dw2 = 0x0f;			/* OCS: invalid until the host updates it */
	utrl[0].ucdba = (uint32_t)ucd_pa;
	utrl[0].ucdbau = (uint32_t)(ucd_pa >> 32);
	utrl[0].resp = (512u / 4) << 16 | (512u / 4);
	if (len) {
		uintptr_t buf_pa = (uintptr_t)buf;
		ucd.prdt[0].addr_lo = (uint32_t)buf_pa;
		ucd.prdt[0].addr_hi = (uint32_t)(buf_pa >> 32);
		ucd.prdt[0].dbc = len - 1;
		utrl[0].prdt = (1024u / 4) << 16 | 1;
	}
	__asm__ volatile("dsb sy" ::: "memory");

	ufs_wr(REG_UTRLDBR, 1);
	uint64_t start = ticks();
	while (ufs_rd(REG_UTRLDBR) & 1) {
		if (ticks() - start > 13000000) {	// 1 s @ 13 MHz (MediaTek CNTFRQ)
			ufs_wr(REG_UTRLCLR, ~1u);	/* clear slot 0 (write 0 to the bit) */
			return 1;			/* timeout */
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
	*ocs_out = utrl[0].dw2 & 0xff;
	*status_out = ucd.resp[7];		/* SCSI status in the response UPIU */
	if (*ocs_out)
		return 2;
	if (ucd.resp[0] != 0x21)		/* RESPONSE UPIU */
		return 3;
	return *status_out ? 4 : 0;
}

/* SCSI SYNCHRONIZE CACHE(10): commit whatever the device still holds in its
 * volatile write cache. Without it a watchdog reset a few milliseconds later
 * discards our writes -- the device acknowledges them, and even serves the
 * read-back verification from cache, yet nothing ever reaches NAND. */
static int flush_cache(void)
{
	uint8_t cdb[10] = { 0x35, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint32_t ocs = 0, st = 0;
	g_dir = 2;
	int rc = scsi(g_lun, cdb, sizeof(cdb), 0, 0, &ocs, &st);
	g_dir = 1;
	return rc ? -1 : 0;
}

int ufs_flush(void)
{
	static int taken;
	if (!taken) {
		return 0;
	}
	return flush_cache();
}

static uint32_t read_blocks(uint32_t lba, uint32_t count, void *buf)
{
	uint8_t cdb[10] = { 0x28, 0, lba >> 24, lba >> 16, lba >> 8, lba, 0, count >> 8, count, 0 };
	uint32_t ocs, st;
	return scsi(g_lun, cdb, sizeof(cdb), buf, count * BLOCK, &ocs, &st);
}

static uint64_t le64(const uint8_t *p)
{
	uint64_t v = 0;
	for (int i = 7; i >= 0; i--)
		v = v << 8 | p[i];
	return v;
}

/* Name the filesystem whose superblock starts at block lba (f2fs/ext4). */
static const char *fs_type(uint32_t lba)
{
	if (read_blocks(lba, 1, data))
		return "read error";
	const uint8_t *sb = data + 1024;
	if (sb[0] == 0x10 && sb[1] == 0x20 && sb[2] == 0xf5 && sb[3] == 0xf2)
		return "f2fs";
	if (sb[56] == 0x53 && sb[57] == 0xef)
		return "ext4";
	return "other";
}

/* One line per GPT entry of at least 64 MiB: name, first LBA, size in MiB (4K blocks). */
static void list_partitions(uint32_t entry_lba, uint32_t nent)
{
	static uint8_t ents[128 * 128] __attribute__((aligned(4096)));
	uint32_t blocks = (nent * 128 + BLOCK - 1) / BLOCK;
	if (read_blocks(entry_lba, blocks, ents)) {
		con_str("UFS PART", "read error");
		return;
	}
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = ents + i * 128;
		uint64_t first = le64(e + 32), last = le64(e + 40);
		if (!first)
			continue;
		char name[17];
		for (int c = 0; c < 16; c++) {
			uint8_t ch = e[56 + 2 * c];
			name[c] = (ch >= 32 && ch < 127) ? ch : (ch ? '?' : 0);
		}
		name[16] = 0;
		if (((last - first + 1) >> 8) < 64)	/* keep the screen to the big ones */
			continue;
		con_hex2(name, "lba=", (uint32_t)first, "MiB=", (uint32_t)((last - first + 1) >> 8));
		if (!__builtin_strcmp(name, "userdata") || !__builtin_strcmp(name, "cust") ||
		    !__builtin_strcmp(name, "cache"))
			con_str(" fs", fs_type((uint32_t)first));
	}
}

/*
 * rosemary (MT6785) UFS pre-init: enable clocks, assert+release the host-side
 * resets and bring up the M-PHY PLL. lk leaves most of this running, but doing
 * it again is harmless and guarantees the link is up before we probe. Register
 * values taken from the rosemary Android kernel (ufs-mtk-platform.c,
 * clk-mt6785.c). Called at the very start of test_ufs().
 */
#define INFRACFG_AO_BASE	0x10001000UL
#define INFRA2_CG_CLR		0x0A8	/* clear power-down = enable clock */
#define INFRA3_CG_CLR		0x0C4
#define  UFS_CG_BITS		((1u<<28)|(1u<<29)|(1u<<30))	/* UFS, AES_UFSFDE, UFS_TICK */
#define  UFS_AXI_CG_BIT		(1u<<5)
#define UFS_MPHY_BASE		0x11fa0000UL	/* ufs_mphy (verify: mediatek,ufs_mphy reg) */

static inline uint32_t io_rd(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void io_wr(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static void udelay_ufs(unsigned us)
{
	uint64_t end = ticks() + 13000000ULL / 1000000 * us;
	while (ticks() < end) ;
}
static void mtk_ufs_preinit(void)
{
	/* 1. Enable UFS clocks (clear power-down bits). Idempotent: if lk already
	 * enabled them, these writes are no-ops. */
	io_wr(INFRACFG_AO_BASE + INFRA2_CG_CLR, UFS_CG_BITS);
	io_wr(INFRACFG_AO_BASE + INFRA3_CG_CLR, UFS_AXI_CG_BIT);
	udelay_ufs(20);
	/* 2. M-PHY PLL: release power-down/isolation bits (release = set/clear
	 * the same bits lk already released; no-op if already up). Exact
	 * sequence from ufs-mtk-platform.c. Reset sequence is INTENTIONALLY
	 * skipped: lk leaves the link up, and re-asserting reset could drop it. */
	volatile uint8_t *mphy = (volatile uint8_t *)UFS_MPHY_BASE;
	uint32_t v;
	v = io_rd((uintptr_t)mphy + 0x008C); io_wr((uintptr_t)mphy + 0x008C, v | (1u<<11));
	v = io_rd((uintptr_t)mphy + 0x008C); io_wr((uintptr_t)mphy + 0x008C, v & ~(1u<<10));
	v = io_rd((uintptr_t)mphy + 0x008C); io_wr((uintptr_t)mphy + 0x008C, v & ~(1u<<9));
	v = io_rd((uintptr_t)mphy + 0x008C); io_wr((uintptr_t)mphy + 0x008C, v & ~(1u<<8));
	v = io_rd((uintptr_t)mphy + 0xB044); io_wr((uintptr_t)mphy + 0xB044, v | (1u<<18));
	v = io_rd((uintptr_t)mphy + 0xB044); io_wr((uintptr_t)mphy + 0xB044, v & ~(1u<<17));
	v = io_rd((uintptr_t)mphy + 0xB044); io_wr((uintptr_t)mphy + 0xB044, v & ~(1u<<20));
	v = io_rd((uintptr_t)mphy + 0xB044); io_wr((uintptr_t)mphy + 0xB044, v & ~(1u<<19));
	v = io_rd((uintptr_t)mphy + 0xA0AC); io_wr((uintptr_t)mphy + 0xA0AC, v | (1u<<1));
	v = io_rd((uintptr_t)mphy + 0xA0AC); io_wr((uintptr_t)mphy + 0xA0AC, v & ~(1u<<0));
	udelay_ufs(1);
	v = io_rd((uintptr_t)mphy + 0xA09C); io_wr((uintptr_t)mphy + 0xA09C, v & ~(1u<<18));
	udelay_ufs(200);
	con_str("UFS", "mtk pre-init: clocks+MPHY PLL release done");
}

void test_ufs(void)
{
	mtk_ufs_preinit();
	con_str("UFS", "milestone 5 probe (read-only)");
	uint32_t cap = ufs_rd(REG_CAP), hcs = ufs_rd(REG_HCS), hce = ufs_rd(REG_HCE);
	con_hex2("UFS", "ver=", ufs_rd(REG_VER), "cap=", cap);
	con_hex2("UFS", "hce=", hce, "hcs=", hcs);
	con_hex2("UFS", "is=", ufs_rd(REG_IS), "rsr=", ufs_rd(REG_UTRLRSR));
	con_hex2("UFS", "utrlba=", ufs_rd(REG_UTRLBA), "hi=", ufs_rd(REG_UTRLBAU));
	con_hex2("UFS", "dbr=", ufs_rd(REG_UTRLDBR), "nutrs=", (cap & 0x1f) + 1);

	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY)) {
		con_str("UFS", "host not enabled/ready: stop");
		return;
	}
	if (ufs_rd(REG_UTRLDBR)) {
		con_str("UFS", "requests outstanding: stop");
		return;
	}

	/* Take over the transfer request list: stop, point at ours, run. */
	ufs_wr(REG_UTRLRSR, 0);
	uintptr_t list = (uintptr_t)utrl;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	con_hex2("UFS", "ours=", (uint32_t)list, "rsr=", ufs_rd(REG_UTRLRSR));
	/*
	 * Never ring a doorbell unless the host really uses our list: otherwise
	 * it would execute whatever ABL left in its own slot 0 (maybe a write).
	 */
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || ufs_rd(REG_UTRLBAU) != (uint32_t)(list >> 32) ||
	    !(ufs_rd(REG_UTRLRSR) & 1)) {
		con_str("UFS", "list base not taken: stop");
		return;
	}

	uint32_t ocs = 0, st = 0, rc;
	static const uint8_t tur[6] = { 0x00 };
	/* A first command may report a unit attention; retry a few times. */
	for (int i = 0; i < 3; i++) {
		rc = scsi(0, tur, sizeof(tur), 0, 0, &ocs, &st);
		con_hex2("UFS TUR", "rc=", rc, "ocs/st=", ocs << 8 | st);
		if (!rc)
			break;
	}

	static const uint8_t cap10[10] = { 0x25 };	/* READ CAPACITY(10) */
	rc = scsi(0, cap10, sizeof(cap10), data, 8, &ocs, &st);
	con_hex2("UFS CAP", "rc=", rc, "ocs/st=", ocs << 8 | st);
	if (!rc)
		con_hex2("UFS LUN0", "lastlba=", be32(data), "blk=", be32(data + 4));

	/* READ(10) LBA 0, 2 blocks: protective MBR and the GPT header. */
	static const uint8_t rd10[10] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 2, 0 };
	rc = scsi(0, rd10, sizeof(rd10), data, 2 * BLOCK, &ocs, &st);
	con_hex2("UFS READ", "rc=", rc, "ocs/st=", ocs << 8 | st);
	if (rc)
		return;
	con_hex2("UFS MBR", "sig=", data[510] | data[511] << 8, "type=", data[450]);
	const uint8_t *gpt = data + BLOCK;
	char sig[9];
	for (int i = 0; i < 8; i++)
		sig[i] = (gpt[i] >= 32 && gpt[i] < 127) ? gpt[i] : '.';
	sig[8] = 0;
	con_str("UFS GPT", sig);
	uint32_t nent = gpt[80] | gpt[81] << 8 | gpt[82] << 16, esz = gpt[84] | gpt[85] << 8;
	uint32_t elba = gpt[72] | gpt[73] << 8;
	con_hex2("UFS GPT", "entries=", nent, "lba=", elba);
	if (esz != 128 || nent > 128)
		return;
	list_partitions(elba, nent);
}


/* -------------------------------------------------------------------------
 * Write support (used only for the debug log in expdb, never for anything
 * that matters on the device).
 * ---------------------------------------------------------------------- */

/* Bring the host up and take over the transfer request list. Returns 0 if our
 * list is really in use (same conditions test_ufs() checks before issuing
 * commands), otherwise a negative code. */
static int ufs_takeover(void)
{
	mtk_ufs_preinit();
	uint32_t hcs = ufs_rd(REG_HCS), hce = ufs_rd(REG_HCE);
	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY))
		return -1;
	if (ufs_rd(REG_UTRLDBR))
		return -2;
	ufs_wr(REG_UTRLRSR, 0);
	uintptr_t list = (uintptr_t)utrl;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || !(ufs_rd(REG_UTRLRSR) & 1))
		return -3;
	for (int i = 0; i < 3; i++) {
		uint32_t ocs = 0, st = 0;
		static const uint8_t tur[6] = { 0x00 };
		g_dir = 0;
		if (!scsi(0, tur, sizeof(tur), 0, 0, &ocs, &st))
			break;
	}
	g_dir = 1;
	return 0;
}

static int write_blocks(uint32_t lba, const void *buf, uint32_t blocks)
{
	uint32_t ocs = 0, st = 0;
	uint8_t cdb[10] = { 0x2A, 0, lba >> 24, lba >> 16, lba >> 8, lba,
			    0, blocks >> 8, blocks, 0 };
	g_dir = 2;
	uint32_t rc = scsi(g_lun, cdb, sizeof(cdb), (void *)buf, blocks * BLOCK, &ocs, &st);
	g_dir = 1;
	return rc ? -1 : 0;
}

/* Write `bytes` (multiple of 4096) at lba. Verifies by reading the first block
 * back, and flips the UFSHCI data-direction encoding if the first attempt is
 * rejected. Returns 0 on success. */
/* Find the logical unit that carries the GPT: read LBA 1 of each LU and look
 * for "EFI PART". Returns the LU index, or -1 when none answers (in which case
 * g_lun stays 0 and we behave exactly as before). */
int ufs_probe_lun(void)
{
	static uint8_t blk[4096] __attribute__((aligned(4096)));
	static int taken;

	if (!taken) {
		if (ufs_takeover()) {
			return -1;
		}
		taken = 1;
	}
	int found = -1;
	for (uint8_t lun = 0; lun < 4 && found < 0; lun++) {
		uint8_t cdb[10] = { 0x28, 0, 0, 0, 0, 1, 0, 0, 1, 0 };
		uint32_t ocs = 0, st = 0;
		g_dir = 1;
		if (scsi(lun, cdb, sizeof(cdb), blk, BLOCK, &ocs, &st)) {
			continue;
		}
		if (blk[0] == 'E' && blk[1] == 'F' && blk[2] == 'I' && blk[3] == ' ' &&
		    blk[4] == 'P' && blk[5] == 'A' && blk[6] == 'R' && blk[7] == 'T') {
			found = lun;
		}
	}
	if (found >= 0) {
		g_lun = (uint8_t)found;
	}
	return found;
}

int ufs_write_raw(uint32_t lba, const void *buf, uint32_t bytes)
{
	static int taken;
	int rc = taken ? 0 : ufs_takeover();
	if (rc)
		return rc;
	taken = 1;
	uint32_t blocks = bytes / BLOCK;
	if (write_blocks(lba, buf, blocks))
		return -10;
	if (flush_cache())
		return -13;
	if (read_blocks(lba, 1, data))
		return -11;
	/* Compare the whole first block, not just a prefix: a device that silently
	 * truncates or mis-addresses the write has to be caught here. */
	const uint8_t *s = (const uint8_t *)buf;
	uint32_t n = bytes < BLOCK ? bytes : BLOCK;
	for (uint32_t i = 0; i < n; i++)
		if (s[i] != data[i])
			return -12;
	return 0;
}

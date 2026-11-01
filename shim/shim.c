/*
 * Milestone 2: one boot answers the questions that decide the XNU port.
 *   - CPU feature registers (16K granule, GIC sysreg interface)
 *   - PSCI/SMCCC version and which PSCI calls the Qualcomm TZ offers
 *   - SMP: PSCI CPU_ON for the 7 secondaries, each reports its MPIDR
 *   - GICv3 Group 1 IRQ from the virtual timer (INTID 19 on Qualcomm)
 * Any unexpected exception paints a blue panic screen.
 */
#include <stdint.h>
#include "console.h"
#include "mmu.h"
#include "loader.h"
#include "ufs.h"
#include "platform_mt6785.h"	/* MT6785: WDT pet + EMI restore (rosemary fixes) */

#define IMEM_RESTART_REASON	0UL	// removed: Qualcomm-only IMEM (MediaTek uses PSCI reset)
#define RESTART_BOOTLOADER	0x77665500U
#define PSHOLD			0UL	// removed: Qualcomm-only PSHOLD (MediaTek uses PSCI reset)

#define GICD_BASE		0x0c000000UL
#define GICR_BASE		0x0c040000UL	/* boot_a DTB: /interrupt-controller reg, 2nd tuple (GICR) */
#define GICR_SIZE		0x200000UL	/* boot_a DTB: Redistributor window size */
#define GICR_PE_SIZE		0x20000UL
#define GICR_TYPER		0x08
#define GICR_WAKER		0x14
#define GICR_SGI		0x10000
#define GICR_IGROUPR0		(GICR_SGI + 0x080)
#define GICR_ISENABLER0		(GICR_SGI + 0x100)
#define GICR_ICENABLER0		(GICR_SGI + 0x180)
#define GICR_IPRIORITYR		(GICR_SGI + 0x400)
#define GICR_ISPENDR0		(GICR_SGI + 0x200)
#define GICR_ISACTIVER0		(GICR_SGI + 0x300)
#define GICR_ICFGR1		(GICR_SGI + 0xc04)
#define VTIMER_INTID		27	/* MediaTek MT6785: standard PPI 11 -> INTID 27. Dump FDT: verify /proc/device-tree/timer interrupts */
#define PTIMER_INTID		30	/* MediaTek: non-secure physical timer PPI 14 -> INTID 30 (verify in FDT) */
#define SGI_INTID		1

#define PSCI_VERSION		0x84000000UL
#define PSCI_CPU_SUSPEND	0xc4000001UL
#define PSCI_CPU_OFF		0x84000002UL
#define PSCI_CPU_ON		0xc4000003UL
#define PSCI_MIGRATE_INFO_TYPE	0x84000006UL
#define PSCI_SYSTEM_RESET	0x84000009UL
#define PSCI_FEATURES		0x8400000aUL
#define PSCI_SYSTEM_SUSPEND	0xc400000eUL
#define PSCI_SYSTEM_RESET2	0xc4000012UL
#define SMCCC_VERSION		0x80000000UL

#define NR_CPUS			8

#define rsr(r) ({ uint64_t _v; __asm__ volatile("mrs %0, " r : "=r"(_v)); _v; })
#define wsr(r, v) __asm__ volatile("msr " r ", %0" : : "r"((uint64_t)(v)))
#define isb() __asm__ volatile("isb" ::: "memory")

static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline uint64_t rd64(uintptr_t a) { return *(volatile uint64_t *)a; }
static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void wr8(uintptr_t a, uint8_t v) { *(volatile uint8_t *)a = v; }

extern char secondary_entry[];

static const char *stage;	/* set at runtime: no absolute pointers in data */
static volatile uint64_t sec_mpidr[NR_CPUS];
static volatile uint64_t sec_probe[4];	/* CPU1 redistributor write test */
static volatile uint32_t irq_count, irq_other, irq_last;
static volatile int irq_src;

/* ICC_SGI1R_EL1 value targeting the calling core only. */
static uint64_t sgi_self(unsigned id)
{
	uint64_t m = rsr("mpidr_el1");
	return ((m >> 16) & 0xff) << 32 |	/* Aff2 */
	       ((m >> 8) & 0xff) << 16 |	/* Aff1 */
	       (uint64_t)id << 24 |
	       (1U << (m & 0xf));		/* target list: Aff0 */
}

static uint64_t smc(uint64_t fn, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = fn;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;
	__asm__ volatile("smc #0"
			 : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			 :
			 : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11",
			   "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static uint64_t now(void)
{
	isb();
	return rsr("cntpct_el0");
}

/*
 * Timer-independent, bounded wait.
 *
 * The original loop was `while (now() < end)` with end derived from
 * cntfrq_el0. If the firmware never programmed CNTFRQ_EL0 (it reads as 0 at
 * handoff on some MTK/ATF combinations) every delay becomes zero-length, so
 * the shim paints one frame, immediately "holds 30 s" for 0 ms, resets, and
 * the phone reboots 5-7 s after power-on with a black screen — exactly the
 * symptom we are chasing. A stuck counter would instead hang forever.
 *
 * So: wait until the timer says we are done OR a fixed iteration budget is
 * spent, whichever comes first. Bounded in both directions.
 */
#define DEFAULT_FRQ	13000000ULL	/* MediaTek default arm,cpu-regs clock */
#define SPIN_PER_MS	1500000ULL	/* loop iterations per ms (floor) */

static void delay_ms(unsigned ms)
{
	uint64_t f = rsr("cntfrq_el0");
	if (!f)
		f = DEFAULT_FRQ;
	uint64_t per_ms = f / 1000;
	if (!per_ms)
		per_ms = 1;
	uint64_t end = now() + per_ms * (uint64_t)ms;
	uint64_t spin = (uint64_t)ms * SPIN_PER_MS;
	uint64_t next_pet = now() + per_ms * 200;	/* pet WDT every ~200 ms */

	while (spin) {
		if (now() >= end)	/* timer is sane: done */
			break;
		if (now() >= next_pet) {
			wdt_pet();
			next_pet = now() + per_ms * 200;
		}
		spin--;
		__asm__ volatile("yield" ::: "memory");
	}
	wdt_pet();
}

static void __attribute__((noreturn)) reset(unsigned after_s)
{
	con_skip();
	con_str("RESET", after_s > 20 ? "in 40s" : after_s > 2 ? "in 20s" : "now");
	delay_ms(after_s * 1000);
	// DRAM/EMI "restore" is disabled: the register values in emi_restore()
	// are speculative (writing DRAMC_ACTIM0 = 1 can wedge the DRAM), and the
	// phone already reboots cleanly into LK/Android after a hard reset, so it
	// is not needed. Keep the function for later, do not call it here.
	// emi_restore();
	// MediaTek: no Qualcomm IMEM restart_reason; PSCI reset below
	__asm__ volatile("dsb sy" ::: "memory");
	smc(PSCI_SYSTEM_RESET, 0, 0, 0);
	// If PSCI reset is not wired up on this LK/ATF combination, fall back to
	// the top RGU software-reset register (always works, hard SoC reset).
	delay_ms(2000);
	wdt_swreset();
}

/* ---- exceptions ---- */

static void irq_handler(void)
{
	uint64_t iar = rsr("icc_iar1_el1");
	uint32_t id = iar & 0xffffff;

	if (id >= 1020 && id <= 1023)
		return;	/* spurious */
	irq_last = id;
	if (id == (uint32_t)irq_src)
		irq_count++;
	else
		irq_other++;
	if (id == VTIMER_INTID)
		wsr("cntv_tval_el0", rsr("cntfrq_el0") / 100);	/* 10 ms */
	else if (id == PTIMER_INTID)
		wsr("cntp_tval_el0", rsr("cntfrq_el0") / 100);
	wsr("icc_eoir1_el1", iar);
	isb();
}

static volatile int in_exc;

void exc_handler(uint64_t slot, uint64_t *regs)
{
	if (slot == 5) {	/* IRQ from current EL, SPx */
		irq_handler();
		return;
	}
	/* A fault inside the panic painter (e.g. the framebuffer range is not
	 * accessible at all) must not recurse forever: skip the console and go
	 * straight to a hard reset, which the timing test can still detect. */
	if (in_exc) {
		delay_ms(500);
		wdt_swreset();
	}
	in_exc = 1;
	/* Do not clear: keep the console output (JUMP/BOOTARG/SEGS values) on
	 * screen and print the panic in a coloured band below it. */
	con_set_dy(0);
	con_set_bg(0xff300000U);
	con_set_row(26);
	switch (slot & 3) {
	case 0: con_str("PANIC", "SYNC"); break;
	case 1: con_str("PANIC", "IRQ"); break;
	case 2: con_str("PANIC", "FIQ"); break;
	default: con_str("PANIC", "SERROR"); break;
	}
	con_hex2("SLOT", "n=", slot, "cpu=", rsr("mpidr_el1"));
	con_str("STAGE", stage);
	con_hex("ESR", rsr("esr_el1"));
	con_hex("ELR", rsr("elr_el1"));
	con_hex("FAR", rsr("far_el1"));
	con_hex("SPSR", rsr("spsr_el1"));
	con_hex("X0", regs[0]);
	con_hex("X1", regs[1]);
	con_hex("X30", regs[29]);	/* stashed lr is just above the frame */
	reset(40);
}

/* ---- SMP ---- */

static uintptr_t find_gicr(void);

void secondary_main(uint64_t idx)
{
	sec_mpidr[idx] = rsr("mpidr_el1") | (1ULL << 63);
	if (idx == 1) {
		uintptr_t r = find_gicr();
		if (r) {
			sec_probe[0] = rd32(r + GICR_IGROUPR0);
			wr32(r + GICR_IGROUPR0, 0xffffffff);
			sec_probe[1] = rd32(r + GICR_IGROUPR0);
			wr8(r + GICR_IPRIORITYR + 19, 0xa0);
			sec_probe[2] = rd32(r + GICR_IPRIORITYR + 16);
		}
		sec_probe[3] = r ? 1 : 0xdead;
	}
	__asm__ volatile("dsb sy" ::: "memory");
	smc(PSCI_CPU_OFF, 0, 0, 0);
}

static void __attribute__((unused)) test_smp(void)
{
	stage = "smp";
	uint64_t self = rsr("mpidr_el1") & 0xffffff;
	int64_t ret[NR_CPUS];

	for (int i = 1; i < NR_CPUS; i++)
		ret[i] = smc(PSCI_CPU_ON, (uint64_t)i << 8, (uintptr_t)secondary_entry, i);
	delay_ms(500);
	for (int i = 1; i < NR_CPUS; i++) {
		char label[] = "CPU0";
		label[3] = '0' + i;
		if (((uint64_t)i << 8) == self)
			con_str(label, "is boot cpu");
		else
			con_hex2(label, "ret=", (uint32_t)ret[i], "mpidr=",
				 (uint32_t)sec_mpidr[i] | (sec_mpidr[i] ? 0 : 0xdead0000));
	}
}

/* ---- GIC + timer ---- */

static uintptr_t find_gicr(void)
{
	uint64_t aff = rsr("mpidr_el1");
	uint32_t want = (aff & 0xffffff) | ((aff >> 8) & 0xff000000);

	for (uintptr_t off = 0; off < GICR_SIZE; off += GICR_PE_SIZE) {
		uint64_t typer = rd64(GICR_BASE + off + GICR_TYPER);
		if ((uint32_t)(typer >> 32) == want)
			return GICR_BASE + off;
		if (typer & (1 << 4))	/* Last */
			break;
	}
	return 0;
}

static void gicd_wait_rwp(void)
{
	for (int i = 0; i < 1000000 && (rd32(GICD_BASE) & (1U << 31)); i++)
		;
}

/* Run one interrupt source for ~300 ms and report what the GIC saw. */
static void irq_window(const char *label, uintptr_t r, int src)
{
	uint64_t f = rsr("cntfrq_el0");
	uint32_t pend = 0, act = 0, isr = 0, hppi = 0x3ff, ist = 0;

	irq_count = irq_other = 0;
	irq_last = 0x3ff;
	irq_src = src;
	if (src == VTIMER_INTID) {
		wsr("cntv_tval_el0", f / 100);
		wsr("cntv_ctl_el0", 1);
	} else if (src == PTIMER_INTID) {
		wsr("cntp_tval_el0", f / 100);
		wsr("cntp_ctl_el0", 1);
	}
	isb();
	/* sample state with IRQs still masked, after the first period */
	uint64_t t = now() + f / 50;
	while (now() < t)
		;
	if (src == SGI_INTID)
		wsr("icc_sgi1r_el1", sgi_self(SGI_INTID));
	isb();
	ist = src == PTIMER_INTID ? rsr("cntp_ctl_el0") : rsr("cntv_ctl_el0");
	pend = rd32(r + GICR_ISPENDR0);
	act = rd32(r + GICR_ISACTIVER0);
	isr = rsr("isr_el1");
	hppi = rsr("icc_hppir1_el1");

	__asm__ volatile("msr daifclr, #2" ::: "memory");
	t = now() + f * 3 / 10;
	/* SGI: one already sent above, send 9 more 10 ms apart -> expect 10 */
	uint64_t next = now() + f / 100;
	for (int sent = 1; now() < t;)
		if (src == SGI_INTID && sent < 10 && now() >= next) {
			wsr("icc_sgi1r_el1", sgi_self(SGI_INTID));
			isb();
			sent++;
			next += f / 100;
		}
	__asm__ volatile("msr daifset, #2" ::: "memory");
	wsr("cntv_ctl_el0", 0);
	wsr("cntp_ctl_el0", 0);
	isb();

	con_hex2(label, "irqs=", irq_count, "other=", irq_other);
	con_hex2(" pre", "pend=", pend, "act=", act);
	con_hex2(" pre", "isr=", isr, "hppi=", hppi);
	con_hex2(" pre", "tctl=", ist, "last=", irq_last);
}

static void test_gic(void)
{
	stage = "gic sre";
	uint64_t sre = rsr("icc_sre_el1");
	if (!(sre & 1)) {
		wsr("icc_sre_el1", sre | 1);
		isb();
	}
	if (!(rsr("icc_sre_el1") & 1)) {
		con_str("GIC", "no sysreg iface, skip");
		return;
	}

	stage = "gic dist";
	uint32_t dctlr = rd32(GICD_BASE);
	/* Non-secure view (DS=0): bit0 EnableGrp1, bit1 EnableGrp1A, bit4 ARE_NS; Linux writes 0x13 */
	wr32(GICD_BASE, dctlr | 0x13);
	gicd_wait_rwp();
	con_hex2("GICD", "was=", dctlr, "now=", rd32(GICD_BASE));

	stage = "gic redist";
	uintptr_t r = find_gicr();
	if (!r) {
		con_str("GICR", "not found");
		return;
	}
	uint32_t waker = rd32(r + GICR_WAKER);
	if (waker & 2) {
		wr32(r + GICR_WAKER, waker & ~2U);
		for (int i = 0; i < 100000 && (rd32(r + GICR_WAKER) & 4); i++)
			;
	}
	con_hex2("GICR", "base=", r, "waker=", rd32(r + GICR_WAKER));

	stage = "gic config";
	uint32_t grp = rd32(r + GICR_IGROUPR0);
	wr32(r + GICR_IGROUPR0, 0xffffffff);	/* NS view: secure bits are WI */
	uint32_t en_before = rd32(r + GICR_ISENABLER0);
	uint32_t pri_before = rd32(r + GICR_IPRIORITYR + 16);	/* INTIDs 16..19 */
	for (int id = 0; id < 32; id++)
		wr8(r + GICR_IPRIORITYR + id, 0xa0);
	wr32(r + GICR_ISENABLER0, (1U << VTIMER_INTID) | (1U << PTIMER_INTID) | (1U << SGI_INTID));
	con_hex2("ENABLE", "was=", en_before, "now=", rd32(r + GICR_ISENABLER0));
	(void)grp;
	(void)pri_before;

	uint64_t ctlr = rsr("icc_ctlr_el1");
	wsr("icc_ctlr_el1", ctlr & ~2ULL);	/* EOImode 0: EOIR also deactivates */
	wsr("icc_pmr_el1", 0xf0);
	wsr("icc_bpr1_el1", 0);
	wsr("icc_igrpen1_el1", 1);
	isb();
	con_hex2("ICC", "ctlr=", ctlr, "grpen1=", rsr("icc_igrpen1_el1"));

	stage = "gic vtimer";
	irq_window("VTIMER", r, VTIMER_INTID);
	stage = "gic sgi";
	irq_window("SGI1", r, SGI_INTID);
	wr32(r + GICR_ICENABLER0, 0xffffffff);
	isb();
}

/* ---- main ---- */

static void __attribute__((unused)) psci_line(const char *label, uint64_t fn)
{
	con_hex2(label, "feat=", smc(PSCI_FEATURES, fn, 0, 0), 0, 0);
}

static uint64_t scratch[8192];	/* 64KB, lives in our (later cacheable) image */

static uint64_t ticks_fill(uint32_t c)
{
	uint64_t t = now();
	con_clear(c);
	return now() - t;
}

static uint64_t ticks_mem(void)
{
	uint64_t t = now();
	for (int pass = 0; pass < 16; pass++)
		for (int i = 0; i < 8192; i++)
			scratch[i] = scratch[i] * 3 + i;
	return now() - t;
}

static uint32_t to_us(uint64_t ticks)
{
	return ticks * 10 / 130;	/* 13 MHz (MediaTek MT6785 CNTFRQ; verify cntfrq_el0) */
}

void shim_main(uint64_t dtb, uint64_t el)
{
	struct mmu_info mi;

#ifdef SHIM_BARE
	/* Minimal "did we even get control?" build: paint the screen, pet the
	 * watchdog forever, never reset. If the phone stays alive here, LK did
	 * jump to us and the diagnostic reset in the normal build is what makes
	 * the 5-7 s reboot. If it still reboots, LK never reached our entry. */
	con_clear(GREEN);
	con_marks(WHITE);
	con_str("SHIM", "bare build: alive, petting WDT, no reset");
	for (;;) {
		wdt_pet();
		delay_ms(200);
	}
#endif

	/* Draw the first diagnostic frame before touching the MTK watchdog.
	 * If WDT setup itself hangs or resets the SoC, the framebuffer test is
	 * still visible and the failure is distinguishable from a display fault. */
	stage = "framebuffer";
#ifdef SHIM_CONSOLE
	/* Plain console: solid fill + text. This combination renders perfectly on
	 * rosemary (verified on hardware). */
	con_set_dy(0);
	con_clear(0xff000040U);
	con_set_row(1);
	con_hex2("CPU", "el=", (uint32_t)el, "dtb=", (uint32_t)dtb);
	con_hex2("TIMER", "frq=", (uint32_t)rsr("cntfrq_el0"), "cnt=", (uint32_t)now());
	con_hex2("WDT", "mode=", wdt_mode(), "len=", wdt_length());
	con_str("SHIM", "rosemary console ok, reset in 20s");
	delay_ms(20000);
	reset(1);
#endif
#ifdef SHIM_LOG
	/* Log channel that does not depend on the display at all: a text report
	 * plus a sample of both framebuffer halves is written into the expdb
	 * partition (GPT: LBA 776, 20 MiB). Read it back with dd from Android or
	 * recovery and analyse offline. */
	{
		static char lb[16384] __attribute__((aligned(4096)));
		int flat = 0;
		int pdet = con_detect_pitch(1, &flat);
		int n = 0;
		#define P(s) do { for (const char *q = (s); *q; q++) lb[n++] = *q; } while (0)
		#define H(v) do { for (int sh = 28; sh >= 0; sh -= 4) \
			lb[n++] = "0123456789ABCDEF"[((uint32_t)(v) >> sh) & 0xf]; } while (0)
		P("ROSEMARYLOG0001\n");
		P("el="); H(el); P(" dtb="); H((uint32_t)dtb); P("\n");
		P("cntfrq="); H((uint32_t)rsr("cntfrq_el0"));
		P(" cntpct="); H((uint32_t)now()); P("\n");
		P("wdt_mode="); H(wdt_mode()); P(" wdt_len="); H(wdt_length()); P("\n");
		P("sctlr_el1="); H(rsr("sctlr_el1")); P(" mpidr="); H(rsr("mpidr_el1")); P("\n");
		P("fb_base=7BD20000 pitch_detect="); H(pdet); P(" flat="); H(flat); P("\n");
		P("fb_first16=");
		for (int i = 0; i < 16; i++) { H(((volatile uint8_t *)FB_BASE)[i]); P(" "); }
		P("\n");
		int rc1 = ufs_write_raw(776, lb, 16384);
		/* 1 MiB from each 2400-row half (64 KiB per UFS command) */
		int rc2 = 0;
		for (int i = 0; i < 16 && !rc2; i++)
			rc2 = ufs_write_raw(800 + i * 16, (void *)(FB_BASE + (uintptr_t)i * 0x10000), 0x10000);
		int rc3 = 0;
		for (int i = 0; i < 16 && !rc3; i++)
			rc3 = ufs_write_raw(1100 + i * 16,
				(void *)(FB_BASE + 2400UL * 1088UL * 4UL + (uintptr_t)i * 0x10000), 0x10000);
		/* rewrite the header with the results */
		n = 0;
		P("ROSEMARYLOG0001\n");
		P("write rc hdr="); H(rc1); P(" fb0="); H(rc2); P(" fb1="); H(rc3); P("\n");
		P("el="); H(el); P(" pitch_detect="); H(pdet); P(" flat="); H(flat); P("\n");
		ufs_write_raw(776, lb, 16384);
		#undef P
		#undef H
		con_clear(BLUE);
		con_str("LOG", "report written to expdb lba 776");
		con_hex2("LOG", "rc=", (uint32_t)((rc1 & 0xff) << 16 | (rc2 & 0xff) << 8 | (rc3 & 0xff)),
			 "pitch=", (uint32_t)pdet);
		delay_ms(20000);
		reset(1);
	}
#endif
#ifdef SHIM_AUTO
	/* Minimal console: black background + text, line pitch auto-detected from
	 * the boot logo LK left in the framebuffer. */
	{
		int flat = 0;
		int p = con_detect_pitch(1, &flat);
		con_set_pitch(p);
		con_paint_buf(1, 0);
		con_set_dy(FB_H);
		con_set_row(2);
		con_hex2("PITCH", "detected=", (uint32_t)p, "flat=", (uint32_t)flat);
		con_hex2("CPU", "el=", (uint32_t)el, "dtb=", (uint32_t)dtb);
		con_hex2("TIMER", "frq=", (uint32_t)rsr("cntfrq_el0"), "cnt=", (uint32_t)now());
		con_hex2("WDT", "mode=", wdt_mode(), "len=", wdt_length());
		con_str("SHIM", "rosemary console, black background");
		con_str("HOLD", "reset in 30s");
		con_mirror_first();
		delay_ms(30000);
		reset(1);
	}
#endif
#ifdef SHIM_PITCHSCAN
	/* Eight labels, eight pitches, one boot. The crisp line names the pitch. */
	static const int cand[8] = { 1080, 1088, 1024, 1056, 1104, 1120, 1152, 960 };
	static const char *lab[8] = { "P1080", "P1088", "P1024", "P1056",
				      "P1104", "P1120", "P1152", "P0960" };
	static const uint32_t col[8] = { WHITE, RED, GREEN, YELLOW,
					 0xff00ffffU, 0xffff00ffU, 0xff4080ffU, 0xffa0a0a0U };
	con_paint_buf(0, 0xff000020U);
	con_paint_buf(1, 0xff000020U);
	for (int i = 0; i < 8; i++)
		con_label_pitch(lab[i], 120 + i * 280, cand[i], col[i]);
	con_set_dy(FB_H);
	con_set_row(0);
	con_str("SCAN", "8 pitches, read the crisp line");
	con_set_dy(0);
	delay_ms(30000);
	reset(1);
#endif
#ifdef SHIM_SIMPLE
	/* Pragmatic console: solid fill + text in BOTH buffers, nothing else.
	 * Solid fills and glyphs render cleanly on this panel; fine horizontal
	 * structures (rulers, bars) come out smeared, so we simply do not use
	 * them. This is all a bring-up console needs. */
	con_paint_buf(0, 0xff000020U);
	con_paint_buf(1, 0xff000020U);
	con_set_dy(0);
	con_set_row(4);
	con_hex2("CPU", "el=", (uint32_t)el, "dtb=", (uint32_t)dtb);
	con_hex2("TIMER", "frq=", (uint32_t)rsr("cntfrq_el0"), "cnt=", (uint32_t)now());
	con_hex2("FB", "w=", FB_W, "stride=", FB_STRIDE);
	con_hex2("WDT", "mode=", wdt_mode(), "len=", wdt_length());
	con_str("SHIM", "rosemary console on buffer 0");
	con_set_dy(FB_H);
	con_set_row(4);
	con_hex2("CPU", "el=", (uint32_t)el, "dtb=", (uint32_t)dtb);
	con_hex2("TIMER", "frq=", (uint32_t)rsr("cntfrq_el0"), "cnt=", (uint32_t)now());
	con_hex2("FB", "w=", FB_W, "stride=", FB_STRIDE);
	con_hex2("WDT", "mode=", wdt_mode(), "len=", wdt_length());
	con_str("SHIM", "rosemary console on buffer 1");
	con_set_dy(0);
	con_str("HOLD", "solid+text test, reset in 30s");
	delay_ms(30000);
	reset(1);
#endif
#ifdef SHIM_GEOM2
	/* The panel scans buffer 1 (verified: painting #1 blue turned the screen
	 * blue). Draw everything there. */
	con_geom2(1);
	con_set_dy(FB_H);
	con_set_row(6);
	con_hex2("CPU", "el=", (uint32_t)el, "frq=", (uint32_t)rsr("cntfrq_el0"));
	con_str("GEOM2", "9 rulers, centre line, corner squares");
	con_str("HOLD", "buffer 1 frame, reset in 30s");
	con_mirror_first();	/* keep buffer 0 identical */
	delay_ms(30000);
	reset(1);
#endif
#ifdef SHIM_BUFID
	/* Which of the two 2400-row buffers does the panel actually scan?
	 * Buffer 0 red, buffer 1 blue, each labelled inside itself. */
	con_paint_buf(0, RED);
	con_paint_buf(1, 0xff0000ffU);
	con_set_dy(0);
	con_set_row(6);
	con_hex2("BUF0", "el=", (uint32_t)el, "frq=", (uint32_t)rsr("cntfrq_el0"));
	con_str("BUF0", "this buffer is RED  (paint #0)");
	con_set_dy(FB_H);
	con_set_row(6);
	con_hex2("BUF1", "el=", (uint32_t)el, "frq=", (uint32_t)rsr("cntfrq_el0"));
	con_str("BUF1", "this buffer is BLUE (paint #1)");
	con_set_dy(0);
	con_str("HOLD", "buffer id test: reset in 30s");
	delay_ms(30000);
	reset(1);
#endif
	con_set_dy(0);
	con_clear(0xff000040U);	/* plain console: the combination that works */
	con_set_row(1);
	con_hex2("CPU", "el=", (uint32_t)el, "dtb=", (uint32_t)dtb);
	con_hex2("FB", "w=", FB_W, "stride=", FB_STRIDE);
	con_hex2("FBM", "h=", FB_H, "base=", (uint32_t)FB_BASE);
	con_hex2("TIMER", "frq=", (uint32_t)rsr("cntfrq_el0"), "cnt=", (uint32_t)now());
	con_str("SHIM", "rosemary milestone 4: xnu loader");
	con_str("FB", "early framebuffer write complete");
	__asm__ volatile("dsb sy" ::: "memory");

	/* MT6785 rosemary: extend A7 WDT timeout and pet it before slow probes. */
	stage = "watchdog";
	con_hex2("WDT", "mode=", wdt_mode(), "len=", wdt_length());
	wdt_init_long_timeout();
	con_hex2("WDT", "now=", wdt_mode(), "len2=", wdt_length());
	con_skip();

	/* An xnu payload in the ramdisk means: boot it (MMU stays off). */
	stage = "loader";
	const char *why = loader_boot(dtb);
	con_str("LOADER", why);
	con_skip();

	/* No payload: diagnostic build. Hold this frame, then reset.
	 * The reboot *delay* is the blind debug channel: a reboot ~30 s after
	 * power-on proves this code ran; a reboot after 5-7 s means LK never
	 * handed control to us (image rejected / AVB / decompression). */
	stage = "hold";
	con_str("HOLD", "no xnu payload: reset in 30s");
	con_str("BUF", "mirroring frame into the second mtkfb buffer");
	con_mirror_second();
	delay_ms(30000);
	reset(1);

	stage = "fill off";
	uint64_t fill_off = ticks_fill(RED);
	uint64_t mem_off = ticks_mem();

	stage = "mmu enable";
	int rc = mmu_enable(&mi);

	stage = "fill on";
	uint64_t fill_on = rc ? 0 : ticks_fill(RED);
	uint64_t mem_on = rc ? 0 : ticks_mem();
	if (rc)
		con_clear(RED);

	con_str("SHIM", "rosemary milestone 3");
	con_skip();
	con_hex2("DTB", "at=", dtb, "magic=", dtb ? __builtin_bswap32(rd32(dtb)) : 0);
	con_hex2("IMAGE", "lo=", mi.img_lo, "hi=", mi.img_hi);
	con_hex2("MMU", "rc=", rc, "l3=", mi.n_l3);
	con_hex2("TTBR0", "pa=", mi.ttbr, "pa_hi=", mi.ttbr >> 32);
	con_hex("SCTLR0", mi.sctlr_before);
	con_hex("SCTLR1", mi.sctlr_after);
	con_hex("TCR", rsr("tcr_el1"));
	con_hex2("PARANGE", "mmfr0=", rsr("id_aa64mmfr0_el1") & 0xf, "tg16=", (rsr("id_aa64mmfr0_el1") >> 20) & 0xf);
	con_skip();
	con_hex("PAR img", mmu_probe((uintptr_t)shim_main));
	con_hex("PAR fb", mmu_probe(FB_PHYS + 0x1000));
	con_hex("PAR gic", mmu_probe(GICD_BASE));
	con_hex("PAR 12G", mmu_probe(0x300000000UL));
	con_skip();
	con_hex2("FILL us", "off=", to_us(fill_off), "on=", to_us(fill_on));
	con_hex2("MEM us", "off=", to_us(mem_off), "on=", to_us(mem_on));
	con_skip();

	stage = "psci";
	con_hex2("PSCI", "ver=", smc(PSCI_VERSION, 0, 0, 0), "smccc=", smc(SMCCC_VERSION, 0, 0, 0));
	con_skip();
	test_gic();

	stage = "done";
	reset(40);
}

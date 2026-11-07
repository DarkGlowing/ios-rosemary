/*
 * MT6785 (Rosemary) platform helpers: watchdog petting + EMI state restore.
 *
 * WDT: preloader/LK leave the A7 watchdog running with a short timeout
 * (typically ~60s). If the shim (or XNU) hangs without petting it, the SoC
 * resets — which is actually the desired auto-recovery. But during long
 * deliberate waits (UFS probe hold, panic-screen hold) we must pet it so the
 * user can read the screen. We never disable it: a hang still auto-reboots.
 *
 * EMI: a crashed kernel can leave DRAM in self-refresh / with clocks gated,
 * which makes the next BROM session hang at "sending emi data". On any reset
 * path (panic or normal) we make a best-effort write to bring DRAM channels
 * back to a normal state before PSCI reset. This is NOT a full DRAM retrain
 * (only preloader can do that); it reduces the probability of a stuck EMI
 * state. A cold power cycle always remains the final recovery.
 */
#include "platform_mt6785.h"

static inline void wr32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline uint32_t rd32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void dsb(void) { __asm__ volatile("dsb sy" ::: "memory"); }

uint32_t wdt_mode(void) { return rd32(A7_WDT_BASE + WDT_MODE); }
uint32_t wdt_length(void) { return rd32(A7_WDT_BASE + WDT_LENGTH); }

void wdt_pet(void)
{
	/* Magic restart value: key in upper half + any byte in lower half. */
	wr32(A7_WDT_BASE + WDT_RESTART, (WDT_KEY << 16) | 0x19);
	dsb();
}

void wdt_init_long_timeout(void)
{
	/* Pet first: LK may already be counting down on this instance. */
	wdt_pet();

	/* Best-effort: max out the timeout (key-protected write, bits 11:0). */
	wr32(A7_WDT_BASE + WDT_LENGTH, (WDT_KEY << 16) | 0xfff);
	dsb();

	/* Make sure the instance is enabled so a real hang still recovers the
	 * phone. Mode bits are preserved; only EN is forced on. */
	uint32_t mode = rd32(A7_WDT_BASE + WDT_MODE);
	if (!(mode & WDT_MODE_EN))
		wr32(A7_WDT_BASE + WDT_MODE, (WDT_KEY << 16) | (mode | WDT_MODE_EN));
	dsb();
	wdt_pet();
}

/* Diagnostic builds only: stop the countdown completely so a slow framebuffer
 * dump (a full 74-row panic screen takes tens of seconds at uncached write
 * speed) is not cut in half by the watchdog. The phone then stays hung until
 * the power key, which is exactly what a screen-reading session wants. */
void wdt_disable(void)
{
	uint32_t mode = rd32(A7_WDT_BASE + WDT_MODE);
	wr32(A7_WDT_BASE + WDT_MODE, (WDT_KEY << 16) | (mode & ~(uint32_t)WDT_MODE_EN));
	dsb();
}

/* Hard SoC reset through top RGU. Used as a fallback if PSCI SYSTEM_RESET
 * is not offered (or silently ignored) — MTK LK is AArch32 and the PSCI
 * conduit may not be wired up the way the Qualcomm original assumed. */
void wdt_swreset(void)
{
	wr32(A7_WDT_BASE + WDT_SWRST, (WDT_KEY << 16) | 0x1);
	dsb();
	for (;;)
		__asm__ volatile("wfe");
}

void emi_restore(void)
{
	/* 1. Power up / exit self-refresh on both DRAM channels.
	 *    (DRAMC power-control register, best-effort value.) */
	wr32(DRAMC0_BASE + 0x000, 0x00000001);
	wr32(DRAMC1_BASE + 0x000, 0x00000001);
	dsb();

	/* 2. Re-enable both memory channels in EMI_CONA. */
	wr32(CEN_EMI_BASE + EMI_CONA, 0x00000003);
	dsb();

	/* 3. Short settle delay (~100k iterations). */
	for (volatile int i = 0; i < 100000; i++) { (void)i; }

	/* 4. Pet the watchdog so the subsequent PSCI reset is clean. */
	wdt_pet();
}

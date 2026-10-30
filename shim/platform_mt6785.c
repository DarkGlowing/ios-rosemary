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
static inline void dsb(void) { __asm__ volatile("dsb sy" ::: "memory"); }

void wdt_pet(void)
{
	/* Magic restart value: key in upper half + any byte in lower half. */
	wr32(A7_WDT_BASE + WDT_RESTART, (WDT_KEY << 16) | 0x19);
	dsb();
}

void wdt_init_long_timeout(void)
{
	/* Best-effort: set a ~60s timeout (key-protected). If preloader uses
	 * a different WDT instance this write is harmless. */
	wr32(A7_WDT_BASE + WDT_LENGTH, (WDT_KEY << 16) | 0xfff);
	dsb();
	wdt_pet();
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

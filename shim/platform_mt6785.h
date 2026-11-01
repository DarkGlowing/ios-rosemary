#pragma once
#include <stdint.h>

/*
 * MediaTek MT6785 (rosemary) AP watchdog — FIXED base.
 *
 * The old value 0xf0400000 is NOT a physical MediaTek address. It is the kind
 * of address LK uses for its own MMIO mapping (LK is AArch32 and runs with the
 * MMU on); a 32-bit kernel/shim running with the MMU off must use the PHYSICAL
 * base instead. LK itself tells us the correct node: the string
 *   "[WDT] [WDT] Found /toprgu@10007000 at offset %d"
 * is present in the device's lk_a.bin, and the DTB has /toprgu@10007000.
 * Writing to 0xf0400000 from EL1/EL2 with the MMU off raises an external
 * abort, which is exactly the kind of fault that kills the shim before it can
 * draw anything (black screen + reboot after a few seconds).
 */
#define A7_WDT_BASE		0x10007000UL	/* top RGU: /toprgu@10007000 */

/* Register offsets (mtk_wdt / top_rgu). */
#define WDT_MODE		0x00	/* bit0 EN, bit1 IRQ, bit2 EXT, bit3 EXT_SEL */
#define WDT_LENGTH		0x04
#define WDT_RESTART		0x08
#define WDT_STATUS		0x0c
#define WDT_INTERVAL		0x10
#define WDT_SWRST		0x14
#define WDT_SWSYSRST		0x18

/* Every write to a protected register must carry this key in bits 31:16. */
#define WDT_KEY			0x1971UL
#define WDT_MODE_EN		(1U << 0)

#define CEN_EMI_BASE		0x10219000UL	/* emi@10219000 (CEN EMI) */
#define EMI_CONA		0x000	/* channel A control */
#define DRAMC0_BASE		0x10230000UL
#define DRAMC1_BASE		0x10240000UL

uint32_t wdt_mode(void);
uint32_t wdt_length(void);
void wdt_pet(void);
void wdt_init_long_timeout(void);
void wdt_swreset(void) __attribute__((noreturn));
void emi_restore(void);

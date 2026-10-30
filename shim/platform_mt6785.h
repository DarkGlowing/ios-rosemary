#pragma once
#include <stdint.h>

/* MediaTek MT6785 (Rosemary) platform helpers.
 * All addresses confirmed from device dumps:
 *   preloader boot1, lk_a, kernel dtb (see README-ROSSEMARY-FIXES.md).
 */

/* A7 subsystem watchdog — base confirmed (3 occurrences in preloader).
 * WDT_RESTART register: write key 0x1971<<16 | 0x19 to pet. */
#define A7_WDT_BASE		0xf0400000UL
#define WDT_MODE		0x00
#define WDT_LENGTH		0x04
#define WDT_RESTART		0x08
#define WDT_SWRST		0x10
#define WDT_KEY			0x1971UL

/* EMI / DRAM controller — confirmed in preloader + kernel dtb. */
#define CEN_EMI_BASE		0x10219000UL
#define DRAMC0_BASE		0x10230000UL
#define DRAMC1_BASE		0x10240000UL
#define EMI_CONA		0x000	/* channel enable (both channels = 0x3) */

/* Display (already brought up by LK; we only reuse its framebuffer).
 * DSI0 controller + RDMA1 (the path LK uses) — confirmed in lk_a + dtb. */
#define DSI0_BASE		0x14014000UL
#define RDMA1_BASE		0x1400c000UL
#define DISP_PWM0_BASE		0x1100e000UL	/* confirmed in kernel dtb */

void wdt_pet(void);
void wdt_init_long_timeout(void);
void emi_restore(void);

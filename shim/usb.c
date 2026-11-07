/*
 * ROSEMARY USB host bring-up probe (MT6785 "rosemary", Synopsys DWC3 + MTK IPPC).
 *
 * The USB code carried over from the sweet port (xnu/patches/0072..0081) is
 * Qualcomm SM7125 code - DWC3 at 0x0a600000, QSCRATCH at 0x0a6f8800, GCC power
 * domains. None of those blocks exist on MT6785, so every USB experiment on
 * rosemary before this file talked to nothing.
 *
 * What two hardware runs established (2026-11-04/05):
 *   run 1: DWC3 core (0x11200000..) read ALL ZEROS, while IPPC (0x11203e00)
 *          and U3PHY (0x11e40000) answered. IPPC decode:
 *            PW_CTRL0 = 0x10411021 -> bit0 SSUSB_IP_SW_RST  = 1 (IP in reset)
 *            PW_CTRL1 = 0x00000001 -> bit0 SSUSB_IP_HOST_PDN = 1 (host off)
 *            PW_STS1  = 0, PW_STS2 = 0
 *   run 2: after enabling INFRA2 gates (infra2_sta = 0x80000002 = bit1 + bit31)
 *          PW_STS1/STS2 STILL read 0 and GSNPSID stayed 0, and worse, PW_CTRL0
 *          started reading 0 instead of 0x10411021 - i.e. one of the writes in
 *          that run (candidate: the APMIXED 0x20 "26 MHz gate" write) reset or
 *          unclocked the IPPC block itself.
 *
 * So the layer above the gates is missing: the TOPCKGEN muxes
 *   CK_CFG_8 bits [9:8]  = usb_top_sel        (parents: clk26m,
 *                                               univpll_d5_d4, univpll_d3_d4,
 *                                               univpll_d5_d2)
 *   CK_CFG_8 bits [17:16] = ssusb_top_xhci_sel (same parents)
 * and their PLL (univ2pll: enable bit 0 of APMIXED+0x24C).
 *
 * Stage 3 therefore bisects, one write at a time, with a read-back after each:
 * it prints every register, enables the INFRA gates, then switches both muxes
 * to clk26m (no PLL needed) and, separately, to univpll_d5_d4. Whichever of the
 * two makes PW_STS1 come alive and GSNPSID nonzero tells us exactly what to
 * program next. Nothing here writes APMIXED: a PLL is not touched blind.
 *
 * Addresses from this device's live FDT:
 *   0x11200000 x10000  xHCI + DWC3 global/device   (/usb3@11200000)
 *   0x11203e00 x100    IPPC power/port control     (reg "ippc")
 *   0x11e40000 x10000  MTK U3PHY                   (/usb3_phy)
 *   0x1000c000 x1000   APMIXED                     (/apmixed@1000c000)
 *   0x10001000         INFRA_AO clock gates        (/infracfg_ao@10001000)
 *   0x10000000         TOPCKGEN muxes              (/topckgen@10000000)
 *
 * The shim runs with the MMU off: raw physical accesses. Print the marker for a
 * step BEFORE executing it, so a frozen screen names the last good step.
 */
#include <stdint.h>
#include "console.h"
#include "usb.h"

/* ---- addresses ---------------------------------------------------------- */
#define DWC3_BASE	0x11200000UL
#define IPPC_BASE	0x11203e00UL
#define U3PHY_BASE	0x11e40000UL
#define APMIXED_BASE	0x1000c000UL
#define INFRA_AO_BASE	0x10001000UL
#define TOPCKGEN_BASE	0x10000000UL

/* xHCI capability registers */
#define X_CAPLENGTH	0x00
#define X_HCSPARAMS1	0x04
#define X_HCCPARAMS1	0x10

/* DWC3 global / device registers */
#define D_GCTL		0xc110
#define D_GSNPSID	0xc120
#define D_GUSB2PHYCFG0	0xc200
#define D_DCTL		0xc704
#define D_DSTS		0xc70c

#define GCTL_PRTCAP_MASK	(3u << 12)
#define GCTL_PRTCAP_HOST	(1u << 12)
#define DCTL_CSFTRST		(1u << 0)

/* IPPC (mtu3_hw_regs.h, SSUSB_SIFSLV_IPPC) */
#define IPPC_PW_CTRL0	0x00	/* bit0: SSUSB_IP_SW_RST */
#define IPPC_PW_CTRL1	0x04	/* bit0: SSUSB_IP_HOST_PDN */
#define IPPC_PW_CTRL2	0x08	/* bit0: SSUSB_IP_DEV_PDN */
#define IPPC_PW_CTRL3	0x0c
#define IPPC_PW_STS1	0x10
#define IPPC_PW_STS2	0x14
#define IPPC_U2_CTRL0	0x50

#define SSUSB_IP_SW_RST		(1u << 0)
#define SSUSB_IP_HOST_PDN	(1u << 0)
#define SSUSB_XHCI_RST_B_STS	(1u << 11)
#define SSUSB_SYS125_RST_B_STS	(1u << 10)
#define SSUSB_REF_RST_B_STS	(1u << 8)
#define SSUSB_SYSPLL_STABLE	(1u << 0)
#define SSUSB_U2_MAC_SYS_RST_B_STS (1u << 0)
#define SSUSB_U2_PORT_HOST_SEL	(1u << 2)
#define SSUSB_U2_PORT_PDN	(1u << 1)
#define SSUSB_U2_PORT_DIS	(1u << 0)
#define SSUSB_U2_PORT_VBUSVALID	(1u << 9)
#define SSUSB_U2_PORT_FORCE_IDDIG (1u << 11)

/* INFRA_AO clock gates (clk-mt6785.c) */
#define INFRA0_SET	0x80
#define INFRA0_CLR	0x84
#define INFRA0_STA	0x90
#define INFRA2_SET	0xa4
#define INFRA2_CLR	0xa8
#define INFRA2_STA	0xac
#define GATE_INFRA0_ICUSB	(1u << 8)
#define GATE_INFRA2_USB		(1u << 1)
#define GATE_INFRA2_SSUSB_XHCI	(1u << 31)

/* TOPCKGEN: CK_CFG_8 (muxes), CK_CFG_8_SET/CLR, CK_CFG_UPDATE1 */
#define TOP_CK_CFG_8		0xa0
#define TOP_CK_CFG_8_SET	0xa4
#define TOP_CK_CFG_8_CLR	0xa8
#define TOP_CK_CFG_UPDATE1	0x008
#define TOP_USB_TOP_SHIFT	8	/* usb_top_sel, 2 bits */
#define TOP_XHCI_SHIFT		16	/* ssusb_top_xhci_sel, 2 bits */
#define TOP_UPD_USB		(1u << 1)
#define TOP_UPD_XHCI		(1u << 2)
#define TOP_SEL_26M		0	/* clk26m */
#define TOP_SEL_UNIVPLL		1	/* univpll_d5_d4 */

/* APMIXED: 26 MHz gate (active low) and univ2pll registers (read-only here) */
#define APMIXED_GATE		0x20
#define APMIXED_UNIVPLL_CON0	0x240
#define APMIXED_UNIVPLL_CON1	0x244
#define APMIXED_UNIVPLL_PWR	0x24c

/*
 * MTK U3PHY registers, MT_PHY_V2 layout (phy-mtk-mt6785.c: the phycfg for this
 * SoC is MT_PHY_V2, port0 is PHY_TYPE_USB3 at port_base = phy_base + 0):
 *   sif_misc = 0x11e40000, sif_u2phy_com = +0x300, sif_u3phya = +0xb00
 * Offsets and bits from phy-mtk-ssusb-reg.h / phy-mtk-mt6785.c.
 */
#define PHY_U2_COM      (U3PHY_BASE + 0x300)
#define PHY_U3PHYA      (U3PHY_BASE + 0xb00)
#define PHY_USBPHYACR6  (PHY_U2_COM + 0x18)
#define PHY_U2PHYACR4   (PHY_U2_COM + 0x20)
#define PHY_U2PHYDTM0   (PHY_U2_COM + 0x68)
#define PHY_U2PHYDTM1   (PHY_U2_COM + 0x6c)
#define PHY_PHYA_REG1   (PHY_U3PHYA + 0x0004)

#define RG_SSUSB_VUSB10_ON      (1u << 29)  /* PHYA_REG1 */
#define RG_UART_MODE_MASK       (3u << 30)  /* U2PHYDTM0 */
#define FORCE_UART_EN           (1u << 26)  /* U2PHYDTM0 */
#define FORCE_SUSPENDM          (1u << 18)  /* U2PHYDTM0 */
#define RG_UART_EN              (1u << 16)  /* U2PHYDTM1 */
#define RG_USB20_GPIO_CTL       (1u << 9)   /* U2PHYACR4 */
#define USB20_GPIO_MODE         (1u << 8)   /* U2PHYACR4 */
#define RG_USB20_DP_100K_MODE   (1u << 18)  /* U2PHYACR4 */
#define RG_USB20_DM_100K_EN     (1u << 17)  /* U2PHYACR4 */
#define USB20_DP_100K_EN        (1u << 16)  /* U2PHYACR4 */
#define RG_USB20_BC11_SW_EN     (1u << 23)  /* USBPHYACR6 */
#define RG_USB20_OTG_VBUSCMP_EN (1u << 20)  /* USBPHYACR6 */

/* xHCI operational registers */
#define XO_USBCMD	0x00
#define USBCMD_RS	(1u << 0)
#define USBCMD_HCRST	(1u << 1)
#define XO_USBSTS	0x04
/* Standard xHCI operational offsets; stage 60 readback confirms masks. */
#define XO_CRCR 0x18
#define XO_DCBAAP 0x30
#define XO_CONFIG 0x38
#define XO_PORTSC	0x400
#define PORTSC_CCS	(1u << 0)
#define PORTSC_PED	(1u << 1)
#define PORTSC_PP	(1u << 9)
#define PORTSC_PLS_MASK	(0xfu << 5)
#define PORTSC_SPEED_MASK (0xfu << 10)
#define PORTSC_CSC	(1u << 17)
#define PORTSC_PR	(1u << 4)	/* port reset */
#define PORTSC_PRC	(1u << 21)	/* port reset change, RW1C */

/* ---- helpers ------------------------------------------------------------ */
#define rd32(a)		(*(volatile uint32_t *)(uintptr_t)(a))
#define rd8(a)		(*(volatile uint8_t *)(uintptr_t)(a))
static inline void wr32(uintptr_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
	__asm__ volatile("dsb sy" ::: "memory");
}

static uint64_t cnt(void)
{
	uint64_t v;
	__asm__ volatile("mrs %0, cntpct_el0" : "=r"(v));
	return v;
}

/* CNTFRQ on MT6785 is 13 MHz (same assumption as the shim's timing code). */
static void delay_us(uint32_t us)
{
	uint64_t t0 = cnt();
	uint64_t want = (uint64_t)us * 13u;
	while (cnt() - t0 < want)
		;
}

static unsigned caplength(void)
{
	return rd8(DWC3_BASE + X_CAPLENGTH) & 0xff;
}

/* ---- stage 0: report ---------------------------------------------------- */
static void usb_report(const char *tag)
{
	con_str(tag, "rd IPPC @11203e00 (power/port control)");
	con_hex2(tag, "pw_ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "pw_ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1));
	con_hex2(tag, "pw_sts1=", rd32(IPPC_BASE + IPPC_PW_STS1),
		 "pw_sts2=", rd32(IPPC_BASE + IPPC_PW_STS2));
	con_str(tag, "rd DWC3 @11200000");
	con_hex2(tag, "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID),
		 "caplen=", caplength());
}

/* ---- TOPCKGEN mux selection -------------------------------------------- */
static void top_usb_mux(uint32_t sel)
{
	/* both fields carry the same parent index: 0 = clk26m, 1 = univpll_d5_d4 */
	wr32(TOPCKGEN_BASE + TOP_CK_CFG_8_CLR,
	     (3u << TOP_USB_TOP_SHIFT) | (3u << TOP_XHCI_SHIFT));
	if (sel & 1u)
		wr32(TOPCKGEN_BASE + TOP_CK_CFG_8_SET,
		     (1u << TOP_USB_TOP_SHIFT) | (1u << TOP_XHCI_SHIFT));
	if (sel & 2u)
		wr32(TOPCKGEN_BASE + TOP_CK_CFG_8_SET,
		     (2u << TOP_USB_TOP_SHIFT) | (2u << TOP_XHCI_SHIFT));
	wr32(TOPCKGEN_BASE + TOP_CK_CFG_UPDATE1, TOP_UPD_USB | TOP_UPD_XHCI);
	__asm__ volatile("dsb sy" ::: "memory");
}

/* ---- stage 2: IPPC power-on (no PLL writes) ----------------------------- */
static void usb_wait_clocks(const char *tag)
{
	const uint32_t want1 = SSUSB_SYSPLL_STABLE | SSUSB_REF_RST_B_STS |
			       SSUSB_SYS125_RST_B_STS | SSUSB_XHCI_RST_B_STS;
	uint32_t sts1 = 0, sts2 = 0;
	int i;

	for (i = 0; i < 200; i++) {
		sts1 = rd32(IPPC_BASE + IPPC_PW_STS1);
		sts2 = rd32(IPPC_BASE + IPPC_PW_STS2);
		if ((sts1 & want1) == want1 && (sts2 & SSUSB_U2_MAC_SYS_RST_B_STS))
			break;
		delay_us(200);
	}
	con_hex2(tag, "sts1=", sts1, "sts2=", sts2);
	con_hex2(tag, "want1=", want1, "ok=", (sts1 & want1) == want1);
}

static void usb_ip_power_on(void)
{
	uint32_t v;

	con_str("USBP2", "wr IPPC: sw_rst=0, host_pdn=0, u2 host_sel=1");
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) | SSUSB_U2_PORT_HOST_SEL);
	delay_us(20000);
	con_hex2("USBP2", "rd back ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1));
	usb_wait_clocks("USBP2");
}

/* ---- host port: power it and look for a device ------------------------- */
static int usb_check_port(const char *tag)
{
	uint32_t cl = caplength();
	uintptr_t op, portsc;
	uint32_t gctl, ps;
	int i;

	if (cl < 0x20 || cl > 0x100) {
		con_hex2(tag, "bad caplen=", cl, "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));
		con_str(tag, "xHCI core not responding");
		return -1;
	}
	op = DWC3_BASE + cl;
	portsc = op + XO_PORTSC;

	gctl = rd32(DWC3_BASE + D_GCTL);
	con_hex2(tag, "prtcap=", (gctl & GCTL_PRTCAP_MASK) >> 12,
		 "usbcmd=", rd32(op + XO_USBCMD));
	if ((gctl & GCTL_PRTCAP_MASK) != GCTL_PRTCAP_HOST) {
		con_str(tag, "GCTL not host: forcing via CSFTRST");
		wr32(DWC3_BASE + D_GCTL, (gctl & ~GCTL_PRTCAP_MASK) | GCTL_PRTCAP_HOST);
		wr32(DWC3_BASE + D_DCTL, rd32(DWC3_BASE + D_DCTL) | DCTL_CSFTRST);
		delay_us(20000);
		wr32(DWC3_BASE + D_DCTL, rd32(DWC3_BASE + D_DCTL) & ~DCTL_CSFTRST);
		delay_us(20000);
		con_hex2(tag, "prtcap=",
			 (rd32(DWC3_BASE + D_GCTL) & GCTL_PRTCAP_MASK) >> 12,
			 "dsts=", rd32(DWC3_BASE + D_DSTS));
	}

	con_str(tag, "wr PORTSC.PP=1 (+clear CSC)");
	wr32(portsc, PORTSC_PP | PORTSC_CSC);
	delay_us(100000);
	for (i = 0; i < 150; i++) {
		ps = rd32(portsc);
		if (ps & PORTSC_CCS) {
			con_hex2(tag, "portsc=", ps, "speed=",
				 (ps & PORTSC_SPEED_MASK) >> 10);
			con_str(tag, "DEVICE CONNECTED - host mode works");
			return 0;
		}
		delay_us(10000);
	}
	con_hex2(tag, "portsc=", rd32(portsc), "pls=",
		 (rd32(portsc) & PORTSC_PLS_MASK) >> 5);
	con_str(tag, "no connect: VBUS 5V or PHY power missing");
	return -1;
}

/* ---- stage 3: bisect, one write at a time ------------------------------ */
static void usb_bisect(void)
{
	uint32_t ctl0_before = rd32(IPPC_BASE + IPPC_PW_CTRL0);

	con_str("USB3", "snapshot before any write of this stage");
	con_hex2("USB3", "pw_ctl0=", ctl0_before,
		 "pw_ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1));
	con_hex2("USB3", "pw_sts1=", rd32(IPPC_BASE + IPPC_PW_STS1),
		 "pw_sts2=", rd32(IPPC_BASE + IPPC_PW_STS2));
	con_hex2("USB3", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "infra0_sta=", rd32(INFRA_AO_BASE + INFRA0_STA));
	con_hex2("USB3", "top_cfg8=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_8),
		 "top_upd1=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_UPDATE1));
	con_hex2("USB3", "apmixed26m=", rd32(APMIXED_BASE + APMIXED_GATE),
		 "univ_pwr=", rd32(APMIXED_BASE + APMIXED_UNIVPLL_PWR));

	/* 1) INFRA2: "infra_usb" + "infra_ssusb_xhci" */
	con_str("USB3", "wr INFRA2_SET bits 1,31");
	wr32(INFRA_AO_BASE + INFRA2_SET, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	delay_us(1000);
	con_hex2("USB3", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "pw_ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0));

	/* 2) INFRA0: "infra_icusb" */
	con_str("USB3", "wr INFRA0_SET bit 8");
	wr32(INFRA_AO_BASE + INFRA0_SET, GATE_INFRA0_ICUSB);
	delay_us(1000);
	con_hex2("USB3", "infra0_sta=", rd32(INFRA_AO_BASE + INFRA0_STA),
		 "pw_ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0));

	/* 3) release IP reset + host power on, no PLL involved yet */
	usb_ip_power_on();

	/* 4) mux both USB clocks to the 26 MHz crystal: no PLL needed */
	con_str("USB3", "wr TOP mux usb/xhci = clk26m");
	top_usb_mux(TOP_SEL_26M);
	delay_us(2000);
	con_hex2("USB3", "top_cfg8=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_8),
		 "pw_ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0));
	usb_wait_clocks("USB3");
	con_hex2("USB3", "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID),
		 "caplen=", caplength());

	/* 5) mux to univpll_d5_d4: works only if univ2pll is powered */
	con_str("USB3", "wr TOP mux usb/xhci = univpll_d5_d4");
	top_usb_mux(TOP_SEL_UNIVPLL);
	delay_us(2000);
	con_hex2("USB3", "top_cfg8=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_8),
		 "univ_pwr=", rd32(APMIXED_BASE + APMIXED_UNIVPLL_PWR));
	usb_wait_clocks("USB3");
	con_hex2("USB3", "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID),
		 "caplen=", caplength());

	usb_check_port("USB3");
}

/* ---- stage 4: PHY power-up + IPPC host sequence ------------------------ */
static void usb_phy_on(void)
{
	uint32_t v;

	con_str("USB4", "phy: snapshot (MT_PHY_V2 banks)");
	con_hex2("USB4", "phya1=", rd32(PHY_PHYA_REG1),
		 "u2dtm0=", rd32(PHY_U2PHYDTM0));
	con_hex2("USB4", "u2dtm1=", rd32(PHY_U2PHYDTM1),
		 "misc=", rd32(U3PHY_BASE));

	con_str("USB4", "phy: VUSB10_ON=1 (U3PHYA_REG1 bit29)");
	wr32(PHY_PHYA_REG1, rd32(PHY_PHYA_REG1) | RG_SSUSB_VUSB10_ON);
	delay_us(250);
	con_hex2("USB4", "phya1=", rd32(PHY_PHYA_REG1),
		 "u2dtm0=", rd32(PHY_U2PHYDTM0));

	con_str("USB4", "phy: uart-mode off, 100k mode, bc1.1 off");
	v = rd32(PHY_U2PHYDTM0);
	wr32(PHY_U2PHYDTM0, v & ~(RG_UART_MODE_MASK | FORCE_UART_EN | FORCE_SUSPENDM));
	v = rd32(PHY_U2PHYDTM1);
	wr32(PHY_U2PHYDTM1, v & ~RG_UART_EN);
	v = rd32(PHY_U2PHYACR4);
	wr32(PHY_U2PHYACR4,
	     (v & ~(RG_USB20_GPIO_CTL | USB20_GPIO_MODE |
		    USB20_DP_100K_EN | RG_USB20_DM_100K_EN)) |
	     RG_USB20_DP_100K_MODE);
	v = rd32(PHY_USBPHYACR6);
	wr32(PHY_USBPHYACR6, v & ~(RG_USB20_BC11_SW_EN | RG_USB20_OTG_VBUSCMP_EN));
	delay_us(800);
	con_hex2("USB4", "u2dtm0=", rd32(PHY_U2PHYDTM0),
		 "u2acr4=", rd32(PHY_U2PHYACR4));
}

static void usb_stage4(void)
{
	uint32_t v;

	con_str("USB4", "1) INFRA gates: usb, ssusb_xhci, icusb");
	wr32(INFRA_AO_BASE + INFRA2_SET, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_SET, GATE_INFRA0_ICUSB);
	con_hex2("USB4", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "infra0_sta=", rd32(INFRA_AO_BASE + INFRA0_STA));

	usb_phy_on();

	con_str("USB4", "2) IPPC: pulse sw_rst, host_pdn=0, u2 host_sel");
	wr32(IPPC_BASE + IPPC_PW_CTRL0, SSUSB_IP_SW_RST);
	delay_us(1000);
	wr32(IPPC_BASE + IPPC_PW_CTRL0, 0);
	delay_us(1000);
	wr32(IPPC_BASE + IPPC_PW_CTRL1, 0);
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) | SSUSB_U2_PORT_HOST_SEL);
	delay_us(20000);
	con_hex2("USB4", "ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1));
	usb_wait_clocks("USB4");

	con_str("USB4", "3) mux = parent 3 (univpll_d5_d2, what LK left)");
	top_usb_mux(3);
	delay_us(2000);
	con_hex2("USB4", "top_cfg8=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_8),
		 "univ_pwr=", rd32(APMIXED_BASE + APMIXED_UNIVPLL_PWR));
	usb_wait_clocks("USB4");
	con_hex2("USB4", "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID),
		 "caplen=", caplength());

	con_str("USB4", "4) port power + connect check");
	usb_check_port("USB4");
}

/* ---- stage 5: is the SSUSB block readable at all? ---------------------- */
static void usb_ipid(const char *tag)
{
	con_hex2(tag, "hw_id=", rd32(IPPC_BASE + 0xa0),
		 "hw_sub_id=", rd32(IPPC_BASE + 0xa4));
	con_hex2(tag, "xhci_cap=", rd32(IPPC_BASE + 0x24),
		 "dev_cap=", rd32(IPPC_BASE + 0x28));
	con_hex2(tag, "ref_ck_ctrl=", rd32(IPPC_BASE + 0x8c),
		 "dev_rst_ctrl=", rd32(IPPC_BASE + 0x98));
	con_hex2(tag, "ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0));
	con_hex2(tag, "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1),
		 "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));
}

static void usb_stage5(void)
{
	uint32_t v;

	con_str("USB5", "A) gates as LK left them (no writes)");
	usb_ipid("USB5");
	/* INFRA_AO reset banks (MTK layout: set/clr/sta triples). A USB/SSUSB
	 * reset still asserted by LK would show up here. Read-only. */
	con_hex2("USB5", "rst0_set=", rd32(INFRA_AO_BASE + 0x120),
		 "rst0_clr=", rd32(INFRA_AO_BASE + 0x124));
	con_hex2("USB5", "rst0_sta=", rd32(INFRA_AO_BASE + 0x128),
		 "rst1_set=", rd32(INFRA_AO_BASE + 0x130));
	con_hex2("USB5", "rst1_sta=", rd32(INFRA_AO_BASE + 0x138),
		 "rst2_sta=", rd32(INFRA_AO_BASE + 0x148));

	con_str("USB5", "B) CLR infra2 bits 1,31 (gate off, if polarity normal)");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	delay_us(2000);
	con_hex2("USB5", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA), "x=", 0);
	usb_ipid("USB5");

	con_str("USB5", "C) SET infra2 bits 1,31 (gate on, if polarity normal)");
	wr32(INFRA_AO_BASE + INFRA2_SET, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	delay_us(2000);
	con_hex2("USB5", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA), "x=", 0);
	usb_ipid("USB5");

	con_str("USB5", "D) IPPC write/readback test (sw_rst bit)");
	wr32(IPPC_BASE + IPPC_PW_CTRL0, SSUSB_IP_SW_RST);
	delay_us(500);
	con_hex2("USB5", "wrote=", SSUSB_IP_SW_RST,
		 "read=", rd32(IPPC_BASE + IPPC_PW_CTRL0));
	wr32(IPPC_BASE + IPPC_PW_CTRL0, 0);
	delay_us(500);
	con_hex2("USB5", "wrote=", 0, "read=", rd32(IPPC_BASE + IPPC_PW_CTRL0));

	con_str("USB5", "E) DWC3 write/readback test (GCTL.PrtCapDir)");
	v = rd32(DWC3_BASE + D_GCTL);
	wr32(DWC3_BASE + D_GCTL, (v & ~GCTL_PRTCAP_MASK) | (2u << 12));
	delay_us(1000);
	con_hex2("USB5", "wrote=", (2u << 12),
		 "read=", rd32(DWC3_BASE + D_GCTL));
	wr32(DWC3_BASE + D_GCTL, v);
}

/* ---- stage 6: correct polarity + release IP reset ---------------------- */
static void usb_stage6(void)
{
	uint32_t v;

	con_str("USB6", "A) snapshot, no writes");
	usb_ipid("USB6");

	con_str("USB6", "B) CLR infra2 1,31 + infra0 8 (bit=1 means clock OFF)");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	delay_us(2000);
	con_hex2("USB6", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "hw_id=", rd32(IPPC_BASE + 0xa0));

	con_str("USB6", "C) ctl0: clear sw_rst bit0 (release IP reset)");
	v = rd32(IPPC_BASE + IPPC_PW_CTRL0);
	wr32(IPPC_BASE + IPPC_PW_CTRL0, v & ~SSUSB_IP_SW_RST);
	delay_us(1000);
	con_hex2("USB6", "ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1));

	con_str("USB6", "D) ctl1: host power on (clear bit0)");
	v = rd32(IPPC_BASE + IPPC_PW_CTRL1);
	wr32(IPPC_BASE + IPPC_PW_CTRL1, v & ~SSUSB_IP_HOST_PDN);
	delay_us(1000);
	con_hex2("USB6", "ctl1=", rd32(IPPC_BASE + IPPC_PW_CTRL1),
		 "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0));

	con_str("USB6", "E) U2 port0: DIS=0, PDN=0, HOST_SEL=1");
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) | SSUSB_U2_PORT_HOST_SEL);
	delay_us(20000);
	con_hex2("USB6", "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0),
		 "dev_rst_ctrl=", rd32(IPPC_BASE + 0x98));

	con_str("USB6", "F) wait for IP clocks");
	usb_wait_clocks("USB6");
	con_hex2("USB6", "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID),
		 "caplen=", caplength());
	con_hex2("USB6", "gctl=", rd32(DWC3_BASE + D_GCTL),
		 "dsts=", rd32(DWC3_BASE + D_DSTS));

	con_str("USB6", "G) port power + connect check");
	usb_check_port("USB6");
}

/* ---- stage 7: find global regs, start the controller ------------------- */
static void dump32(const char *tag, uintptr_t base)
{
	con_hex2(tag, "+00=", rd32(base + 0x00), "+04=", rd32(base + 0x04));
	con_hex2(tag, "+08=", rd32(base + 0x08), "+0c=", rd32(base + 0x0c));
	con_hex2(tag, "+10=", rd32(base + 0x10), "+14=", rd32(base + 0x14));
	con_hex2(tag, "+18=", rd32(base + 0x18), "+1c=", rd32(base + 0x1c));
}

static void usb_stage7(void)
{
	uint32_t v, cl;
	uintptr_t op;
	int i;

	con_str("USB7", "0) repeat the working power sequence");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	v = rd32(IPPC_BASE + IPPC_PW_CTRL0);
	wr32(IPPC_BASE + IPPC_PW_CTRL0, v & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1, rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) | SSUSB_U2_PORT_HOST_SEL);
	delay_us(20000);
	con_hex2("USB7", "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1),
		 "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0));

	cl = caplength();
	con_hex2("USB7", "1) xHCI caps @11200000 caplen=", cl,
		 "hbcs=", rd32(DWC3_BASE + 0x04));
	/* xHCI capability words */
	con_hex2("USB7", "+08=", rd32(DWC3_BASE + 0x08), "+0c=", rd32(DWC3_BASE + 0x0c));
	con_hex2("USB7", "+10=", rd32(DWC3_BASE + 0x10), "+14=", rd32(DWC3_BASE + 0x14));
	con_hex2("USB7", "+18=", rd32(DWC3_BASE + 0x18), "+1c=", rd32(DWC3_BASE + 0x1c));

	con_str("USB7", "2) DWC3 global @1120c100 (expect 5533xxxx at +20)");
	dump32("USB7g", DWC3_BASE + 0xc100);
	con_str("USB7", "3) MTK MAC glue @11201000");
	dump32("USB7m", DWC3_BASE + 0x1000);

	op = DWC3_BASE + cl;
	con_hex2("USB7", "4) op base caplen=", cl,
		 "usbcmd=", rd32(op + XO_USBCMD));
	con_hex2("USB7", "usbsts=", rd32(op + XO_USBSTS),
		 "portsc=", rd32(op + XO_PORTSC));

	con_str("USB7", "5) USBCMD.RS=1 (start xHCI)");
	wr32(op + XO_USBCMD, 1u);
	delay_us(50000);
	con_hex2("USB7", "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));

	con_str("USB7", "6) PORTSC samples (PP, PLS, CCS)");
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	for (i = 0; i < 3; i++) {
		delay_us(50000);
		v = rd32(op + XO_PORTSC);
		con_hex2("USB7", "portsc=", v, "pls=", (v & PORTSC_PLS_MASK) >> 5);
	}
	con_hex2("USB7", "ccs=", rd32(op + XO_PORTSC) & PORTSC_CCS,
		 "gctl_fallback=", rd32(DWC3_BASE + D_GCTL));
}

/* ---- stage 8: xHCI reset/start + PHY session bits --------------------- */
static uint32_t port_sample(const char *tag)
{
	uintptr_t op = DWC3_BASE + caplength();
	uint32_t ps = rd32(op + XO_PORTSC);

	con_hex2(tag, "portsc=", ps, "pls=", (ps >> 5) & 0xfu);
	return ps;
}

static void usb_stage8(void)
{
	uint32_t v, t;
	uintptr_t op;
	int i;

	con_str("USB8", "0) power sequence");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) | SSUSB_U2_PORT_HOST_SEL);
	delay_us(20000);
	con_hex2("USB8", "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1),
		 "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0));

	op = DWC3_BASE + caplength();
	con_str("USB8", "1) HCRST (USBCMD bit1)");
	wr32(op + XO_USBCMD, 2u);
	for (i = 0; i < 100; i++) {
		if (!(rd32(op + XO_USBCMD) & 2u))
			break;
		delay_us(1000);
	}
	con_hex2("USB8", "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));

	con_str("USB8", "2) CNR wait + USBCMD.RS=1");
	for (i = 0; i < 100; i++) {
		t = rd32(op + XO_USBSTS);
		if (!(t & (1u << 11)))  /* CNR */
			break;
		delay_us(1000);
	}
	wr32(op + XO_USBCMD, 1u);
	delay_us(50000);
	con_hex2("USB8", "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));

	con_str("USB8", "3) A: PP=1, host_sel only");
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(50000);
	port_sample("USB8");

	con_str("USB8", "4) B: + FORCE_IDDIG (force A-device/host)");
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     rd32(IPPC_BASE + IPPC_U2_CTRL0) | SSUSB_U2_PORT_FORCE_IDDIG);
	delay_us(50000);
	con_hex2("USB8", "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0), "x=", 0);
	port_sample("USB8");

	con_str("USB8", "5) C: + VBUSVALID (tell PHY session is valid)");
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     rd32(IPPC_BASE + IPPC_U2_CTRL0) | SSUSB_U2_PORT_VBUSVALID);
	delay_us(50000);
	con_hex2("USB8", "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0), "x=", 0);
	v = port_sample("USB8");

	con_str("USB8", "6) 5 more samples over 500 ms");
	for (i = 0; i < 5; i++) {
		delay_us(100000);
		port_sample("USB8");
	}
	con_hex2("USB8", "ccs=", v & PORTSC_CCS,
		 "ped=", v & PORTSC_PED);
}

/* ---- stage 9: identify the real operational base -------------------------------------- */
static void usb_op_probe(const char *tag, uint32_t cl)
{
	uintptr_t op = DWC3_BASE + cl;
	uint32_t before, after;

	before = rd32(op + XO_PORTSC);
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(20000);
	after = rd32(op + XO_PORTSC);
	con_hex2(tag, "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));
	con_hex2(tag, "portsc_before=", before, "portsc_after=", after);
	con_hex2(tag, "portsc2=", rd32(op + XO_PORTSC + 0x10),
		 "pp_sticks=", after & PORTSC_PP);
}

static void usb_stage9(void)
{
	uint32_t v, hcs1;

	con_str("USB9", "0) power sequence");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	v = rd32(IPPC_BASE + IPPC_U2_CTRL0);
	wr32(IPPC_BASE + IPPC_U2_CTRL0,
	     (v & ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS)) |
	     SSUSB_U2_PORT_HOST_SEL | SSUSB_U2_PORT_VBUSVALID |
	     SSUSB_U2_PORT_FORCE_IDDIG);
	delay_us(20000);

	hcs1 = rd32(DWC3_BASE + 0x04);
	con_hex2("USB9", "hcs1=", hcs1, "maxports=", (hcs1 >> 24) & 0xff);
	con_hex2("USB9", "otg_sts=", rd32(IPPC_BASE + 0x18),
		 "ref_ck_ctrl=", rd32(IPPC_BASE + 0x8c));
	con_hex2("USB9", "xecp140=", rd32(DWC3_BASE + 0x140),
		 "xecp144=", rd32(DWC3_BASE + 0x144));
	con_hex2("USB9", "u2_ctrl0=", rd32(IPPC_BASE + IPPC_U2_CTRL0),
		 "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1));

	con_str("USB9", "1) HCRST + RS at op=caplen(0x20)");
	wr32(DWC3_BASE + 0x20 + XO_USBCMD, 2u);
	delay_us(20000);
	wr32(DWC3_BASE + 0x20 + XO_USBCMD, 1u);
	delay_us(20000);
	usb_op_probe("USB9a", 0x20);

	con_str("USB9", "2) HCRST + RS at op=0x40 (DWC3 default)");
	wr32(DWC3_BASE + 0x40 + XO_USBCMD, 2u);
	delay_us(20000);
	wr32(DWC3_BASE + 0x40 + XO_USBCMD, 1u);
	delay_us(20000);
	usb_op_probe("USB9b", 0x40);

	con_str("USB9", "3) ports at op20 and op40, port0/1");
	con_hex2("USB9", "p20+400=", rd32(DWC3_BASE + 0x20 + 0x400),
		 "p20+410=", rd32(DWC3_BASE + 0x20 + 0x410));
	con_hex2("USB9", "p40+400=", rd32(DWC3_BASE + 0x40 + 0x400),
		 "p40+410=", rd32(DWC3_BASE + 0x40 + 0x410));
}

/* ---- stage 10: bisect U2_CTRL0 bits, watch PLS ------------------------- */
static void u2_try(const char *tag, uint32_t val)
{
	uintptr_t op = DWC3_BASE + caplength();
	uint32_t ps;

	wr32(IPPC_BASE + IPPC_U2_CTRL0, val);
	delay_us(60000);
	ps = rd32(op + XO_PORTSC);
	con_hex2(tag, "u2=", val, "portsc=", ps);
	con_hex2(tag, "pls=", (ps >> 5) & 0xfu,
		 "otg_sts=", rd32(IPPC_BASE + 0x18));
}

static void usb_stage10(void)
{
	uint32_t v;
	uintptr_t op;

	con_str("USBA", "0) power sequence");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	con_hex2("USBA", "u3_ctrl0=", rd32(IPPC_BASE + 0x30),
		 "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1));

	op = DWC3_BASE + caplength();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 4u);
	delay_us(20000);
	wr32(op + XO_USBCMD, 2u);
	delay_us(20000);
	wr32(op + XO_USBCMD, 1u);
	delay_us(20000);
	con_hex2("USBA", "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(30000);
	con_hex2("USBA", "after PP portsc=", rd32(op + XO_PORTSC),
		 "pls=", (rd32(op + XO_PORTSC) >> 5) & 0xfu);

	con_str("USBA", "1) u2_ctrl0 bisect");
	u2_try("USBA", 0x00000004u);		/* host_sel only */
	u2_try("USBA", 0x0000000cu);		/* + bit3 */
	u2_try("USBA", 0x00000044u);		/* + bit6 */
	u2_try("USBA", 0x0000004cu);		/* + bit3|bit6 (LK) */
	u2_try("USBA", 0x0000084cu);		/* + force_iddig */
	u2_try("USBA", 0x00000a4cu);		/* + vbusvalid */
	u2_try("USBA", 0x00000004u);		/* back to plain host */

	/* PHY: try to force host-side UTMI signalling. UTMI_MUXSEL(bit15)
	 * switches the PHY from controller-driven to force registers. */
	con_str("USBA", "2) phy: UTMI_MUXSEL + no suspend + drvvbus");
	v = rd32(PHY_U2PHYDTM0);
	con_hex2("USBA", "dtm0_before=", v, "x=", 0);
	v &= ~FORCE_SUSPENDM;
	v |= (1u << 15);		/* UTMI_MUXSEL */
	v |= (1u << 24);		/* FORCE_DRVVBUS */
	wr32(PHY_U2PHYDTM0, v);
	delay_us(20000);
	con_hex2("USBA", "dtm0_after=", rd32(PHY_U2PHYDTM0), "x=", 0);
	u2_try("USBA", 0x00000004u);
	u2_try("USBA", 0x00000a4cu);
}

/* ---- stage 11: vendor-faithful PHY recover + host ---------------------- */
static void usb_stage11(void)
{
	uint32_t v, ps;
	uintptr_t op;

	con_str("USBB", "0) power sequence");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);

	con_str("USBB", "1) phy: VUSB10 on");
	wr32(PHY_PHYA_REG1, rd32(PHY_PHYA_REG1) | (1u << 29));
	delay_us(250);

	con_str("USBB", "2) phy: leave uart/gpio, clear dtm0 forces");
	v = rd32(PHY_U2PHYDTM0);
	v &= ~(0x3u << 30);		/* RG_UART_MODE */
	v &= ~(1u << 26);		/* FORCE_UART_EN */
	v &= ~(1u << 18);		/* FORCE_SUSPENDM */
	v &= ~(1u << 15);		/* UTMI_MUXSEL (undo stage 10) */
	v &= ~(1u << 24);		/* FORCE_DRVVBUS */
	v &= ~((1u << 23) | (1u << 21) | (1u << 20) | (1u << 19) | (1u << 17));
	v &= ~((0xfu << 10) | (1u << 7) | (1u << 6) | (0x3u << 4) | (1u << 2));
	wr32(PHY_U2PHYDTM0, v);
	v = rd32(PHY_U2PHYDTM1);
	v &= ~((1u << 16) | (1u << 15));	/* RG_UART_EN, RG_IP_U2_PORT_POWER */
	wr32(PHY_U2PHYDTM1, v);
	v = rd32(PHY_U2PHYACR4);
	wr32(PHY_U2PHYACR4, v & ~((1u << 9) | (1u << 8)));
	v = rd32(PHY_USBPHYACR6);
	v &= ~(1u << 23);		/* BC1.1 off */
	v = (v & ~(0xfu << 4)) | (0xfu << 4);	/* RG_USB20_DISCTH=0xF */
	wr32(PHY_USBPHYACR6, v);
	delay_us(800);

	con_str("USBB", "3) phy: DTM1 VBUSVALID=1 AVALID=1 SESSEND=0");
	v = rd32(PHY_U2PHYDTM1);
	v |= (1u << 5) | (1u << 2);	/* RG_VBUSVALID, RG_AVALID */
	v &= ~(1u << 4);		/* RG_SESSEND */
	wr32(PHY_U2PHYDTM1, v);
	delay_us(2000);
	con_hex2("USBB", "dtm0=", rd32(PHY_U2PHYDTM0),
		 "dtm1=", rd32(PHY_U2PHYDTM1));
	con_hex2("USBB", "otg_sts=", rd32(IPPC_BASE + 0x18),
		 "sts1=", rd32(IPPC_BASE + IPPC_PW_STS1));

	op = DWC3_BASE + caplength();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 4u);
	delay_us(20000);
	wr32(op + XO_USBCMD, 2u);
	delay_us(20000);
	wr32(op + XO_USBCMD, 1u);
	delay_us(20000);
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(50000);
	ps = rd32(op + XO_PORTSC);
	con_hex2("USBB", "A host_sel: portsc=", ps, "pls=", (ps >> 5) & 0xfu);
	con_hex2("USBB", "otg_sts=", rd32(IPPC_BASE + 0x18),
		 "ccs=", ps & PORTSC_CCS);

	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0xA4Cu);
	delay_us(60000);
	ps = rd32(op + XO_PORTSC);
	con_hex2("USBB", "B +vbus+id: portsc=", ps, "pls=", (ps >> 5) & 0xfu);
	con_hex2("USBB", "otg_sts=", rd32(IPPC_BASE + 0x18),
		 "ccs=", ps & PORTSC_CCS);
}

/* ---- stage 12: same as 11, short labels + long port polling ----------- */
static void usb_port_line(const char *tag)
{
	uintptr_t op = DWC3_BASE + caplength();
	uint32_t ps = rd32(op + XO_PORTSC);

	con_hex2(tag, "ps=", ps, "pls=", (ps >> 5) & 0xfu);
	con_hex2(tag, "ccs=", ps & PORTSC_CCS, "ped=", ps & PORTSC_PED);
	con_hex2(tag, "spd=", (ps >> 10) & 0xfu, "otg=", rd32(IPPC_BASE + 0x18));
}

static void usb_stage12(void)
{
	uint32_t v;
	uintptr_t op;
	int i;

	con_str("USBc", "power seq");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);

	wr32(PHY_PHYA_REG1, rd32(PHY_PHYA_REG1) | (1u << 29));
	delay_us(250);
	v = rd32(PHY_U2PHYDTM0);
	v &= ~(0x3u << 30);
	v &= ~((1u << 26) | (1u << 24) | (1u << 23) | (1u << 21) | (1u << 20) |
	       (1u << 19) | (1u << 18) | (1u << 17) | (1u << 15));
	v &= ~((0xfu << 10) | (1u << 7) | (1u << 6) | (0x3u << 4) | (1u << 2));
	wr32(PHY_U2PHYDTM0, v);
	wr32(PHY_U2PHYDTM1, rd32(PHY_U2PHYDTM1) & ~((1u << 16) | (1u << 15)));
	wr32(PHY_U2PHYACR4, rd32(PHY_U2PHYACR4) & ~((1u << 9) | (1u << 8)));
	v = rd32(PHY_USBPHYACR6);
	v &= ~(1u << 23);
	wr32(PHY_USBPHYACR6, (v & ~(0xfu << 4)) | (0xfu << 4));
	delay_us(800);
	v = rd32(PHY_U2PHYDTM1);
	v |= (1u << 5) | (1u << 2);
	v &= ~(1u << 4);
	wr32(PHY_U2PHYDTM1, v);
	delay_us(2000);
	con_hex2("USBc", "dtm0=", rd32(PHY_U2PHYDTM0),
		 "dtm1=", rd32(PHY_U2PHYDTM1));

	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000A4Cu);
	delay_us(10000);

	op = DWC3_BASE + caplength();
	wr32(op + XO_USBCMD, 2u);
	delay_us(30000);
	wr32(op + XO_USBCMD, 1u);
	delay_us(30000);
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(30000);
	con_hex2("USBc", "cmd=", rd32(op + XO_USBCMD),
		 "sts=", rd32(op + XO_USBSTS));

	for (i = 0; i < 6; i++) {
		usb_port_line("USBc");
		delay_us(200000);
	}
}

/* ---- stage 13: real xHCI enumeration (one device, polled) -------------- */
#define RING_TRBS 32

static uint32_t x_dcbaa[64] __attribute__((aligned(64)));
static uint32_t x_cmd[RING_TRBS][4] __attribute__((aligned(64)));
static uint32_t x_evt[32][4] __attribute__((aligned(64)));
static uint32_t x_erst[4] __attribute__((aligned(64)));
static uint32_t x_inctx[24] __attribute__((aligned(64)));
static uint32_t x_devctx[256] __attribute__((aligned(64)));
static uint32_t x_ep0[RING_TRBS][4] __attribute__((aligned(64)));
static uint8_t  x_desc[64] __attribute__((aligned(64)));

static uintptr_t x_rt, x_db;
static uint32_t x_slot;
static int x_cmd_i, x_cmd_cyc = 1, x_evt_i, x_evt_cyc = 1;
static int x_ep0_n;	/* EP0 control transfers issued so far */

static void xhci_zero(uint32_t *p, int words)
{
	int i;
	for (i = 0; i < words; i++)
		p[i] = 0;
	__asm__ volatile("dsb sy" ::: "memory");
}

static void xhci_wr64(uintptr_t reg, uint32_t lo, uint32_t hi)
{
	wr32(reg, lo);
	wr32(reg + 4, hi);
}

/* wait for one event TRB; copies it into out[] */
static int xhci_evt(uint32_t *out, uint32_t ms)
{
	uint64_t t0 = cnt();
	while ((cnt() - t0) < (uint64_t)ms * 13000u) {
		volatile uint32_t *t = x_evt[x_evt_i];
		if ((t[3] & 1u) == (uint32_t)x_evt_cyc) {
			out[0] = t[0];
			out[1] = t[1];
			out[2] = t[2];
			out[3] = t[3];
			x_evt_i = (x_evt_i + 1) % 32;
			if (x_evt_i == 0)
				x_evt_cyc ^= 1;
			xhci_wr64(x_rt + 0x38,
				  (uint32_t)(uintptr_t)&x_evt[x_evt_i][0] | 8u, 0);
			return 0;
		}
	}
	return -1;
}

/* push a command TRB, ring doorbell 0, return completion code (-1 = timeout) */
static uint32_t x_last_ev[4]	__attribute__((aligned(16)));
static uint32_t x_evlog[6][4];	/* every transfer event we consumed */
static int x_evlog_n;

static int xhci_cmd(uint32_t d0, uint32_t d1, uint32_t d2, uint32_t type,
		    uint32_t slot, uint32_t *slot_out)
{
	uint32_t *t = x_cmd[x_cmd_i];
	uint32_t ev[4];
	uint64_t t0;

	t[0] = d0;
	t[1] = d1;
	t[2] = d2;
	/* TRB control dword: Slot ID in bits[31:24], TRB type in bits[15:10],
	 * cycle state in bit 0. Getting the slot wrong makes the controller
	 * answer with completion code 5 (TRB Error). */
	t[3] = (slot << 24) | (type << 10) | (uint32_t)x_cmd_cyc;
	x_cmd_i = (x_cmd_i + 1) % RING_TRBS;
	if (x_cmd_i == 0)
		x_cmd_cyc ^= 1;
	__asm__ volatile("dsb sy" ::: "memory");
	wr32(x_db, 0);

	t0 = cnt();
	while ((cnt() - t0) < 500ull * 13000u) {
		if (xhci_evt(ev, 200))
			continue;
		if (((ev[3] >> 10) & 0x3fu) != 33)	/* command completion */
			continue;
		if (slot_out)
			*slot_out = (ev[3] >> 24) & 0xffu;
		x_last_ev[0] = ev[0];
		x_last_ev[1] = ev[1];
		x_last_ev[2] = ev[2];
		x_last_ev[3] = ev[3];
		return (int)(ev[2] >> 24);
	}
	return -1;
}

static void usb_stage13(void)
{
	uint32_t v, ps, hcs1, pagesize, speed, mps, cc, ev[4];
	uintptr_t op;
	int i;

	con_str("USBd", "power + phy");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	wr32(PHY_PHYA_REG1, rd32(PHY_PHYA_REG1) | (1u << 29));
	delay_us(250);
	v = rd32(PHY_U2PHYDTM0);
	v &= ~(0x3u << 30);
	v &= ~((1u << 26) | (1u << 24) | (1u << 23) | (1u << 21) | (1u << 20) |
	       (1u << 19) | (1u << 18) | (1u << 17) | (1u << 15));
	v &= ~((0xfu << 10) | (1u << 7) | (1u << 6) | (0x3u << 4) | (1u << 2));
	wr32(PHY_U2PHYDTM0, v);
	wr32(PHY_U2PHYDTM1, rd32(PHY_U2PHYDTM1) & ~((1u << 16) | (1u << 15)));
	wr32(PHY_U2PHYACR4, rd32(PHY_U2PHYACR4) & ~((1u << 9) | (1u << 8)));
	v = rd32(PHY_USBPHYACR6);
	v &= ~(1u << 23);
	wr32(PHY_USBPHYACR6, (v & ~(0xfu << 4)) | (0xfu << 4));
	delay_us(800);
	v = rd32(PHY_U2PHYDTM1);
	v |= (1u << 5) | (1u << 2);
	v &= ~(1u << 4);
	wr32(PHY_U2PHYDTM1, v);
	delay_us(2000);
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000A4Cu);
	delay_us(10000);

	op = DWC3_BASE + caplength();
	hcs1 = rd32(DWC3_BASE + 0x04);
	con_hex2("USBd", "op=", (uint32_t)op, "hcs1=", hcs1);
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	pagesize = rd32(op + 0x08);
	con_hex2("USBd", "db=", (uint32_t)x_db, "rt=", (uint32_t)x_rt);
	con_hex2("USBd", "pagesize=", pagesize, "maxslots=", hcs1 & 0xffu);

	con_str("USBd", "HCRST");
	wr32(op + XO_USBCMD, 2u);
	delay_us(50000);
	con_hex2("USBd", "cmd=", rd32(op + XO_USBCMD),
		 "cnr=", rd32(op + XO_USBSTS) & (1u << 11));

	wr32(op + XO_CONFIG, 1u);
	xhci_zero(x_dcbaa, 64);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	/* The wrapper shifted CRCR by +8, so DCBAAP may sit elsewhere too. Write
	 * the array pointer to every plausible offset at once: whichever is the
	 * real DCBAAP will make the controller write the device context, which we
	 * can then see in memory (x_devctx[0] != 0). Done after Enable Slot and
	 * right before Address Device, so a stray write cannot break this run. */
	xhci_wr64(op + 0x30, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + 0x38, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + 0x40, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + 0x48, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);		/* IMAN: IE=1 */
	wr32(op + XO_USBCMD, 1u);	/* RS */
	delay_us(30000);
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(30000);
	ps = rd32(op + XO_PORTSC);
	con_hex2("USBd", "ps=", ps, "pls=", (ps >> 5) & 0xfu);
	con_hex2("USBd", "ccs=", ps & PORTSC_CCS, "spd=", (ps >> 10) & 0xfu);
	if (!(ps & PORTSC_CCS)) {
		con_str("USBd", "no device on the port (VBUS?)");
		return;
	}

	speed = (ps >> 10) & 0xfu;
	mps = (speed == 1u) ? 8u : 64u;

	cc = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &x_slot);	/* Enable Slot */
	con_hex2("USBd", "enslot cc=", cc, "slot=", x_slot);
	if (cc != 1u)
		return;

	xhci_zero(x_inctx, 24);
	xhci_zero(x_devctx, 256);
	x_inctx[1] = 3u;			/* Add A0 + A1 */
	x_inctx[8] = (speed << 20) | (1u << 27);
	x_inctx[9] = (1u << 16);		/* root hub port 1 */
	x_inctx[17] = (3u << 1) | (4u << 3) | (mps << 16);
	x_inctx[18] = (uint32_t)(uintptr_t)&x_ep0[0][0] | 1u;
	/* Fill every plausible DCBAA entry index so the controller writes the
	 * device context regardless of how it indexes the array (0-based entry
	 * for slot 1, slot-based entry 1, or entry 2). If x_devctx[0] stays 0
	 * after Address Device, this wrapper simply keeps contexts internally. */
	x_dcbaa[0] = (uint32_t)(uintptr_t)&x_devctx[0];
	x_dcbaa[2 * x_slot] = (uint32_t)(uintptr_t)&x_devctx[0];
	x_dcbaa[2 * x_slot + 2] = (uint32_t)(uintptr_t)&x_devctx[0];
	xhci_zero(&x_ep0[0][0], RING_TRBS * 4);

	cc = (uint32_t)xhci_cmd((uint32_t)(uintptr_t)&x_inctx[0], 0, 0,
				11u, x_slot, 0);
	con_hex2("USBd", "addr cc=", cc, "x=", 0);
	if (cc != 1u)
		return;

	/* GET_DESCRIPTOR(dev, 18) on EP0 */
	x_ep0[0][0] = 0x01000680u;
	x_ep0[0][1] = 0x00120000u;
	x_ep0[0][2] = 8u;
	x_ep0[0][3] = (2u << 10) | (1u << 6) | 1u;
	x_ep0[1][0] = (uint32_t)(uintptr_t)&x_desc[0];
	x_ep0[1][1] = 0;
	x_ep0[1][2] = 18u;
	x_ep0[1][3] = (3u << 10) | (1u << 5) | 1u;
	x_ep0[2][0] = 0;
	x_ep0[2][1] = 0;
	x_ep0[2][2] = 0;
	x_ep0[2][3] = (4u << 10) | (1u << 16) | 1u;

	__asm__ volatile("dsb sy" ::: "memory");
	wr32(x_db + x_slot * 4u, 1u);
	for (i = 0; i < 20; i++) {
		if (xhci_evt(ev, 200))
			continue;
		if (((ev[3] >> 10) & 0x3fu) == 32u && ((ev[3] >> 16) & 0x1fu) == 1u)
			return (int)(ev[2] >> 24);
	}
	return -1;
}

/* ---- BOT/SCSI over bulk (uses the endpoints found by enumeration) ------ */
static uint32_t x_bulk[6][RING_TRBS][4] __attribute__((aligned(64)));
static int x_bulk_i[6];
static int x_bulk_cyc[6] = { 1, 1, 1, 1, 1, 1 };
static uint8_t x_cbw[32] __attribute__((aligned(64)));
static uint8_t x_csw[16] __attribute__((aligned(64)));
static uint8_t x_blk[512] __attribute__((aligned(64)));
static uint32_t x_ictx2[64] __attribute__((aligned(64)));
static int x_dci_in, x_dci_out;

/* Doorbell targets: normally the DCI itself, but this wrapper may expect the
 * endpoint number instead - that is exactly what we have to measure. */
static int x_tg_out = 4;
static int x_tg_in = 3;

static int set_deq(int dci, uint32_t ptr)
{
	uint32_t s = 0;

	return xhci_cmd(ptr | 1u, 0, (uint32_t)dci << 16, 16u, x_slot, &s);
}

static int reset_ep(int dci)
{
	uint32_t s = 0;

	return xhci_cmd(0, 0, (uint32_t)dci << 16, 15u, x_slot, &s);
}

/* One zero-length transfer: completes immediately if the endpoint is alive,
 * unlike a bulk IN which would just NAK forever. Returns the raw event code. */
static int bulk_probe(int dci, int target, int in)
{
	uint32_t ev[4], *t;
	int i, idx = x_bulk_i[dci], cyc = x_bulk_cyc[dci];

	t = x_bulk[dci][idx];
	t[0] = (uint32_t)(uintptr_t)x_blk;
	t[1] = 0;
	t[2] = 0;
	t[3] = (1u << 10) | (1u << 5) | (in ? (1u << 16) : 0u) |
	       (uint32_t)cyc | ((uint32_t)x_slot << 24);
	idx++;
	if (idx >= RING_TRBS) {
		idx = 0;
		x_bulk_cyc[dci] ^= 1;
	}
	x_bulk_i[dci] = idx;
	__asm__ volatile("dsb sy" ::: "memory");
	for (i = 0; i < 50; i++)
		if (xhci_evt(ev, 1))
			break;
	wr32(x_db + x_slot * 4u, (uint32_t)target);
	for (i = 0; i < 15; i++) {
		if (xhci_evt(ev, 100))
			continue;
		if (((ev[3] >> 10) & 0x3fu) != 32u)
			continue;
		return (int)(ev[2] >> 24);
	}
	return -1;
}

static int xhci_bulk(int dci, int in, uint8_t *buf, uint32_t len)
{
	uint32_t ev[4], *t;
	int i, idx = x_bulk_i[dci], cyc = x_bulk_cyc[dci];

	/* Slot 31 is a Link TRB, never a Normal TRB. Publish its cycle
	 * for this lap before making the last data TRB available. */
	if (idx == RING_TRBS - 2) {
		uint32_t *link = x_bulk[dci][RING_TRBS - 1];
		link[0] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0];
		link[1] = 0;
		link[2] = 0;
		link[3] = (6u << 10) | (1u << 1) | (uint32_t)cyc;
		__asm__ volatile("dsb sy" ::: "memory");
	}
	t = x_bulk[dci][idx];
	t[0] = (uint32_t)(uintptr_t)buf;
	t[1] = 0;
	t[2] = len & 0xffffu;
	/* Normal TRB: direction belongs to the endpoint, not this TRB. */
	t[3] = (1u << 10) | (1u << 5) | (in ? (1u << 2) : 0u) |
	       (uint32_t)cyc;
	idx++;
	if (idx >= RING_TRBS - 1) {
		idx = 0;
		x_bulk_cyc[dci] ^= 1;
	}
	x_bulk_i[dci] = idx;
	__asm__ volatile("dsb sy" ::: "memory");
	for (i = 0; i < 50; i++) {
		if (xhci_evt(ev, 1))
			break;
	}
	wr32(x_db + x_slot * 4u, (uint32_t)dci);
	for (i = 0; i < 41; i++) {
		if (xhci_evt(ev, 100))
			continue;
		if (((ev[3] >> 10) & 0x3fu) != 32u)
			continue;
		if (((ev[3] >> 24) & 0xffu) != x_slot ||
		    ((ev[3] >> 16) & 0x1fu) != (uint32_t)dci)
			continue;
		if ((ev[2] >> 24) != 1u || (ev[2] & 0xffffffu))
		con_hex2("BULK", "dci_cc=", ((uint32_t)dci << 16) | (ev[2] >> 24),
		         "remaining=", ev[2] & 0xffffffu);
		return (int)(ev[2] >> 24);

	}
	con_hex2("BTIME", "dci=", (uint32_t)dci, "idx_cyc=",
	         ((uint32_t)(((uintptr_t)t - (uintptr_t)&x_bulk[dci][0][0]) / 16u) << 8) | (uint32_t)cyc);
	return -1;
}

static int xhci_ctrl_nodata(uint8_t bmReq, uint8_t bReq, uint16_t wValue,
			    uint16_t wIndex)
{
	uint32_t ev[4];
	int i, b, rc = -1;

	b = x_ep0_n * 3;
	if (b > RING_TRBS - 4)
		b = RING_TRBS - 4;
	x_ep0_n++;
	x_ep0[b + 0][0] = (uint32_t)bmReq | ((uint32_t)bReq << 8) |
			  ((uint32_t)(wValue & 0xff) << 16) |
			  ((uint32_t)(wValue >> 8) << 24);
	x_ep0[b + 0][1] = (uint32_t)(wIndex & 0xff) |
			  ((uint32_t)(wIndex >> 8) << 8);
	x_ep0[b + 0][2] = 8u;
	/* TRT = 0: no data stage. Status stage of a no-data control transfer is IN. */
	x_ep0[b + 0][3] = (2u << 10) | (1u << 6) | 1u |
			  ((uint32_t)x_slot << 24);
	x_ep0[b + 1][0] = 0;
	x_ep0[b + 1][1] = 0;
	x_ep0[b + 1][2] = 0;
	x_ep0[b + 1][3] = (4u << 10) | (1u << 16) | (1u << 5) | 1u |
			  ((uint32_t)x_slot << 24);
	__asm__ volatile("dsb sy" ::: "memory");
	for (i = 0; i < 50; i++) {
		if (xhci_evt(ev, 1))
			break;
	}
	wr32(x_db + x_slot * 4u, 1u);
	for (i = 0; i < 41; i++) {
		if (xhci_evt(ev, 100))
			continue;
		if (((ev[3] >> 10) & 0x3fu) != 32u)
			continue;
		if ((ev[2] >> 24) == 1u)
			return 1;
		if (rc < 0)
			rc = (int)(ev[2] >> 24);
	}
	return rc;
}

static void ep_build(int dci_in, int dci_out, uint32_t mps_in, uint32_t mps_out)
{
	int k;

	xhci_zero(x_ictx2, 48);
	x_ictx2[0] = 0;
	for (k = 0; k < 2; k++) {
		int dci = k ? dci_out : dci_in;
		int type = k ? 2 : 6;		/* bulk OUT / bulk IN */
		uint32_t mp = k ? mps_out : mps_in;
		int base;

		/* terminate this endpoint ring with a Link TRB */
		x_bulk[dci][RING_TRBS - 1][0] =
			(uint32_t)(uintptr_t)&x_bulk[dci][0][0];
		x_bulk[dci][RING_TRBS - 1][1] = 0;
		x_bulk[dci][RING_TRBS - 1][2] = 0;
		x_bulk[dci][RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;
		base = 8 + (dci - 1) * 8;
		x_ictx2[base + 0] = 0;		/* EP state 0, interval 0 */
		x_ictx2[base + 1] = (3u << 1) | ((uint32_t)type << 3) | (mp << 16);
		x_ictx2[base + 2] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0] | 1u;
		x_ictx2[base + 3] = 0;
	}
}

/* One Configure Endpoint command with an explicit add-mask / LAST_CTX. */
static uint32_t ep_cfg_try(uint32_t addmask, uint32_t lastdci)
{
	uint32_t slot2 = 0;

	x_ictx2[1] = addmask;
	x_ictx2[8] = (3u << 20) | (lastdci << 27);	/* HS, Context Entries */
	x_ictx2[9] = (1u << 16);			/* root hub port 1 */
	x_ictx2[11] = 0;			/* input: address 0 */
	return (uint32_t)xhci_cmd((uint32_t)(uintptr_t)&x_ictx2[0], 0, 0, 12u,
				  x_slot, &slot2);
}

/* One Bulk-Only Transport command: CBW -> optional data -> CSW.
 * Returns bCSWStatus (0 = good) or a negative synthetic error. */
static int bot(const uint8_t *cdb, int cdblen, int dir_in, uint32_t dlen,
	       uint8_t *dat)
{
	int i, cc;

	for (i = 0; i < 32; i++)
		x_cbw[i] = 0;
	x_cbw[0] = 0x55;	x_cbw[1] = 0x53;	x_cbw[2] = 0x42;	x_cbw[3] = 0x43;
	x_cbw[4] = 0x78;	x_cbw[5] = 0x56;	x_cbw[6] = 0x34;	x_cbw[7] = 0x12;
	x_cbw[8] = (uint8_t)dlen;
	x_cbw[9] = (uint8_t)(dlen >> 8);
	x_cbw[10] = (uint8_t)(dlen >> 16);
	x_cbw[11] = (uint8_t)(dlen >> 24);
	x_cbw[12] = (uint8_t)(dir_in ? 0x80 : 0x00);
	x_cbw[13] = 0;
	x_cbw[14] = (uint8_t)cdblen;
	for (i = 0; i < cdblen && i < 16; i++)
		x_cbw[15 + i] = cdb[i];
	__asm__ volatile("dsb sy" ::: "memory");
	cc = xhci_bulk(x_dci_out, 0, x_cbw, 31);
	if (cc != 1)
		return -100 - cc;
	if (dlen) {
		cc = xhci_bulk(x_dci_in, 1, dat, dlen);
		if (cc != 1)
			return -200 - cc;
	}
	for (i = 0; i < 16; i++)
		x_csw[i] = 0;
	cc = xhci_bulk(x_dci_in, 1, x_csw, 13);
	if (cc != 1)
		return -300 - cc;
	if (x_csw[0] != 0x55 || x_csw[1] != 0x53 || x_csw[2] != 0x42 ||
	    x_csw[3] != 0x53 || x_csw[4] != 0x78 || x_csw[5] != 0x56 ||
	    x_csw[6] != 0x34 || x_csw[7] != 0x12 || x_csw[12] > 2)
		return -400;
	return x_csw[12];
}

/* Device context lives in memory and the controller writes it, so these two
 * reads tell us what the CONTROLLER thinks, not what we hope. EP0 state:
 * 0=Disabled 1=Running 2=Halted 3=Stopped 4=Error. Slot state: 0=Disabled
 * 1=Default 2=Addressed 3=Configured. */
static uint32_t ep0_state(void)		{ return x_devctx[8] & 7u; }
static uint32_t slot_state(void)	{ return (x_devctx[3] >> 27) & 0x1fu; }

/* Build the Address Device input context (EP0 only) and issue the command. */
static int addr_dev(void)
{
	uint32_t s = 0;

	xhci_zero(x_inctx, 24);
	xhci_zero(x_devctx, 256);
	x_inctx[1] = 3u;
	x_inctx[8] = (3u << 20) | (1u << 27);
	x_inctx[9] = (1u << 16);
	x_inctx[17] = (3u << 1) | (4u << 3) | (64u << 16);
	x_inctx[18] = (uint32_t)(uintptr_t)&x_ep0[0][0] | 1u;
	x_dcbaa[2 * x_slot] = (uint32_t)(uintptr_t)&x_devctx[0];
	return xhci_cmd((uint32_t)(uintptr_t)&x_inctx[0], 0, 0, 11u, x_slot, &s);
}

/* Exactly one endpoint context, at an explicit DCI. */
/* Fill the context for one DCI: CErr=3, given type, MPS, and a ring with a
 * Link TRB terminator. */
static void ep_ctx_at(int dci, uint32_t type, uint32_t mps)
{
	int base = 8 + (dci - 1) * 8;

	x_bulk[dci][RING_TRBS - 1][0] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0];
	x_bulk[dci][RING_TRBS - 1][1] = 0;
	x_bulk[dci][RING_TRBS - 1][2] = 0;
	x_bulk[dci][RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;
	x_ictx2[base + 0] = 0;
	x_ictx2[base + 1] = (3u << 1) | (type << 3) | (mps << 16);
	x_ictx2[base + 2] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0] | 1u;
	x_ictx2[base + 3] = 0;
}

/* Same, but with the context index and the ring given separately: this
 * wrapper reads the context for endpoint DCI d at input-context index d+1
 * (proved by: adding DCI 3 required a valid context at index 4, adding DCI 4
 * required a valid context at 5, and the doorbell for an enabled endpoint
 * produced no event because its ring pointer had come from the next context). */
static void ep_ctx_ring(int idx, uint32_t type, uint32_t mps, int ring)
{
	int base = 8 + (idx - 1) * 8;

	x_bulk[ring][RING_TRBS - 1][0] = (uint32_t)(uintptr_t)&x_bulk[ring][0][0];
	x_bulk[ring][RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;
	x_ictx2[base + 0] = 0;
	x_ictx2[base + 1] = (3u << 1) | (type << 3) | (mps << 16);
	x_ictx2[base + 2] = (uint32_t)(uintptr_t)&x_bulk[ring][0][0] | 1u;
	x_ictx2[base + 3] = 0;
}

static void ep_build_one(int dci, int type, uint32_t mps)
{
	int base;

	xhci_zero(x_ictx2, 48);
	x_bulk[dci][RING_TRBS - 1][0] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0];
	x_bulk[dci][RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;
	base = 8 + (dci - 1) * 8;
	x_ictx2[base + 1] = (3u << 1) | ((uint32_t)type << 3) | (mps << 16);
	x_ictx2[base + 2] = (uint32_t)(uintptr_t)&x_bulk[dci][0][0] | 1u;
}

/* Drop one endpoint again (add_flags = SLOT only), so the next matrix cell
 * starts from a clean state without touching the slot itself: Disable Slot
 * does not free the slot on this wrapper (MaxSlots=1). */
static uint32_t ep_cfg_drop(int dci)
{
	uint32_t s = 0;

	x_ictx2[0] = 1u << dci;	/* drop flags */
	x_ictx2[1] = 1u;		/* add SLOT only */
	x_ictx2[8] = (3u << 20) | (1u << 27);
	x_ictx2[9] = (1u << 16);
	x_ictx2[11] = 0;
	return (uint32_t)xhci_cmd((uint32_t)(uintptr_t)&x_ictx2[0], 0, 0, 12u,
				  x_slot, &s);
}

/* ---- stage 14: enum + config descriptor + configure endpoints ---------- */
static uint8_t x_cfg[512] __attribute__((aligned(64)));

static int xhci_ctrl_in(uint8_t *buf, uint32_t len, uint8_t bmReq, uint8_t bReq,
			uint16_t wValue, uint16_t wIndex)
{
	uint32_t ev[4];
	int i, b, str_code = -1;

	/* The EP0 ring is append-only: the controller keeps its own dequeue
	 * pointer and advances 3 TRBs per control transfer, so writing the same
	 * three TRBs again (as the first version did) leaves it staring at an
	 * un-owned TRB - the second transfer died with "USB Transaction Error"
	 * (code 4) even though the device descriptor had just worked.
	 * 16 TRBs give five transfers, which is enough for enumeration. */
	b = x_ep0_n * 3;
	if (b > RING_TRBS - 4)
		b = RING_TRBS - 4;
	x_ep0_n++;

	x_ep0[b + 0][0] = (uint32_t)bmReq | ((uint32_t)bReq << 8) |
		      ((uint32_t)(wValue & 0xff) << 16) | ((uint32_t)(wValue >> 8) << 24);
	x_ep0[b + 0][1] = (uint32_t)(wIndex & 0xff) | ((uint32_t)(wIndex >> 8) << 8) |
		      ((uint32_t)(len & 0xff) << 16);
	x_ep0[b + 0][2] = 8u;
	/* Setup Stage TRB, exactly as the vendor kernel builds it
	 * (drivers/usb/host/xhci-ring.c, xhci_queue_ctrl_tx):
	 *   field = TRB_IDT | TRB_TYPE(TRB_SETUP) | TRB_TX_TYPE(TRB_DATA_IN)
	 * i.e. TRT=3, no CH, no IOC - setup/data/status are separate TDs. */
	x_ep0[b + 0][3] = (3u << 16) | (2u << 10) | (1u << 6) | 1u |
		      ((uint32_t)x_slot << 24);		/* setup: TRT=IN, IDT */
	x_ep0[b + 1][0] = (uint32_t)(uintptr_t)buf;
	x_ep0[b + 1][1] = 0;
	x_ep0[b + 1][2] = len;
	/* Data Stage: TRB_ISP | TRB_TYPE(TRB_DATA) | TRB_DIR_IN, no IOC. */
	x_ep0[b + 1][3] = (3u << 10) | (1u << 16) | (1u << 2) | 1u |
		      ((uint32_t)x_slot << 24);		/* data IN, ISP */
	x_ep0[b + 2][0] = 0;
	x_ep0[b + 2][1] = 0;
	x_ep0[b + 2][2] = 0;
	/* Status Stage carries IOC - this is where completion is reported. */
	x_ep0[b + 2][3] = (4u << 10) | (1u << 5) | 1u |
		      ((uint32_t)x_slot << 24);		/* status OUT, IOC */
	__asm__ volatile("dsb sy" ::: "memory");
	/* Real drain: xhci_evt(.., 0) returns immediately without looking at the
	 * ring, so the previous version never drained anything. 1 ms per poll is
	 * enough and 50 polls bound the loop. */
	for (i = 0; i < 50; i++) {
		if (xhci_evt(ev, 1))
			break;
	}
	wr32(x_db + x_slot * 4u, 1u);
	/* Success wins. Measured behaviour of this wrapper: the data stage
	 * reports code 1, the trailing status stage reports code 4 (with the
	 * payload already in the buffer - see devdesc=4 with a valid VID/PID).
	 * Address matching turned out to be unreliable, so take the first code-1
	 * event after the doorbell and only report an error if none arrives. */
	for (i = 0; i < 41; i++) {
		uint32_t code;

		if (xhci_evt(ev, 100))
			continue;
		if (((ev[3] >> 10) & 0x3fu) != 32u)	/* transfer event */
			continue;
		if (x_evlog_n < 6) {
			x_evlog[x_evlog_n][0] = ev[0];
			x_evlog[x_evlog_n][1] = ev[1];
			x_evlog[x_evlog_n][2] = ev[2];
			x_evlog[x_evlog_n][3] = ev[3];
			x_evlog_n++;
		}
		x_last_ev[0] = ev[0];
		x_last_ev[1] = ev[1];
		x_last_ev[2] = ev[2];
		x_last_ev[3] = ev[3];
		code = ev[2] >> 24;
		if (code == 1u)
			return 1;
		if (str_code < 0)
			str_code = (int)code;
	}
	return str_code;
}

/* ---- stage 60 instrumentation: register map + context signature --------- */
static int x_diag;

/* Where did the controller put the slot/endpoint contexts? A 1 KB dump is 16
 * screen lines; nz/first/w8 answers "wrote nothing / wrote somewhere else". */
static void ctx_sig(const char *tag, volatile uint32_t *p, int n)
{
	int i, nz = 0, first = -1;
	uint32_t fv = 0;

	for (i = 0; i < n; i++) {
		if (!p[i])
			continue;
		nz++;
		if (first < 0) {
			first = i;
			fv = p[i];
		}
	}
	con_hex2(tag, "nz=", (uint32_t)nz,
		 "first=", first < 0 ? 0xffffffffu : (uint32_t)first);
	con_hex2(tag, "fv=", fv, "w8=", p[8]);
}

/* Which of 0x28 / 0x30 / 0x38 latches a write, and where the value lands.
 * Only safe once the controller is stopped: it overwrites DCBAAP/CONFIG. */
static void xhci_map_probe(const char *tag, uintptr_t op)
{
	uint32_t w28 = rd32(op + 0x28), w30 = rd32(op + 0x30);
	uint32_t w38 = rd32(op + 0x38);

	wr32(op + 0x28, 0xa5a50028u);
	wr32(op + 0x30, 0xa5a50030u);
	wr32(op + 0x38, 0xa5a50038u);
	delay_us(2000);
	con_hex2(tag, "rd28=", rd32(op + 0x28), "rd30=", rd32(op + 0x30));
	con_hex2(tag, "rd38=", rd32(op + 0x38), "was28=", w28);
	wr32(op + 0x28, w28);
	wr32(op + 0x30, w30);
	wr32(op + 0x38, w38);
}

static void usb_stage14(void)
{
	uint32_t v, ps, hcs1, pagesize, speed, mps, cc, cc2, cc3, rc1, rc2, slot2;
	uint32_t v0, v1, v2, v3, v4, v5;
	uintptr_t op;
	int i, loop, cfgval = 0, ep_in = 0, ep_out = 0, mps_in = 0, mps_out = 0;
	int iface = 0, subcls = 0, proto = 0;

	con_str("USBe", "power + phy");
	x_ep0_n = 0;			/* fresh EP0 ring for this round */
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	wr32(PHY_PHYA_REG1, rd32(PHY_PHYA_REG1) | (1u << 29));
	delay_us(250);
	v = rd32(PHY_U2PHYDTM0);
	v &= ~(0x3u << 30);
	v &= ~((1u << 26) | (1u << 24) | (1u << 23) | (1u << 21) | (1u << 20) |
	       (1u << 19) | (1u << 18) | (1u << 17) | (1u << 15));
	v &= ~((0xfu << 10) | (1u << 7) | (1u << 6) | (0x3u << 4) | (1u << 2));
	wr32(PHY_U2PHYDTM0, v);
	wr32(PHY_U2PHYDTM1, rd32(PHY_U2PHYDTM1) & ~((1u << 16) | (1u << 15)));
	wr32(PHY_U2PHYACR4, rd32(PHY_U2PHYACR4) & ~((1u << 9) | (1u << 8)));
	v = rd32(PHY_USBPHYACR6);
	v &= ~(1u << 23);
	wr32(PHY_USBPHYACR6, (v & ~(0xfu << 4)) | (0xfu << 4));
	delay_us(800);
	v = rd32(PHY_U2PHYDTM1);
	v |= (1u << 5) | (1u << 2);
	v &= ~(1u << 4);
	wr32(PHY_U2PHYDTM1, v);
	delay_us(2000);
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000A4Cu);
	delay_us(10000);

	op = DWC3_BASE + caplength();
	hcs1 = rd32(DWC3_BASE + 0x04);
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	con_hex2("USB61", "hcc=", rd32(DWC3_BASE + 0x10), "pages=", rd32(op + 8));
	if (rd32(DWC3_BASE + 0x10) & (1u << 2)) {
		con_str("USB61", "64-byte contexts unsupported: stop");
		return;
	}
	x_cmd_i = x_evt_i = 0;
	x_cmd_cyc = x_evt_cyc = 1;
	pagesize = rd32(op + 0x08);
	wr32(op + XO_USBCMD, 2u);
	delay_us(50000);
	/* CNR (USBSTS bit11) must clear before any op register is writable */
	for (i = 0; i < 200; i++) {
		if (!(rd32(op + XO_USBSTS) & (1u << 11)))
			break;
		delay_us(5000);
	}
	/* CONFIG.MaxSlotsEn: HCSPARAMS1 here reports MaxSlots=1, and enabling
	 * more slots than the controller has makes Enable Slot come back as
	 * "TRB Error" (completion code 5). One slot is all this probe needs. */
	wr32(op + XO_CONFIG, 1u);
	xhci_zero(x_dcbaa, 64);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);
	wr32(op + XO_USBCMD, 1u);
	delay_us(30000);
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	delay_us(30000);
	con_hex2("USBe", "maxslots=", hcs1 & 0xffu, "maxports=", (hcs1 >> 24) & 0xffu);

	/* wait up to 5 s for a device to appear */
	for (loop = 0; loop < 50; loop++) {
		ps = rd32(op + XO_PORTSC);
		if (ps & PORTSC_CCS)
			break;
		delay_us(100000);
	}
	if (!(ps & PORTSC_CCS)) {
		con_str("USBe", "still no device: VBUS 5V missing");
		return;
	}

	/* Port reset. Until the port is reset the device stays in the Default
	 * state and Enable Slot / Address Device are refused, and a port stuck
	 * in Polling (pls=7) never reaches Enabled (pls=0). */
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_PR);
	for (loop = 0; loop < 40; loop++) {
		ps = rd32(op + XO_PORTSC);
		if (ps & PORTSC_PRC)
			break;
		delay_us(10000);
	}
	wr32(op + XO_PORTSC, PORTSC_PP | PORTSC_PRC | PORTSC_CSC);
	delay_us(20000);
	ps = rd32(op + XO_PORTSC);
	con_hex2("USBe", "rst ps=", ps, "pls=", (ps >> 5) & 0xfu);
	con_hex2("USBe", "ped=", ps & PORTSC_PED, "spd=", (ps >> 10) & 0xfu);

	speed = (ps >> 10) & 0xfu;
	mps = (speed == 1u) ? 8u : 64u;
	/* Add flags live in ICC dword 1; dword 0 is Drop Flags. Writing 3 into
	 * dword 0 asked the controller to DROP A0/A1 instead of adding them. */
	cc = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &x_slot);
	if (cc != 1u) {
		delay_us(30000);
		cc = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &x_slot);
	}
	con_hex2("USBe", "enslot=", cc, "slot=", x_slot);
	if (cc != 1u)
		return;
	xhci_zero(x_inctx, 24);
	xhci_zero(x_devctx, 256);
	x_inctx[1] = 3u;
	x_inctx[8] = (speed << 20) | (1u << 27);
	x_inctx[9] = (1u << 16);
	x_inctx[17] = (3u << 1) | (4u << 3) | (mps << 16);
	x_inctx[18] = (uint32_t)(uintptr_t)&x_ep0[0][0] | 1u;
	x_dcbaa[2 * x_slot] = (uint32_t)(uintptr_t)&x_devctx[0];	/* 64-bit entries */
	xhci_zero(&x_ep0[0][0], RING_TRBS * 4);
	cc = (uint32_t)xhci_cmd((uint32_t)(uintptr_t)&x_inctx[0], 0, 0,
				11u, x_slot, 0);
	con_hex2("USBe", "addr=", cc, "x=", 0);
	con_hex2("USBe", "devctx0=", x_devctx[0], "devctx8=", x_devctx[8]);
	if (x_diag) {
		/* What did the four candidate ring registers actually latch, and
		 * did the controller write anything into our device context? */
		con_hex2("DReg", "at28=", rd32(op + 0x28), "at30=", rd32(op + 0x30));
		con_hex2("DReg", "at38=", rd32(op + 0x38), "crcr=", rd32(op + 0x18));
		con_hex2("DReg", "dcbaa0=", x_dcbaa[0], "dcbaa2=", x_dcbaa[2]);
		ctx_sig("DCtx", x_devctx, 256);
	}
	if (cc != 1u)
		return;

	x_evlog_n = 0;
	for (i = 0; i < 64; i++)
		x_desc[i] = 0;
	cc = (uint32_t)xhci_ctrl_in(x_desc, 18, 0x80, 6, 0x0100, 0);
	con_hex2("USBe", "devdesc=", cc,
		 "vid_pid=", (x_desc[8] | (x_desc[9] << 8)) | ((x_desc[10] | (x_desc[11] << 8)) << 16));
	if (cc != 1u)
		return;



	for (i = 0; i < 512; i++)
		x_cfg[i] = 0;
	x_evlog_n = 0;
	cc = (uint32_t)xhci_ctrl_in(x_cfg, 9, 0x80, 6, 0x0200, 0);
	con_hex2("USBe", "cfg9=", cc, "ev2=", x_last_ev[2]);

	if (cc != 1u) {
		/* xHCI way to clear a halted endpoint: Reset Endpoint, then point it
		 * at the triple we are about to fill (Set TR Dequeue Pointer). */
		rc1 = (uint32_t)xhci_cmd(0, 0, (1u << 16), 15u, x_slot, &slot2);
		rc2 = (uint32_t)xhci_cmd((uint32_t)(uintptr_t)&x_ep0[6][0] | 1u, 0,
					 (1u << 16), 16u, x_slot, &slot2);
		con_hex2("USBe", "rst_ep=", rc1, "deq=", rc2);
		x_evlog_n = 0;
		cc = (uint32_t)xhci_ctrl_in(x_cfg, 9, 0x80, 6, 0x0200, 0);
		con_hex2("USBe", "cfg9b=", cc, "cfgd0b=", x_cfg[0]);
		con_hex2("USBe", "ep0st2=", ep0_state(), "nev2=", (uint32_t)x_evlog_n);
	}
	/* If the bytes are a real configuration descriptor, keep them. */
	if (cc != 1u && x_cfg[0] >= 9u && x_cfg[1] == 2u)
		cc = 1u;
	if (cc != 1u)
		return;
	v = x_cfg[2] | (x_cfg[3] << 8);	/* wTotalLength */
	con_hex2("USBe", "cfglen=", v, "cfgval=", x_cfg[5]);
	if (v > 512)
		v = 512;
	cc = (uint32_t)xhci_ctrl_in(x_cfg, v, 0x80, 6, 0x0200, 0);
	if (cc != 1u) {
		delay_us(20000);
		cc = (uint32_t)xhci_ctrl_in(x_cfg, v, 0x80, 6, 0x0200, 0);
	}
	con_hex2("USBe", "cfgfull=", cc, "len=", v);
	if (cc != 1u)
		return;

	/* walk descriptors */
	for (i = 0; i + 2 < (int)v; ) {
		int dlen = x_cfg[i], dtype = x_cfg[i + 1];
		if (dlen < 2 || i + dlen > (int)v)
			break;
		if (dtype == 4) {		/* interface */
			iface = x_cfg[i + 5];	/* bInterfaceClass */
			subcls = x_cfg[i + 6];
			proto = x_cfg[i + 7];
			if (cfgval == 0)
				cfgval = x_cfg[5];
		} else if (dtype == 5) {	/* endpoint */
			uint8_t ea = x_cfg[i + 2];
			uint8_t at = x_cfg[i + 3] & 3u;
			int mp = x_cfg[i + 4] | (x_cfg[i + 5] << 8);
			if (at == 2) {
				if (ea & 0x80) {
					ep_in = ea;
					mps_in = mp;
				} else {
					ep_out = ea;
					mps_out = mp;
				}
			}
		}
		i += dlen;
	}
	con_hex2("USBe", "ep_in=", ep_in, "ep_out=", ep_out);
	con_hex2("USBe", "mps_in=", mps_in, "mps_out=", mps_out);
	con_hex2("USBe", "ifclass=", iface, "slot0=", x_devctx[0]);
	con_hex2("USBe", "subcls=", subcls, "proto=", proto);
	con_str("USB61", "standard input contexts; descriptor DCI");
	if (iface != 8 || subcls != 6 || proto != 0x50 || !ep_in || !ep_out)
		return;
	x_dci_in = 2 * (ep_in & 15) + 1;
	x_dci_out = 2 * (ep_out & 15);
	if (x_dci_in >= 6 || x_dci_out >= 6) {
		con_str("USB61", "endpoint exceeds diagnostic ring array");
		return;
	}
	cc = (uint32_t)xhci_ctrl_nodata(0x00, 0x09, (uint16_t)cfgval, 0);
	con_hex2("USB61", "setcfg=", cc, "dci_in_out=", (x_dci_in << 8) | x_dci_out);
	if (cc != 1) return;
	xhci_zero(x_ictx2, 64);
	for (i = 0; i < 6; i++) {
		xhci_zero(&x_bulk[i][0][0], RING_TRBS * 4);
		x_bulk_i[i] = 0;
		x_bulk_cyc[i] = 1;
	}
	/* Input context index = DCI+1: index 0 is Input Control, 1 Slot. */
	ep_ctx_ring(x_dci_in + 1, 6, mps_in, x_dci_in);
	ep_ctx_ring(x_dci_out + 1, 2, mps_out, x_dci_out);
	x_ictx2[(x_dci_in + 1) * 8 + 4] = 512;
	x_ictx2[(x_dci_out + 1) * 8 + 4] = 512;
	v1 = ep_cfg_try(1u | (1u << x_dci_in) | (1u << x_dci_out),
	               x_dci_in > x_dci_out ? x_dci_in : x_dci_out);
	con_hex2("USB61", "cfg_ep=", v1, "slot_state=", slot_state());
	if (v1 != 1u) return;
	con_str("USBe", "both endpoints configured");
	if (x_diag) {
		con_hex2("DReg2", "at28=", rd32(op + 0x28), "at30=", rd32(op + 0x30));
		ctx_sig("DCtx2", x_devctx, 256);
	}

	{
		uint8_t cdb[16];
		uint32_t t32;
		int k2;

		for (k2 = 0; k2 < 16; k2++)
			cdb[k2] = 0;
		cdb[0] = 0x12;			/* INQUIRY */
		cdb[4] = 36;
		cc = (uint32_t)bot(cdb, 6, 1, 36, x_blk);
		t32 = ((uint32_t)x_blk[8] << 24) | ((uint32_t)x_blk[9] << 16) |
		      ((uint32_t)x_blk[10] << 8) | x_blk[11];
		con_hex2("USBe", "inq=", cc, "vnd=", t32);
		if (cc != 0) return;

		for (k2 = 0; k2 < 16; k2++)
			cdb[k2] = 0;
		cdb[0] = 0x25;			/* READ CAPACITY(10) */
		cc = (uint32_t)bot(cdb, 10, 1, 8, x_blk);
		t32 = ((uint32_t)x_blk[0] << 24) | ((uint32_t)x_blk[1] << 16) |
		      ((uint32_t)x_blk[2] << 8) | x_blk[3];
		con_hex2("USBe", "cap=", cc, "last_lba=", t32);
		con_hex2("USBe", "block_sz=", ((uint32_t)x_blk[4] << 24) |
			((uint32_t)x_blk[5] << 16) | ((uint32_t)x_blk[6] << 8) | x_blk[7],
			"status=", cc);
		if (cc != 0) return;

		for (k2 = 0; k2 < 16; k2++)
			cdb[k2] = 0;
		cdb[0] = 0x28;			/* READ(10) LBA 2 */
		cdb[5] = 2;
		cdb[8] = 1;
		cc = (uint32_t)bot(cdb, 10, 1, 512, x_blk);
		t32 = ((uint32_t)x_blk[0] << 8) | x_blk[1];
		con_hex2("USBe", "rd2=", cc, "sig=", t32);
		/* LBA 2 of a 512-byte sector disk is the HFS+ volume header
		 * (offset 1024). Fields are big-endian. */
		if (cc == 0 && x_blk[0] == 'H' && x_blk[1] == 'X') {
			uint32_t bs = ((uint32_t)x_blk[40] << 24) | ((uint32_t)x_blk[41] << 16) |
				      ((uint32_t)x_blk[42] << 8) | x_blk[43];
			uint32_t tb = ((uint32_t)x_blk[44] << 24) | ((uint32_t)x_blk[45] << 16) |
				      ((uint32_t)x_blk[46] << 8) | x_blk[47];
			uint32_t fb = ((uint32_t)x_blk[48] << 24) | ((uint32_t)x_blk[49] << 16) |
				      ((uint32_t)x_blk[50] << 8) | x_blk[51];
			con_hex2("HFS", "ver=", x_blk[3], "blocksize=", bs);
			con_hex2("HFS", "blocks=", tb, "free=", fb);
			con_hex2("HFS", "size_mb=", (tb / 2048u), "x=", 0);
		} else {
			con_hex2("HFS", "no volume header at 1024", 0, "b0=", x_desc[0]);
		}
	}
	con_str("USBe", "BOT/SCSI done");
}

/* ---- stage 16: MTK i2c5 master, READ-ONLY probe of MT6360 -------------- */
#define I2C5_BASE	0x11016000UL

/* mt65xx i2c register offsets (i2c-mt65xx.c, legacy IP) */
#define IC_DATA_PORT	0x00
#define IC_SLAVE_ADDR	0x04
#define IC_INTR_MASK	0x08
#define IC_INTR_STAT	0x0c
#define IC_CONTROL	0x10
#define IC_TRANSFER_LEN	0x14
#define IC_TRANSAC_LEN	0x18
#define IC_DELAY_LEN	0x1c
#define IC_TIMING	0x20
#define IC_START	0x24
#define IC_EXT_CONF	0x28
#define IC_FIFO_STAT	0x30
#define IC_FIFO_ADDR_CLR 0x38
#define IC_IO_CONFIG	0x40
#define IC_HS		0x48
#define IC_SOFTRESET	0x50
#define IC_DCM_EN	0x54

#define IC_COMP		0x01
#define IC_ACKERR	0x02
#define IC_NACKERR	0x04
#define IC_CTRL_RS	0x02
#define IC_CTRL_CLK_EXT 0x08
#define IC_CTRL_DIR_CHG 0x10
#define IC_CTRL_ACKDET	0x20
#define IC_ST_START_CON	0x8001
#define IC_WRRD		0x0002

static inline uint16_t rd16(uintptr_t a)
{
	uint16_t v = *(volatile uint16_t *)a;
	__asm__ volatile("dsb sy" ::: "memory");
	return v;
}

static inline void wr16(uintptr_t a, uint16_t v)
{
	*(volatile uint16_t *)a = v;
	__asm__ volatile("dsb sy" ::: "memory");
}

/* read one byte from a 16-bit-addressed MT6360 register (SMBus-style) */
static int mt6360_read16(uint8_t slave, uint16_t reg, uint8_t *out)
{
	uint16_t st;
	int i;

	wr16(I2C5_BASE + IC_EXT_CONF, IC_ST_START_CON);
	wr16(I2C5_BASE + IC_SLAVE_ADDR, (uint16_t)(slave << 1));
	wr16(I2C5_BASE + IC_CONTROL, IC_CTRL_ACKDET | IC_CTRL_CLK_EXT |
					 IC_CTRL_DIR_CHG | IC_CTRL_RS);
	wr16(I2C5_BASE + IC_INTR_STAT, IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_FIFO_ADDR_CLR, 1);
	wr16(I2C5_BASE + IC_INTR_MASK, IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_TRANSFER_LEN, (uint16_t)(2u | (1u << 8)));
	wr16(I2C5_BASE + IC_TRANSAC_LEN, IC_WRRD);
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg >> 8));
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg & 0xff));
	wr16(I2C5_BASE + IC_START, 1);

	for (i = 0; i < 600; i++) {
		st = rd16(I2C5_BASE + IC_INTR_STAT);
		if (st & (IC_COMP | IC_ACKERR | IC_NACKERR))
			break;
		delay_us(100);
	}
	wr16(I2C5_BASE + IC_INTR_MASK, 0);
	if (!(st & IC_COMP))
		return -1;
	if (st & IC_ACKERR)
		return -2;
	*out = (uint8_t)rd16(I2C5_BASE + IC_DATA_PORT);
	return 0;
}

static void usb_stage16(void)
{
	uint32_t v;
	uint8_t b = 0;
	int rc;

	con_str("USBi", "mtk i2c5 @11016000, MT6360 read-only");
	v = rd32(INFRA_AO_BASE + INFRA2_STA);
	con_hex2("USBi", "infra2_sta=", v, "i2c5_bits=", (v >> 18) & 7u);
	if ((v >> 18) & 7u) {
		con_str("USBi", "gates closed (bit=1 means off), clearing");
		wr32(INFRA_AO_BASE + INFRA2_CLR, 7u << 18);
		delay_us(2000);
		con_hex2("USBi", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA), "x=", 0);
	}

	con_str("USBi", "soft reset + config");
	wr16(I2C5_BASE + IC_SOFTRESET, 1);
	delay_us(1000);
	wr16(I2C5_BASE + IC_DCM_EN, 0);
	wr16(I2C5_BASE + IC_IO_CONFIG, 0x0003);	/* open drain */
	wr16(I2C5_BASE + IC_TIMING, 0x0737);	/* sample 8, step 55 -> slow */
	wr16(I2C5_BASE + IC_HS, 0);
	wr16(I2C5_BASE + IC_DELAY_LEN, 2);
	wr16(I2C5_BASE + IC_CONTROL, IC_CTRL_ACKDET | IC_CTRL_CLK_EXT);
	con_hex2("USBi", "timing=", rd16(I2C5_BASE + IC_TIMING),
		 "cntrl=", rd16(I2C5_BASE + IC_CONTROL));

	con_str("USBi", "read CHG_CTRL2 (0x0312), expect 0x1B");
	rc = mt6360_read16(0x34, 0x0312, &b);
	con_hex2("USBi", "rc=", (uint32_t)rc, "val=", b);
	if (rc == 0) {
		con_str("USBi", "read CHG_CTRL1 (0x0311), expect 0x00");
		rc = mt6360_read16(0x34, 0x0311, &b);
		con_hex2("USBi", "rc=", (uint32_t)rc, "val=", b);
		con_str("USBi", "I2C to MT6360 WORKS");
	} else {
		con_str("USBi", "no answer: check clk/pins/addr");
	}
}

/* ---- stage 17: i2c5 with the CORRECT (V2) MTK register layout --------- */
#define V2_IO_CONFIG	0x34
#define V2_HS		0x30
#define V2_LTIMING	0x2c
#define V2_MCU_INTR	0x40
#define V2_FIFO_STAT	0xf4
#define V2_DEBUGSTAT	0xe4

static int mt6360_read_v2(uint8_t slave, uint16_t reg, uint8_t *out)
{
	uint16_t st, fifo;
	int i;

	wr16(I2C5_BASE + IC_EXT_CONF, 0x1801);	/* ext_time_config from DT */
	wr16(I2C5_BASE + IC_SLAVE_ADDR, (uint16_t)(slave << 1));
	wr16(I2C5_BASE + IC_INTR_STAT, IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_FIFO_ADDR_CLR, 1);
	wr16(I2C5_BASE + IC_INTR_MASK, IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_TRANSFER_LEN, (uint16_t)(2u | (1u << 8)));
	wr16(I2C5_BASE + IC_TRANSAC_LEN, IC_WRRD);
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg >> 8));
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg & 0xff));
	fifo = rd16(I2C5_BASE + V2_FIFO_STAT);
	wr16(I2C5_BASE + V2_MCU_INTR, 1);	/* I2C_MCU_INTR_EN */
	wr16(I2C5_BASE + IC_START, 1);

	for (i = 0; i < 1000; i++) {
		st = rd16(I2C5_BASE + IC_INTR_STAT);
		if (st & (IC_COMP | IC_ACKERR | IC_NACKERR))
			break;
		delay_us(100);
	}
	wr16(I2C5_BASE + IC_INTR_MASK, 0);
	if (!(st & IC_COMP)) {
		con_hex2("USBj", "fifo_before=", fifo, "intr=", st);
		con_hex2("USBj", "dbgstat=", rd16(I2C5_BASE + V2_DEBUGSTAT),
			 "fifo_now=", rd16(I2C5_BASE + V2_FIFO_STAT));
		return -1;
	}
	if (st & (IC_ACKERR | IC_NACKERR))
		return -2;
	*out = (uint8_t)rd16(I2C5_BASE + IC_DATA_PORT);
	return 0;
}

static void usb_stage17(void)
{
	uint8_t b = 0;
	int rc;

	con_str("USBj", "i2c5 V2 layout, MT6360 read-only");
	con_hex2("USBj", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "i2c5_bits=", (rd32(INFRA_AO_BASE + INFRA2_STA) >> 18) & 7u);

	wr16(I2C5_BASE + IC_SOFTRESET, 1);
	delay_us(2000);
	wr16(I2C5_BASE + V2_IO_CONFIG, 0x0003);	/* open drain */
	wr16(I2C5_BASE + IC_TIMING, 0x073f | 0x0001);	/* slow + timeout en */
	wr16(I2C5_BASE + V2_LTIMING, 0x01ff);
	wr16(I2C5_BASE + V2_HS, 0x0000);
	wr16(I2C5_BASE + IC_DELAY_LEN, 2);
	wr16(I2C5_BASE + IC_CONTROL, IC_CTRL_ACKDET | IC_CTRL_CLK_EXT |
					 IC_CTRL_DIR_CHG | IC_CTRL_RS);
	con_hex2("USBj", "io_cfg=", rd16(I2C5_BASE + V2_IO_CONFIG),
		 "timing=", rd16(I2C5_BASE + IC_TIMING));
	con_hex2("USBj", "ltiming=", rd16(I2C5_BASE + V2_LTIMING),
		 "cntrl=", rd16(I2C5_BASE + IC_CONTROL));

	con_str("USBj", "read CHG_CTRL2 (0x0312), expect 0x1B");
	rc = mt6360_read_v2(0x34, 0x0312, &b);
	con_hex2("USBj", "rc=", (uint32_t)rc, "val=", b);
	if (rc == 0) {
		con_str("USBj", "read CHG_CTRL1 (0x0311), expect 0x00");
		rc = mt6360_read_v2(0x34, 0x0311, &b);
		con_hex2("USBj", "rc=", (uint32_t)rc, "val=", b);
		con_str("USBj", "I2C to MT6360 WORKS");
	} else {
		con_str("USBj", "still no answer");
	}
}

/* ---- stage 18: i2c5 V2 with HW_TIMEOUT + full intr clear -------------- */
#define V2_HW_TIMEOUT	0x4c
#define IC_INTR_BUS_ERR	0x0100
#define IC_INTR_TIMEOUT	0x0020

static int i2c5_xfer_cfg(uint16_t ext_conf, uint16_t timing, uint16_t ltiming,
			 uint8_t slave, uint16_t reg, uint8_t *out)
{
	uint16_t st, fifo;
	int i;

	wr16(I2C5_BASE + IC_SOFTRESET, 1);
	delay_us(2000);
	wr16(I2C5_BASE + V2_IO_CONFIG, 0x0003);
	wr16(I2C5_BASE + IC_TIMING, (uint16_t)(timing | 0x0001));
	wr16(I2C5_BASE + V2_LTIMING, ltiming);
	wr16(I2C5_BASE + V2_HS, 0x0000);
	wr16(I2C5_BASE + IC_DELAY_LEN, 2);
	wr16(I2C5_BASE + IC_CONTROL, IC_CTRL_ACKDET | IC_CTRL_CLK_EXT |
					 IC_CTRL_DIR_CHG | IC_CTRL_RS);
	/* hardware scl timeout, V2 only: must not be left at 0 */
	wr16(I2C5_BASE + V2_HW_TIMEOUT, 0xffff);

	wr16(I2C5_BASE + IC_EXT_CONF, ext_conf);
	wr16(I2C5_BASE + IC_SLAVE_ADDR, (uint16_t)(slave << 1));
	wr16(I2C5_BASE + IC_INTR_STAT, 0x01ff);	/* clear everything */
	wr16(I2C5_BASE + IC_FIFO_ADDR_CLR, 1);
	wr16(I2C5_BASE + IC_INTR_MASK,
	     IC_INTR_BUS_ERR | IC_INTR_TIMEOUT | IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_TRANSFER_LEN, (uint16_t)(2u | (1u << 8)));
	wr16(I2C5_BASE + IC_TRANSAC_LEN, IC_WRRD);
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg >> 8));
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg & 0xff));
	fifo = rd16(I2C5_BASE + V2_FIFO_STAT);
	wr16(I2C5_BASE + V2_MCU_INTR, 1);
	wr16(I2C5_BASE + IC_START, 1);

	for (i = 0; i < 1200; i++) {
		st = rd16(I2C5_BASE + IC_INTR_STAT);
		if (st & 0x0127)
			break;
		delay_us(100);
	}
	wr16(I2C5_BASE + IC_INTR_MASK, 0);
	con_hex2("USBk", "fifo0=", fifo, "intr=", st);
	if (!(st & IC_COMP))
		return -1;
	if (st & (IC_ACKERR | IC_NACKERR | IC_INTR_BUS_ERR | IC_INTR_TIMEOUT))
		return -2;
	*out = (uint8_t)rd16(I2C5_BASE + IC_DATA_PORT);
	return 0;
}

static void usb_stage18(void)
{
	uint8_t b = 0;
	int rc;

	con_str("USBk", "i2c5 V2 + hw_timeout, two speeds");
	con_hex2("USBk", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "i2c5_bits=", (rd32(INFRA_AO_BASE + INFRA2_STA) >> 18) & 7u);

	con_str("USBk", "A: ST 100kHz, ext_conf=0x8001");
	rc = i2c5_xfer_cfg(0x8001, 0x073f, 0x01ff, 0x34, 0x0312, &b);
	con_hex2("USBk", "A rc=", (uint32_t)rc, "val=", b);

	con_str("USBk", "B: FS 400kHz, ext_conf=0x1801");
	b = 0;
	rc = i2c5_xfer_cfg(0x1801, 0x0129, 0x0069, 0x34, 0x0312, &b);
	con_hex2("USBk", "B rc=", (uint32_t)rc, "val=", b);

	if (rc == 0) {
		con_str("USBk", "read CHG_CTRL1 0x0311 (expect 0x00)");
		rc = i2c5_xfer_cfg(0x1801, 0x0129, 0x0069, 0x34, 0x0311, &b);
		con_hex2("USBk", "rc=", (uint32_t)rc, "val=", b);
		con_str("USBk", "I2C to MT6360 WORKS");
	}
}

/* ---- stage 19: i2c_sel mux -> clk26m, then retry ---------------------- */
#define TOP_CK_CFG_9		0xb0
#define TOP_CK_CFG_9_SET	0xb4
#define TOP_CK_CFG_9_CLR	0xb8

static void usb_stage19(void)
{
	uint8_t b = 0;
	int rc;

	con_str("USBm", "i2c5: mux i2c_sel -> clk26m");
	con_hex2("USBm", "cfg9_before=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_9),
		 "upd1=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_UPDATE1));

	/* parent 0 = clk26m: clear the 2-bit field, then commit all updates */
	wr32(TOPCKGEN_BASE + TOP_CK_CFG_9_CLR, 0x3u);
	wr32(TOPCKGEN_BASE + TOP_CK_CFG_UPDATE1, 0x003eu);
	delay_us(2000);
	con_hex2("USBm", "cfg9_after=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_9),
		 "upd1=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_UPDATE1));
	con_hex2("USBm", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "i2c5_bits=", (rd32(INFRA_AO_BASE + INFRA2_STA) >> 18) & 7u);

	con_str("USBm", "retry read CHG_CTRL2 (0x0312)");
	rc = i2c5_xfer_cfg(0x8001, 0x073f, 0x01ff, 0x34, 0x0312, &b);
	con_hex2("USBm", "A rc=", (uint32_t)rc, "val=", b);

	b = 0;
	rc = i2c5_xfer_cfg(0x1801, 0x0129, 0x0069, 0x34, 0x0312, &b);
	con_hex2("USBm", "B rc=", (uint32_t)rc, "val=", b);

	/* also try the TCPC address 0x4e (8-bit register 0x00/hi) */
	b = 0;
	rc = i2c5_xfer_cfg(0x8001, 0x073f, 0x01ff, 0x4e, 0x0000, &b);
	con_hex2("USBm", "tcpc rc=", (uint32_t)rc, "val=", b);

	if (rc == 0)
		con_str("USBm", "I2C ANSWERED - address 0x4e");
}

/* ---- stage 20: pericfg i2c mode enable, then retry -------------------- */
#define PERICFG_BASE		0x10003000UL
#define PERI_I2C_MODE_ENABLE	0x0410

static void usb_stage20(void)
{
	uint8_t b = 0;
	uint16_t before;
	int rc;

	con_str("USBp", "pericfg i2c mode enable + retry");
	before = rd16(PERICFG_BASE + PERI_I2C_MODE_ENABLE);
	wr16(PERICFG_BASE + PERI_I2C_MODE_ENABLE, 0x0003);
	delay_us(2000);
	con_hex2("USBp", "peri_before=", before,
		 "peri_after=", rd16(PERICFG_BASE + PERI_I2C_MODE_ENABLE));
	con_hex2("USBp", "infra2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "cfg9=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_9));

	con_str("USBp", "retry A: ST 100kHz");
	rc = i2c5_xfer_cfg(0x8001, 0x073f, 0x01ff, 0x34, 0x0312, &b);
	con_hex2("USBp", "A rc=", (uint32_t)rc, "val=", b);

	con_str("USBp", "retry B: FS 400kHz");
	b = 0;
	rc = i2c5_xfer_cfg(0x1801, 0x0129, 0x0069, 0x34, 0x0312, &b);
	con_hex2("USBp", "B rc=", (uint32_t)rc, "val=", b);

	if (rc == 0)
		con_str("USBp", "I2C to MT6360 WORKS");
}

/* ---- stage 21: correct i2c5 clocks from DT (i2c0 gate + apdma + /5) ---- */
#define INFRA0_CLR_REG	0x84
#define INFRA1_SET	0x88
#define INFRA1_CLR	0x8c
#define INFRA1_STA	0x94
#define V2_CLOCK_DIV	0x48

static int i2c5_xfer_v3(uint16_t ext_conf, uint16_t timing, uint16_t ltiming,
			uint8_t slave, uint16_t reg, uint8_t *out)
{
	uint16_t st, fifo;
	int i;

	wr16(I2C5_BASE + IC_SOFTRESET, 1);
	delay_us(2000);
	wr16(I2C5_BASE + V2_IO_CONFIG, 0x01a3);	/* open drain + aed=0x1a */
	wr16(I2C5_BASE + V2_CLOCK_DIV, 0x0404);	/* clock-div = 5 */
	wr16(I2C5_BASE + IC_TIMING, (uint16_t)(timing | 0x0001));
	wr16(I2C5_BASE + V2_LTIMING, ltiming);
	wr16(I2C5_BASE + V2_HS, 0x0000);
	wr16(I2C5_BASE + IC_DELAY_LEN, 2);
	wr16(I2C5_BASE + IC_CONTROL, IC_CTRL_ACKDET | IC_CTRL_CLK_EXT |
					 IC_CTRL_DIR_CHG | IC_CTRL_RS);
	wr16(I2C5_BASE + V2_HW_TIMEOUT, 0xffff);
	wr16(I2C5_BASE + IC_EXT_CONF, ext_conf);
	wr16(I2C5_BASE + IC_SLAVE_ADDR, (uint16_t)(slave << 1));
	wr16(I2C5_BASE + IC_INTR_STAT, 0x01ff);
	wr16(I2C5_BASE + IC_FIFO_ADDR_CLR, 1);
	wr16(I2C5_BASE + IC_INTR_MASK,
	     IC_INTR_BUS_ERR | IC_INTR_TIMEOUT | IC_NACKERR | IC_ACKERR | IC_COMP);
	wr16(I2C5_BASE + IC_TRANSFER_LEN, (uint16_t)(2u | (1u << 8)));
	wr16(I2C5_BASE + IC_TRANSAC_LEN, IC_WRRD);
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg >> 8));
	wr16(I2C5_BASE + IC_DATA_PORT, (uint16_t)(reg & 0xff));
	fifo = rd16(I2C5_BASE + V2_FIFO_STAT);
	wr16(I2C5_BASE + V2_MCU_INTR, 1);
	wr16(I2C5_BASE + IC_START, 1);

	for (i = 0; i < 1200; i++) {
		st = rd16(I2C5_BASE + IC_INTR_STAT);
		if (st & 0x0127)
			break;
		delay_us(100);
	}
	wr16(I2C5_BASE + IC_INTR_MASK, 0);
	con_hex2("USBq", "fifo0=", fifo, "intr=", st);
	if (!(st & IC_COMP))
		return -1;
	if (st & (IC_ACKERR | IC_NACKERR | IC_INTR_BUS_ERR | IC_INTR_TIMEOUT))
		return -2;
	*out = (uint8_t)rd16(I2C5_BASE + IC_DATA_PORT);
	return 0;
}

static void usb_stage21(void)
{
	uint8_t b = 0;
	int rc;

	con_str("USBq", "i2c5: infra_i2c0 gate + apdma + div5");
	con_hex2("USBq", "infra0_sta=", rd32(INFRA_AO_BASE + INFRA0_STA),
		 "infra1_sta=", rd32(INFRA_AO_BASE + INFRA1_STA));
	/* gates are active-low: bit 1 = clock OFF, so clear both */
	wr32(INFRA_AO_BASE + INFRA0_CLR_REG, 1u << 11);	/* infra_i2c0 */
	wr32(INFRA_AO_BASE + INFRA1_CLR, 1u << 18);		/* infra_apdma */
	delay_us(2000);
	con_hex2("USBq", "infra0_sta=", rd32(INFRA_AO_BASE + INFRA0_STA),
		 "infra1_sta=", rd32(INFRA_AO_BASE + INFRA1_STA));
	wr16(PERICFG_BASE + PERI_I2C_MODE_ENABLE, 0x0003);

	con_str("USBq", "read 0x0312 (expect 0x1B)");
	rc = i2c5_xfer_v3(0x8001, 0x073f, 0x01ff, 0x34, 0x0312, &b);
	con_hex2("USBq", "A rc=", (uint32_t)rc, "val=", b);

	b = 0;
	rc = i2c5_xfer_v3(0x1801, 0x0129, 0x0069, 0x34, 0x0312, &b);
	con_hex2("USBq", "B rc=", (uint32_t)rc, "val=", b);
	if (rc == 0)
		con_str("USBq", "I2C to MT6360 WORKS");
}

/* ---- stage 22: bit-bang i2c on GPIO 28 (SCL) / 29 (SDA) ---------------- */
#define GPIO_BASE	0x10005000UL
#define BB_SCL	28u
#define BB_SDA	29u

static inline uintptr_t g_dir(unsigned p)  { return GPIO_BASE + 0x0000 + (p >> 5) * 0x10; }
static inline uintptr_t g_dout(unsigned p) { return GPIO_BASE + 0x0100 + (p >> 5) * 0x10; }
static inline uintptr_t g_din(unsigned p)  { return GPIO_BASE + 0x0200 + (p >> 5) * 0x10; }
static inline uintptr_t g_mode(unsigned p) { return GPIO_BASE + 0x0300 + (p >> 3) * 0x10; }

static void g_out(unsigned p, int v)
{
	uintptr_t r = g_dout(p);
	uint32_t b = 1u << (p & 31), d = rd32(r);
	wr32(r, v ? (d | b) : (d & ~b));
}

static int g_in(unsigned p) { return (int)((rd32(g_din(p)) >> (p & 31)) & 1u); }

static void g_dirset(unsigned p, int out)
{
	uintptr_t r = g_dir(p);
	uint32_t b = 1u << (p & 31), d = rd32(r);
	wr32(r, out ? (d | b) : (d & ~b));
}

static void g_mode_gpio(unsigned p)
{
	uintptr_t r = g_mode(p);
	uint32_t sh = (p & 7) * 4, d = rd32(r);
	wr32(r, d & ~(0xfu << sh));
}

static void bb_d(void) { delay_us(4); }
static void bb_scl(int v) { g_out(BB_SCL, v); }
static void bb_sda(int v) { g_out(BB_SDA, v); }

static void bb_start(void)
{
	g_dirset(BB_SDA, 1);
	bb_sda(1); bb_scl(1); bb_d();
	bb_sda(0); bb_d();
	bb_scl(0); bb_d();
}

static void bb_stop(void)
{
	g_dirset(BB_SDA, 1);
	bb_sda(0); bb_d();
	bb_scl(1); bb_d();
	bb_sda(1); bb_d();
}

static int bb_write(uint8_t b)
{
	int i, ack;

	for (i = 7; i >= 0; i--) {
		bb_sda((b >> i) & 1);
		bb_d();
		bb_scl(1); bb_d();
		bb_scl(0); bb_d();
	}
	g_dirset(BB_SDA, 0);		/* release for ACK */
	bb_scl(1); bb_d();
	ack = !g_in(BB_SDA);
	bb_scl(0); bb_d();
	g_dirset(BB_SDA, 1);
	return ack;
}

static uint8_t bb_read(int nack)
{
	uint8_t v = 0;
	int i;

	g_dirset(BB_SDA, 0);
	for (i = 7; i >= 0; i--) {
		bb_d();
		bb_scl(1); bb_d();
		v = (uint8_t)((v << 1) | (g_in(BB_SDA) & 1));
		bb_scl(0);
	}
	g_dirset(BB_SDA, 1);
	bb_sda(nack ? 1 : 0);
	bb_d();
	bb_scl(1); bb_d();
	bb_scl(0); bb_d();
	bb_sda(1);
	return v;
}

static void usb_stage22(void)
{
	int ack1, ack2, ack3;
	uint8_t v = 0;

	con_str("USBr", "bit-bang i2c on gpio 28/29");
	g_mode_gpio(BB_SCL);
	g_mode_gpio(BB_SDA);
	g_dirset(BB_SCL, 1);
	g_dirset(BB_SDA, 1);
	bb_scl(1); bb_sda(1);
	delay_us(200);
	con_hex2("USBr", "idle_scl=", g_in(BB_SCL), "idle_sda=", g_in(BB_SDA));
	con_hex2("USBr", "mode_reg=", rd32(g_mode(BB_SCL)),
		 "dir_reg=", rd32(g_dir(BB_SCL)));

	/* read MT6360 PMU register 0x0312 */
	bb_start();
	ack1 = bb_write(0x34 << 1);
	ack2 = bb_write(0x03);
	ack3 = bb_write(0x12);
	con_hex2("USBr", "ack_addr=", (uint32_t)ack1, "ack_reg=", (uint32_t)(ack2 & ack3));
	if (ack1) {
		bb_start();
		ack1 = bb_write((0x34 << 1) | 1);
		v = bb_read(1);
		bb_stop();
		con_hex2("USBr", "chg_ctrl2=", v, "ack_rd=", (uint32_t)ack1);
		con_str("USBr", "if chg_ctrl2=1B bit-bang works");
	} else {
		bb_stop();
		con_str("USBr", "no ACK on 0x34: pins/pull-ups");
	}
}

/* ---- stage 23: bit-bang register sweep -------------------------------- */
static int bb_rd16(uint8_t slave, uint16_t reg, uint8_t *out)
{
	int ack;

	bb_start();
	ack = bb_write((uint8_t)(slave << 1));
	if (!ack) { bb_stop(); return -1; }
	ack = bb_write((uint8_t)(reg >> 8));
	ack &= bb_write((uint8_t)(reg & 0xff));
	if (!ack) { bb_stop(); return -2; }
	delay_us(150);			/* let the chip fetch the register */
	bb_start();
	ack = bb_write((uint8_t)((slave << 1) | 1));
	if (!ack) { bb_stop(); return -3; }
	*out = bb_read(1);
	bb_stop();
	return 0;
}

static int bb_rd8(uint8_t slave, uint8_t reg, uint8_t *out)
{
	int ack;

	bb_start();
	ack = bb_write((uint8_t)(slave << 1));
	if (!ack) { bb_stop(); return -1; }
	ack = bb_write(reg);
	if (!ack) { bb_stop(); return -2; }
	delay_us(150);
	bb_start();
	ack = bb_write((uint8_t)((slave << 1) | 1));
	if (!ack) { bb_stop(); return -3; }
	*out = bb_read(1);
	bb_stop();
	return 0;
}

static void bb_pins_init(void)
{
	g_mode_gpio(BB_SCL);
	g_mode_gpio(BB_SDA);
	g_dirset(BB_SCL, 1);
	g_dirset(BB_SDA, 1);
	bb_scl(1); bb_sda(1);
	delay_us(200);
}

static void usb_stage23(void)
{
	static const uint16_t regs[] = { 0x0311, 0x0312, 0x0313, 0x0300, 0x0301, 0x0315 };
	uint8_t v = 0;
	int i, rc;

	con_str("USBs", "bit-bang sweep (slow + 150us gap)");
	bb_pins_init();
	/* slow the clock down to be safe */
	con_hex2("USBs", "idle_scl=", g_in(BB_SCL), "idle_sda=", g_in(BB_SDA));

	for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++) {
		v = 0;
		rc = bb_rd16(0x34, regs[i], &v);
		con_hex2("USBs", "pmu_rd rc=", (uint32_t)rc, "val=", v);
	}
	con_str("USBs", "also read reg 0x0312 twice back-to-back");
	v = 0;
	rc = bb_rd16(0x34, 0x0312, &v);
	con_hex2("USBs", "again rc=", (uint32_t)rc, "val=", v);

	con_str("USBs", "tcpc 0x4e reg 0x00 / 0x01 (8-bit addr)");
	v = 0;
	rc = bb_rd8(0x4e, 0x00, &v);
	con_hex2("USBs", "tcpc00 rc=", (uint32_t)rc, "val=", v);
	v = 0;
	rc = bb_rd8(0x4e, 0x01, &v);
	con_hex2("USBs", "tcpc01 rc=", (uint32_t)rc, "val=", v);
}

/* ---- stage 24: full TCPC dump + PMU protocol variants ------------------ */
static void usb_stage24(void)
{
	uint8_t v1 = 0, v2 = 0;
	int r1, r2;
	unsigned r;

	con_str("USBt", "tcpc 0x4e dump 00..1f");
	bb_pins_init();
	for (r = 0; r < 0x20; r += 2) {
		v1 = 0; v2 = 0;
		r1 = bb_rd8(0x4e, (uint8_t)r, &v1);
		r2 = bb_rd8(0x4e, (uint8_t)(r + 1), &v2);
		con_hex2("USBt", "lo=", (uint32_t)v1, "hi=", (uint32_t)v2);
		if (r1 < 0 || r2 < 0)
			con_hex2("USBt", "rc_err", (uint32_t)r1, "r2=", (uint32_t)r2);
	}

	con_str("USBt", "pmu variants on 0x34");
	v1 = 0;
	r1 = bb_rd16(0x34, 0x0312, &v1);
	con_hex2("USBt", "pmu16_312 rc=", (uint32_t)r1, "val=", v1);
	v1 = 0;
	r1 = bb_rd8(0x34, 0x12, &v1);
	con_hex2("USBt", "pmu8_012 rc=", (uint32_t)r1, "val=", v1);
	v1 = 0;
	r1 = bb_rd16(0x34, 0x0300, &v1);
	con_hex2("USBt", "pmu16_300 rc=", (uint32_t)r1, "val=", v1);
	v1 = 0;
	r1 = bb_rd16(0x1a, 0x0000, &v1);
	con_hex2("USBt", "pmic1a_000 rc=", (uint32_t)r1, "val=", v1);
}

/* ---- stage 25: enable MT6360 OTG boost, then bring up host port ------- */
static void bb_wr8(uint8_t slave, uint8_t reg, uint8_t val)
{
	bb_start();
	bb_write((uint8_t)(slave << 1));
	bb_write(reg);
	bb_write(val);
	bb_stop();
}

static void usb_stage25(void)
{
	uint8_t a = 0, b = 0, c = 0;
	int i;

	con_str("USBu", "pmu 8-bit regs + OTG enable");
	bb_pins_init();
	bb_rd8(0x34, 0x11, &a);
	bb_rd8(0x34, 0x12, &b);
	bb_rd8(0x34, 0x15, &c);
	con_hex2("USBu", "ctrl1_11=", a, "ctrl2_12=", b);
	con_hex2("USBu", "ctrl5_15=", c, "ctrl0_10=", 0);

	con_str("USBu", "write CHG_CTRL1 bit0 (OPA_MODE) = 1");
	bb_wr8(0x34, 0x11, 0x01);
	delay_us(50000);
	a = 0;
	bb_rd8(0x34, 0x11, &a);
	con_hex2("USBu", "ctrl1_after=", a, "x=", 0);

	con_str("USBu", "usb host bring-up");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000a4cu);
	delay_us(20000);

	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);

	for (i = 0; i < 4; i++) {
		delay_us(120000);
		usb_port_line("USBu");
	}
}

/* ---- stage 26: proper MT6360 OTG bring-up ---------------------------- */
static void usb_stage26(void)
{
	uint8_t dt = 0, st = 0, c1 = 0, c5 = 0, c10 = 0, c22 = 0;
	int i;

	con_str("USBv", "otg per mainline: bc1.2 off, oc 3A, vobst 5.0V");
	bb_pins_init();
	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x27, &st);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	con_hex2("USBv", "dev_type22=", dt, "usb_st27=", st);
	con_hex2("USBv", "chg1_11=", c1, "chg5_15=", c5);
	con_hex2("USBv", "chg10_1a=", c10, "x=", 0);

	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));	/* BC1.2 off */
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));	/* OTG OC 3A */
	bb_wr8(0x34, 0x15, 0x5c);			/* VOBST = 5.0 V */
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));	/* OPA_MODE on */
	delay_us(60000);

	c22 = 0; c1 = 0; c5 = 0; st = 0;
	bb_rd8(0x34, 0x22, &c22);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x27, &st);
	con_hex2("USBv", "after dev22=", c22, "chg1=", c1);
	con_hex2("USBv", "after chg5=", c5, "usb_st=", st);

	con_str("USBv", "host bring-up + port poll");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000a4cu);
	delay_us(20000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);

	for (i = 0; i < 4; i++) {
		delay_us(150000);
		usb_port_line("USBv");
	}
}

/* ---- stage 27: TCPC role -> Rp (source) + boost + port ---------------- */
static void usb_stage27(void)
{
	uint8_t dt = 0, st = 0, c1 = 0, c5 = 0, c10 = 0, rc = 0, cmd = 0;
	int i;

	con_str("USBw", "tcpc role -> Rp(source), boost, poll");
	bb_pins_init();
	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));
	delay_us(30000);

	bb_rd8(0x4e, 0x1a, &rc);
	bb_rd8(0x4e, 0x23, &cmd);
	con_hex2("USBw", "role_1a=", rc, "cmd_23=", cmd);

	/* both CC = Rp (source), default Rp; then VBUS_SOURCE_EN */
	bb_wr8(0x4e, 0x1a, 0x05);
	delay_us(20000);
	rc = 0;
	bb_rd8(0x4e, 0x1a, &rc);
	cmd = 0;
	bb_rd8(0x4e, 0x23, &cmd);
	con_hex2("USBw", "role_after=", rc, "cmd=", cmd);
	bb_wr8(0x4e, 0x23, (uint8_t)(cmd | 0x02));
	delay_us(30000);
	cmd = 0;
	bb_rd8(0x4e, 0x23, &cmd);
	st = 0;
	bb_rd8(0x34, 0x27, &st);
	con_hex2("USBw", "cmd_after=", cmd, "usb_st=", st);

	con_str("USBw", "host bring-up + poll");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000a4cu);
	delay_us(20000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	for (i = 0; i < 4; i++) {
		delay_us(150000);
		usb_port_line("USBw");
	}
}

/* ---- stage 28: full TCPC init (mode_ctrl3 0x87) + role + boost ------- */
static void usb_stage28(void)
{
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0;
	uint8_t m3 = 0, role = 0, vc = 0, vb = 0, st = 0;
	int i;

	con_str("USBx", "tcpc init: mode_ctrl3=0x87, vconn, vbus_ctrl2");
	bb_pins_init();

	bb_wr8(0x4e, 0x90, 0x87);	/* PD_BG_EN|IREF_EN|VBUS_DET_EN|OSC_EN */
	bb_wr8(0x4e, 0x8c, 0x41);	/* VCONN_CTRL1 */
	bb_wr8(0x4e, 0xda, 0x0a);	/* VBUS_CTRL2 */
	delay_us(5000);
	bb_rd8(0x4e, 0x90, &m3);
	bb_rd8(0x4e, 0x8c, &vc);
	bb_rd8(0x4e, 0xda, &vb);
	con_hex2("USBx", "m3_90=", m3, "vconn_8c=", vc);
	con_hex2("USBx", "vbus_ctrl2_da=", vb, "x=", 0);

	bb_rd8(0x4e, 0x1a, &role);
	bb_wr8(0x4e, 0x1a, 0x05);	/* Rp source on both CC */
	delay_us(10000);
	m3 = 0;
	bb_rd8(0x4e, 0x1a, &m3);
	con_hex2("USBx", "role_before=", role, "role_after=", m3);

	/* boost */
	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0xff));	/* OTG_OC max */
	bb_wr8(0x34, 0x15, 0x5c);			/* VOBST 5.0 V */
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));	/* OPA_MODE */
	delay_us(50000);
	c5 = 0; c1 = 0; st = 0;
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x27, &st);
	con_hex2("USBx", "chg1=", c1, "chg5=", c5);
	con_hex2("USBx", "usb_st=", st, "x=", 0);

	con_str("USBx", "host bring-up + poll");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000a4cu);
	delay_us(20000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
	for (i = 0; i < 4; i++) {
		delay_us(150000);
		usb_port_line("USBx");
	}
}

/* ---- stage 30: role sweep (Rp + opposite Rd) + poll ------------------- */
static void usb_stage30(void)
{
	static const uint8_t roles[] = { 0x05, 0x09, 0x0d, 0x19, 0x06, 0x0a };
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0, st = 0;
	unsigned i;

	con_str("USBy", "boost on, then role sweep");
	bb_pins_init();
	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));

	/* tcpc: bandgap/iref/vbusdet/osc on */
	bb_wr8(0x4e, 0x90, 0x87);

	/* host side first, so the port is in RxDetect while we sweep roles */
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x00000a4cu);
	delay_us(10000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);

	for (i = 0; i < sizeof(roles); i++) {
		uintptr_t op = DWC3_BASE + caplength();
		uint32_t ps;

		bb_wr8(0x4e, 0x1a, roles[i]);
		delay_us(200000);
		ps = rd32(op + XO_PORTSC);
		con_hex2("USBy", "role_wr=", roles[i], "pls=", (ps >> 5) & 0xfu);
		con_hex2("USBy", "ccs=", ps & PORTSC_CCS,
			 "spd=", (ps >> 10) & 0xfu);
	}
	st = 0;
	bb_rd8(0x34, 0x27, &st);
	con_hex2("USBy", "usb_st27=", st, "x=", 0);
}

/* ---- stage 31: real VBUS measurement (no forced VBUSVALID) ------------ */
static uint32_t otg_sts(void)
{
	return rd32(IPPC_BASE + 0x18);
}

static void usb_stage31(void)
{
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0, st = 0;
	uint32_t a, b, c;

	con_str("USBz", "real vbus via otg_sts bit9 (no force)");
	bb_pins_init();

	/* bring the port up WITHOUT forcing VBUSVALID/IDDIG */
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, 0x0000000cu);	/* host_sel, no force */
	delay_us(30000);

	a = otg_sts();			/* baseline: boost still off */
	con_hex2("USBz", "sts_off=", a, "vbus_bit=", (a >> 9) & 1u);

	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));
	bb_wr8(0x4e, 0x90, 0x87);
	bb_wr8(0x4e, 0x1a, 0x09);	/* role: Rp + opposite Rd */
	delay_us(200000);

	b = otg_sts();
	con_hex2("USBz", "sts_on=", b, "vbus_bit=", (b >> 9) & 1u);
	delay_us(300000);
	c = otg_sts();
	con_hex2("USBz", "sts_on2=", c, "vbus_bit=", (c >> 9) & 1u);

	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x27, &st);
	con_hex2("USBz", "chg1=", c1, "usb_st=", st);
}

/* ---- stage 32: PMU reg diff, boost off vs on ------------------------- */
static void usb_stage32(void)
{
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0;
	uint8_t v[8];
	unsigned r;

	con_str("USBa", "pmu 00..1f: boost OFF");
	bb_pins_init();
	for (r = 0; r < 0x20; r += 2) {
		v[0] = 0; v[1] = 0;
		bb_rd8(0x34, (uint8_t)r, &v[0]);
		bb_rd8(0x34, (uint8_t)(r + 1), &v[1]);
		con_hex2("USBa", "lo=", v[0], "hi=", v[1]);
	}

	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));
	delay_us(250000);

	con_str("USBa", "pmu 00..1f: boost ON");
	for (r = 0; r < 0x20; r += 2) {
		v[0] = 0; v[1] = 0;
		bb_rd8(0x34, (uint8_t)r, &v[0]);
		bb_rd8(0x34, (uint8_t)(r + 1), &v[1]);
		con_hex2("USBa", "lo=", v[0], "hi=", v[1]);
	}
}

/* ---- stage 33: full vendor TCPC init + boost + port ------------------- */
static void usb_host_up(uint32_t u2val)
{
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_phy_on();
	wr32(IPPC_BASE + IPPC_U2_CTRL0, u2val);
	delay_us(20000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_HCRST);
	delay_us(5000);
	wr32(DWC3_BASE + caplength() + XO_USBCMD, USBCMD_RS);
	wr32(DWC3_BASE + caplength() + XO_PORTSC, PORTSC_PP | PORTSC_CSC);
}

static void usb_stage33(void)
{
	static const uint8_t roles[] = { 0x09, 0x05, 0x0d };
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0, st = 0;
	unsigned i;

	con_str("USBc1", "vendor tcpc init + boost + port");
	bb_pins_init();

	/* --- mt6360_tcpc_init(), vendor order --- */
	bb_wr8(0x4e, 0x1a, 0x0a);	/* ROLE_CTRL Rd,Rd */
	bb_wr8(0x4e, 0xa1, 10);		/* DEBOUNCE_CTRL1 */
	bb_wr8(0x4e, 0x87, 0x88);	/* CLK_CTRL1 */
	bb_wr8(0x4e, 0x88, 0x10);	/* CLK_CTRL2 */
	bb_wr8(0x4e, 0xa2, 4);		/* DRP_CTRL1 */
	bb_wr8(0x4e, 0x80, 0x70);	/* PHY_CTRL1 */
	bb_wr8(0x4e, 0x81, 0x3a);	/* PHY_CTRL2 */
	bb_wr8(0x4e, 0x82, 0x82);	/* PHY_CTRL3 */
	bb_wr8(0x4e, 0x86, 0x36);	/* PHY_CTRL7 */
	bb_wr8(0x4e, 0xca, 0x60);	/* PHY_CTRL11 */
	bb_wr8(0x4e, 0xcb, 0x3c);	/* PHY_CTRL12 */
	bb_wr8(0x4e, 0xce, 0xe8);	/* RX_CTRL1 */
	bb_wr8(0x4e, 0x8c, 0x41);	/* VCONN_CTRL1 */
	bb_wr8(0x4e, 0xda, 0x0a);	/* VBUS_CTRL2 */
	bb_wr8(0x4e, 0xcf, 0x80);	/* RX_CTRL2 */
	bb_wr8(0x4e, 0x90, 0x87);	/* MODE_CTRL3 */
	bb_wr8(0x4e, 0x04, 0x10);	/* TCPC_CTRL look4conn */
	bb_wr8(0x4e, 0xec, 0x40);	/* CTD_CTRL2 oneshot */

	/* --- boost --- */
	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));
	delay_us(100000);

	usb_host_up(0x00000004u);
	delay_us(50000);
	con_hex2("USBc1", "host up, port:", rd32(DWC3_BASE + caplength() + XO_PORTSC), "chg1=", 0);

	for (i = 0; i < 3; i++) {
		bb_wr8(0x4e, 0x1a, roles[i]);
		delay_us(200000);
		con_hex2("USBc1", "role=", roles[i], "pls=",
			 (rd32(DWC3_BASE + caplength() + XO_PORTSC) >> 5) & 0xfu);
		con_hex2("USBc1", "ccs=",
			 rd32(DWC3_BASE + caplength() + XO_PORTSC) & 1u,
			 "otg=", rd32(IPPC_BASE + 0x18));
	}
}

/* ---- stage 34: real VBUS voltmeter via MT6360 ADC -------------------- */
static uint32_t pmu_adc_vbus(void)
{
	uint8_t a = 0, b = 0, c = 0;

	bb_wr8(0x34, 0x5a, 0x10);	/* report channel 1 = VBUSDIV5 */
	bb_wr8(0x34, 0x56, 0x80);	/* adc_en */
	bb_wr8(0x34, 0x57, 0x02);	/* enable channel 1 */
	delay_us(80000);
	bb_rd8(0x34, 0x5a, &a);
	bb_rd8(0x34, 0x5b, &b);
	bb_rd8(0x34, 0x5c, &c);
	bb_wr8(0x34, 0x57, 0x00);	/* disable channel */
	return ((uint32_t)b << 8) | c;
}

static void usb_stage34(void)
{
	uint8_t dt = 0, c1 = 0, c5 = 0, c10 = 0;
	uint32_t raw0, raw1, raw2;

	con_str("USBv1", "vbus adc: before boost");
	bb_pins_init();
	raw0 = pmu_adc_vbus();
	con_hex2("USBv1", "raw0=", raw0, "mv0=", raw0 * 25u / 4u);

	bb_rd8(0x34, 0x22, &dt);
	bb_rd8(0x34, 0x11, &c1);
	bb_rd8(0x34, 0x15, &c5);
	bb_rd8(0x34, 0x1a, &c10);
	bb_wr8(0x34, 0x22, (uint8_t)(dt & ~0x80));
	bb_wr8(0x34, 0x1a, (uint8_t)(c10 | 0x0f));
	bb_wr8(0x34, 0x15, 0x5c);
	bb_wr8(0x34, 0x11, (uint8_t)(c1 | 0x01));
	delay_us(150000);

	c1 = 0;
	bb_rd8(0x34, 0x11, &c1);
	raw1 = pmu_adc_vbus();
	con_hex2("USBv1", "raw1=", raw1, "mv1=", raw1 * 25u / 4u);
	con_hex2("USBv1", "chg1=", c1, "raw_prev=", raw0);

	delay_us(300000);
	raw2 = pmu_adc_vbus();
	con_hex2("USBv1", "raw2=", raw2, "mv2=", raw2 * 25u / 4u);
}

/* ---- stage 35: why does the boost never start? ------------------------ */
/* ADC read that cannot return a stale sample: ADC_RPT_1 bits[3:0] carry the
 * channel that actually produced ADC_RPT_2/3, and the vendor driver waits for
 * that nibble to match the requested channel before trusting the result.
 * Stage 34 read the data registers unconditionally, which is where its first
 * sample (0x59B = 8969 mV on a 5 V cable) came from. */
/* Last ADC_RPT_1 seen, for the log: bits[7:4] = channel requested,
 * bits[3:0] = channel that actually produced the data. If they differ, the
 * 16-bit result is a leftover from an earlier conversion and must be ignored. */
static uint8_t adc_rpt1;

static uint32_t pmu_adc_raw(uint8_t ch)
{
	uint8_t a = 0, b = 0, c = 0;
	int i;

	/* Stage 35 measured 0x0BBA on all three channels: re-selecting a channel
	 * that is already enabled does not always start a new conversion. So drop
	 * adc_en, drop the channel, then bring both back up in order - that is a
	 * rising edge on adc_en *and* on the channel bit.
	 * Vendor driver waits on the adc_donei IRQ with a 600 ms timeout; we poll
	 * the report nibble for 1.2 s instead. */
	bb_wr8(0x34, 0x57, 0x00);
	bb_wr8(0x34, 0x56, 0x00);
	delay_us(2000);
	bb_wr8(0x34, 0x5a, (uint8_t)((ch & 0x0fu) << 4));	/* want channel */
	bb_wr8(0x34, 0x56, 0x80);				/* adc_en */
	bb_wr8(0x34, 0x57, (uint8_t)(1u << (ch & 7)));		/* enable it */
	for (i = 0; i < 60; i++) {
		delay_us(20000);
		a = 0;
		bb_rd8(0x34, 0x5a, &a);
		if ((a & 0x0fu) == (ch & 0x0fu))
			break;
	}
	adc_rpt1 = a;
	bb_rd8(0x34, 0x5b, &b);
	bb_rd8(0x34, 0x5c, &c);
	bb_wr8(0x34, 0x57, 0x00);
	return ((uint32_t)b << 8) | c;
}

static uint32_t pmu_vbus_mv(void)
{
	return pmu_adc_raw(1) * 25u / 4u;		/* VBUSDIV5: 6.25 mV/LSB */
}

/* OPA_MODE really wants a 0 -> 1 transition; OR-ing into a bit that is already
 * set (it survives SoC resets in the PMIC) gives the FSM no edge at all. */
static void otg_edge(uint8_t c11, uint32_t gap_us)
{
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(gap_us);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(300000);
}

static void usb_stage35(void)
{
	uint8_t c11 = 0, c15 = 0, c1a = 0, c1d = 0, c61 = 0;
	uint8_t s27 = 0, s4a = 0, e0 = 0, e1 = 0;
	uint32_t r, mv;

	con_str("USBv2", "boost: vbat/vsys + edge + ovp tests");
	bb_pins_init();

	r = pmu_adc_raw(4);				/* VBAT, 1.25 mV/LSB */
	con_hex2("USBv2", "vbat=", r * 5u / 4u, "raw=", r);
	r = pmu_adc_raw(3);				/* VSYS, 1.25 mV/LSB */
	con_hex2("USBv2", "vsys=", r * 5u / 4u, "raw=", r);
	r = pmu_adc_raw(1);				/* VBUSDIV5, 6.25 mV/LSB */
	con_hex2("USBv2", "vbus0=", r * 25u / 4u, "raw=", r);

	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x15, &c15);
	bb_rd8(0x34, 0x1a, &c1a);
	bb_rd8(0x34, 0x1d, &c1d);
	bb_rd8(0x34, 0x61, &c61);
	con_hex2("USBv2", "c11=", c11, "c15=", c15);
	con_hex2("USBv2", "c1a=", c1a, "c1d=", c1d);
	bb_rd8(0x34, 0x27, &s27);
	bb_rd8(0x34, 0x4a, &s4a);
	bb_rd8(0x34, 0xe0, &e0);
	bb_rd8(0x34, 0xe1, &e1);
	con_hex2("USBv2", "usb27=", s27, "chg4a=", s4a);
	con_hex2("USBv2", "e0=", e0, "e1=", e1);

	/* C1: charger watchdog first (mt6360_enable_otg() does exactly this),
	 * VOBST = 5.00 V, then a clean rising edge on OPA_MODE. */
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80));	/* CHG_WDT_EN */
	bb_wr8(0x34, 0x15, 0x5c);			/* VOBST 4425+23*25 */
	otg_edge(c11, 300000);
	c11 = 0;
	bb_rd8(0x34, 0x11, &c11);
	mv = pmu_adc_raw(1) * 25u / 4u;
	con_hex2("USBv2", "c1 vbus=", mv, "c11=", c11);

	/* C2: VIN_OVP 5.5 V -> 14.5 V (vendor init value), then another edge.
	 * If the boost output trips the input OVP comparator at 5 V, this is it. */
	bb_wr8(0x34, 0x61, (uint8_t)((c61 & ~0x60u) | 0x60));
	otg_edge(c11, 200000);
	mv = pmu_adc_raw(1) * 25u / 4u;
	con_hex2("USBv2", "c2 vbus=", mv, "c61=", (c61 & ~0x60u) | 0x60);

	/* C3: OTG current limit 3 A, touching only bits[2:0] (stage 33/34 wrote
	 * 0x0F into CHG_CTRL10, which also pokes bit3 of an adjacent field). */
	bb_wr8(0x34, 0x1a, (uint8_t)((c1a & ~0x07u) | 0x07u));
	otg_edge(c11, 200000);
	mv = pmu_adc_raw(1) * 25u / 4u;
	con_hex2("USBv2", "c3 vbus=", mv, "c1a=", (c1a & ~0x07u) | 0x07u);

	/* C4: read the hidden charger block (pass code 69 96 63 72 on 0x07..0x0A). */
	bb_wr8(0x34, 0x07, 0x69);
	bb_wr8(0x34, 0x08, 0x96);
	bb_wr8(0x34, 0x09, 0x63);
	bb_wr8(0x34, 0x0a, 0x72);
	delay_us(2000);
	{
		uint8_t h[8];
		unsigned i;

		for (i = 0; i < 8; i++) {
			h[i] = 0;
			bb_rd8(0x34, (uint8_t)(0x30 + i), &h[i]);
		}
		con_hex2("USBv2", "h0x30=", h[0], "h31=", h[1]);
		con_hex2("USBv2", "h32=", h[2], "h33=", h[3]);
		con_hex2("USBv2", "h34=", h[4], "h35=", h[5]);
		con_hex2("USBv2", "h36=", h[6], "h37=", h[7]);
	}
	bb_wr8(0x34, 0x07, 0x00);

	otg_edge(c11, 200000);
	mv = pmu_adc_raw(1) * 25u / 4u;
	c11 = 0;
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x4a, &s4a);
	con_hex2("USBv2", "fin vbus=", mv, "chg4a=", s4a);
}

/* ---- stage 36: A/B/A proof of VBUS, then Rp + host + enumeration ------ */
/* The only way to tell a live rail from a stale ADC register is to make the
 * rail move: boost off -> measure -> boost on -> measure -> off -> measure.
 * A rail that follows CHG_CTRL1 is real. PORTSC.CCS is the second, ADC-free
 * witness: a powered flash drive makes the PHY report connect. */
static void usb_stage36(void)
{
	uint8_t c11 = 0, c15 = 0, c1d = 0, rl = 0;
	uint32_t a_mv, b_mv, c_mv, ps = 0;
	int i;

	con_str("USBv3", "A/B/A vbus, then Rp + host + enum");
	bb_pins_init();

	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x15, &c15);
	bb_rd8(0x34, 0x1d, &c1d);
	con_hex2("USBv3", "c11=", c11, "c15=", c15);
	{
		uint8_t g56 = 0, g57 = 0, g58 = 0, g5a = 0;

		bb_rd8(0x34, 0x56, &g56);
		bb_rd8(0x34, 0x57, &g57);
		bb_rd8(0x34, 0x58, &g58);
		bb_rd8(0x34, 0x5a, &g5a);
		con_hex2("USBv3", "adc56=", g56, "adc5a=", g5a);
		con_hex2("USBv3", "adc57=", g57, "adc58=", g58);
	}

	/* A: boost off */
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(500000);
	a_mv = pmu_vbus_mv();
	con_hex2("USBv3", "A off mv=", a_mv, "rpt=", adc_rpt1);

	/* B: watchdog + VOBST 5.00 V + clean 0 -> 1 edge */
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	otg_edge((uint8_t)(c11 & ~0x01u), 300000);
	bb_rd8(0x34, 0x11, &rl);
	b_mv = pmu_vbus_mv();
	con_hex2("USBv3", "B on  mv=", b_mv, "c11=", rl);

	/* C: off again, the rail must fall back */
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(500000);
	c_mv = pmu_vbus_mv();
	con_hex2("USBv3", "C off mv=", c_mv, "rpt=", adc_rpt1);

	if (b_mv >= 4000u && c_mv < 1500u)
		con_str("USBv3", "VBUS follows OPA_MODE: boost is real");
	else if (b_mv >= 4000u)
		con_str("USBv3", "high but no fall: slow bleed or stale");
	else
		con_str("USBv3", "boost still dead, checking port anyway");

	/* Leave the boost on and present Rp so a Type-C drive powers up. */
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	bb_wr8(0x4e, 0x1a, 0x05);		/* ROLE_CONTROL: Rp default, CC1+CC2 */
	delay_us(200000);
	rl = 0;
	bb_rd8(0x4e, 0x1a, &rl);
	con_hex2("USBv3", "role_1a=", rl, "vbus=", pmu_vbus_mv());

	/* Host port up, then watch the ADC-free witness: PORTSC.CCS. */
	usb_host_up(0x00000004u);
	for (i = 0; i < 40; i++) {
		ps = rd32(DWC3_BASE + caplength() + XO_PORTSC);
		if (ps & 1u)
			break;
		delay_us(50000);
	}
	con_hex2("USBv3", "portsc=", ps, "ccs=", ps & 1u);
	con_hex2("USBv3", "pls=", (ps >> 5) & 0xfu, "spd=", (ps >> 10) & 0xfu);

	if (ps & 1u)
		usb_stage14();
	else
		con_str("USBv3", "no ccs: drive unpowered or absent");
}

/* ---- stage 37: is anything actually attached to D+/D-? ---------------- */
/* VBUS is real now, yet PORTSC stays in RxDetect with a powered drive on the
 * connector. Three independent witnesses tell the three possible faults apart:
 *   U2PHYDMON1  : analog VBUSVALID/SESSEND, D+/D- LINE_STATE, HST_DISCON
 *   TCPC 0x11   : CC1/CC2 state (00 open, 01 Ra, 10 Rd, 11 Rp)
 *   IPPC 0x24   : how many U2/U3 ports the hardware actually declares
 * plus PORTSC of ports 2 and 3, in case the connector is not port 1. */
static void usb_stage37(void)
{
	uint8_t c11 = 0, c1d = 0, cc = 0, rl = 0;
	uint32_t dmon = 0, dmon2 = 0, ps = 0, otg = 0, cap = 0, u2 = 0, u3 = 0;
	int i;

	con_str("USBv4", "line-state / cc / port-map probe");
	bb_pins_init();
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x1d, &c1d);

	/* boost on, the sequence stage 36 proved (A=0, B=4987, C=0) */
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(150000);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(250000);
	bb_rd8(0x34, 0x11, &c11);
	con_hex2("USBv4", "vbus=", pmu_vbus_mv(), "c11=", c11);

	/* TCPC: what does the chip see on CC, before and after we become Rp? */
	bb_rd8(0x4e, 0x11, &cc);
	bb_rd8(0x4e, 0x1a, &rl);
	con_hex2("USBv4", "cc_bef=", cc, "role0=", rl);
	bb_wr8(0x4e, 0x1a, 0x05);		/* Rp default, CC1+CC2 */
	bb_wr8(0x4e, 0x04, 0x10);		/* TCPC_CTRL: look4connection */
	delay_us(300000);
	bb_rd8(0x4e, 0x11, &cc);
	bb_rd8(0x4e, 0x1a, &rl);
	con_hex2("USBv4", "cc_aft=", cc, "role1=", rl);

	/* IPPC: OTG status (bit9 VBUS_VALID), declared port counts, port ctrl */
	otg = rd32(IPPC_BASE + 0x18);
	cap = rd32(IPPC_BASE + 0x24);
	u2 = rd32(IPPC_BASE + 0x50);
	u3 = rd32(IPPC_BASE + 0x30);
	con_hex2("USBv4", "otg18=", otg, "xhci_cap=", cap);
	con_hex2("USBv4", "u2_50=", u2, "u3_30=", u3);

	/* host port up (PHY + IPPC + PP), then watch for a connect */
	usb_host_up(0x00000004u);
	for (i = 0; i < 40; i++) {
		ps = rd32(DWC3_BASE + caplength() + XO_PORTSC);
		if (ps & 1u)
			break;
		delay_us(50000);
	}
	dmon = rd32(U3PHY_BASE + 0x300 + 0x74);	/* U2PHYDMON1 */
	con_hex2("USBv4", "dmon1=", dmon, "portsc=", ps);
	con_hex2("USBv4", "psc2=", rd32(DWC3_BASE + caplength() + XO_PORTSC + 0x10),
		 "psc3=", rd32(DWC3_BASE + caplength() + XO_PORTSC + 0x20));

	/* Second attempt with the UTMI session signals forced valid from the RG
	 * bits: FORCE_VBUSVALID|FORCE_AVALID|FORCE_SESSEND, RG values 1/1/0. */
	wr32(U3PHY_BASE + 0x300 + 0x6c,
	     (1u << 13) | (1u << 12) | (1u << 10) | (1u << 5) | (1u << 2));
	delay_us(300000);
	ps = rd32(DWC3_BASE + caplength() + XO_PORTSC);
	dmon2 = rd32(U3PHY_BASE + 0x300 + 0x74);
	con_hex2("USBv4", "dmon1b=", dmon2, "portsc2=", ps);

	if (ps & 1u) {
		con_str("USBv4", "device seen: enumerating");
		usb_stage14();
	} else {
		con_str("USBv4", "still no ccs (see dmon1 line bits)");
	}
}

/* ---- stage 38: boost + forced session + port reset + enumeration ------ */
/* Stage 37 found the missing link: the U2 PHY's analog session signals are
 * dead on this board (dmon1 bit26 VBUSVALID=0, bit25 SESSEND=1 while the
 * connector really has 4.99 V), so the controller sat in RxDetect forever.
 * Forcing VBUSVALID/AVALID/SESSEND from the RG bits moved PLS 5 -> 7
 * (Polling) and CCS to 1 - the drive was there all along. */
static void usb_stage38(void)
{
	uint8_t c11 = 0, c1d = 0, rl = 0;

	con_str("USBv5", "force session, reset port, enumerate");
	bb_pins_init();
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x1d, &c1d);

	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(150000);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(250000);
	bb_rd8(0x34, 0x11, &c11);
	con_hex2("USBv5", "vbus=", pmu_vbus_mv(), "c11=", c11);

	bb_wr8(0x4e, 0x1a, 0x05);		/* Rp default on both CC */
	bb_wr8(0x4e, 0x04, 0x10);		/* look4connection */
	bb_rd8(0x4e, 0x1a, &rl);
	con_hex2("USBv5", "role=", rl, "x=", 0);

	/* Force the UTMI session signals: FORCE_VBUSVALID | FORCE_SESSEND |
	 * FORCE_AVALID, with RG_VBUSVALID=1, RG_AVALID=1, RG_SESSEND=0. */
	wr32(PHY_U2PHYDTM1,
	     (1u << 13) | (1u << 12) | (1u << 10) | (1u << 5) | (1u << 2));
	delay_us(100000);

	usb_stage14();
}

/* ---- stage 39: is the command ring actually programmed? ---------------- */
/* Enable Slot still answers "TRB Error" after the offset fix, so the next
 * question is whether the controller took CRCR/DCBAAP at all and what the
 * event really says. This stage prints the capability registers, reads the
 * ring registers back, and dumps all four dwords of the completion event. */
static void usb_stage39(void)
{
	uint8_t c11 = 0, c1d = 0;
	uint32_t op, ps = 0, ev[4];
	uint64_t t0;
	int i;

	con_str("USBv6", "ring readback + raw event dump");
	bb_pins_init();
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x1d, &c1d);
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(150000);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(250000);
	bb_wr8(0x4e, 0x1a, 0x05);
	bb_wr8(0x4e, 0x04, 0x10);
	wr32(PHY_U2PHYDTM1,
	     (1u << 13) | (1u << 12) | (1u << 10) | (1u << 5) | (1u << 2));
	delay_us(50000);

	/* ports/gates/PHY, then stop the controller again to program the ring */
	usb_host_up(0x00000004u);
	op = DWC3_BASE + caplength();
	con_hex2("USBv6", "op=", (uint32_t)op, "hcsp1=", rd32(DWC3_BASE + 0x04));
	con_hex2("USBv6", "hccp1=", rd32(DWC3_BASE + 0x10),
		 "db_rt=", rd32(DWC3_BASE + 0x14) | (rd32(DWC3_BASE + 0x18) << 16));

	wr32(op + XO_USBCMD, 2u);		/* HCRST */
	delay_us(50000);
	con_hex2("USBv6", "cmd=", rd32(op + XO_USBCMD),
		 "sts=", rd32(op + XO_USBSTS));

	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	xhci_zero(x_dcbaa, 64);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_cmd_cyc = 1;
	x_cmd_i = 0;
	x_evt_cyc = 1;
	x_evt_i = 0;
	wr32(op + XO_CONFIG, 1u);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);
	wr32(op + XO_USBCMD, 1u);		/* RS */
	delay_us(30000);

	con_hex2("USBv6", "crcr=", rd32(op + XO_CRCR), "want=",
		 (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u);
	con_hex2("USBv6", "crcr_hi=", rd32(op + XO_CRCR + 4),
		 "dcbaap=", rd32(op + XO_DCBAAP));
	con_hex2("USBv6", "cmd2=", rd32(op + XO_USBCMD),
		 "sts2=", rd32(op + XO_USBSTS));
	con_hex2("USBv6", "config=", rd32(op + XO_CONFIG),
		 "porte=", rd32(op + XO_PORTSC));

	/* one raw Enable Slot TRB, then dump the event fields verbatim */
	x_cmd[0][0] = 0;
	x_cmd[0][1] = 0;
	x_cmd[0][2] = 0;
	x_cmd[0][3] = (9u << 10) | 1u;
	__asm__ volatile("dsb sy" ::: "memory");
	wr32(x_db, 0);
	for (i = 0; i < 4; i++)
		ev[i] = 0xDEADBEEFu;
	t0 = cnt();
	for (i = 0; i < 30; i++) {
		if (!xhci_evt(ev, 100))
			break;
		if ((cnt() - t0) > 400ull * 13000u)
			break;
	}
	con_hex2("USBv6", "ev0=", ev[0], "ev1=", ev[1]);
	con_hex2("USBv6", "ev2=", ev[2], "ev3=", ev[3]);
	con_hex2("USBv6", "code=", ev[2] >> 24, "type=", (ev[3] >> 10) & 0x3fu);
	con_hex2("USBv6", "slot=", ev[3] & 0xffu, "dbg=", rd32(x_db));
}

/* ---- stage 40: which op register accepts our writes, and where? ------- */
/* Stage 39: crcr/dcbaap/config all read back as 0, yet the completion event's
 * "command TRB pointer" was 0x400AB4C0 - exactly the address of x_dcbaa, i.e.
 * the value we wrote as DCBAAP became the command-ring pointer. So either the
 * registers are shifted or reads are broken. This stage writes distinct
 * patterns and reads every candidate offset back. */
static void usb_stage40(void)
{
	uint32_t op;

	con_str("USBv7", "op register write/read map");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	delay_us(20000);
	op = DWC3_BASE + caplength();
	con_hex2("USBv7", "op=", (uint32_t)op, "caplen=", caplength());

	wr32(op + XO_USBCMD, 2u);		/* stop so config regs are writable */
	delay_us(50000);
	con_hex2("USBv7", "cmd0=", rd32(op + XO_USBCMD), "sts0=", rd32(op + XO_USBSTS));

	/* A: distinct pattern per register, then dump the whole op block */
	wr32(op + 0x10, 0x11111111u);
	wr32(op + 0x18, 0x22222222u);
	wr32(op + 0x20, 0x33333333u);
	delay_us(1000);
	con_hex2("USBv7", "00=", rd32(op + 0x00), "04=", rd32(op + 0x04));
	con_hex2("USBv7", "08=", rd32(op + 0x08), "0c=", rd32(op + 0x0c));
	con_hex2("USBv7", "10=", rd32(op + 0x10), "14=", rd32(op + 0x14));
	con_hex2("USBv7", "18=", rd32(op + 0x18), "1c=", rd32(op + 0x1c));
	con_hex2("USBv7", "20=", rd32(op + 0x20), "24=", rd32(op + 0x24));
	con_hex2("USBv7", "28=", rd32(op + 0x28), "2c=", rd32(op + 0x2c));
	con_hex2("USBv7", "30=", rd32(op + 0x30), "34=", rd32(op + 0x34));
	con_hex2("USBv7", "38=", rd32(op + 0x38), "3c=", rd32(op + 0x3c));

	/* B: program the spec layout, read back before and after starting */
	xhci_zero(x_dcbaa, 64);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	delay_us(1000);
	con_hex2("USBv7", "crcrP=", rd32(op + XO_CRCR),
		 "want=", (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u);
	wr32(op + XO_CONFIG, 1u);
	wr32(op + XO_USBCMD, 1u);		/* RS */
	delay_us(20000);
	con_hex2("USBv7", "crcrQ=", rd32(op + XO_CRCR),
		 "dcbap=", rd32(op + XO_DCBAAP));
	con_hex2("USBv7", "conf=", rd32(op + XO_CONFIG), "sts=", rd32(op + XO_USBSTS));
}

/* ---- stage 41: CNR, the ssusb26m gate, and a proper command ring ----- */
/* Stage 40 decoded USBSTS = 0x0B01: bit11 CNR (Controller Not Ready) is set.
 * While CNR=1 the controller must ignore writes to CRCR/DCBAAP/CONFIG - which
 * is exactly what we saw (all three read back as 0, patterns dropped).
 *
 * CNR stays set when the core's own clock is missing. On MT6785 the SSUSB
 * 26 MHz gate lives in APMIXED 0x20 bit4 and is inverted (bit=1 -> clock off),
 * so enabling it means writing 1<<4 into the clear register. That write is
 * also the one a previous session made blind, and it unclocked the IPPC block.
 * This stage does it explicitly, then waits for CNR and reads CRCR back
 * BEFORE starting the controller. */
static void usb_stage41(void)
{
	uint32_t op, v, sts = 0, slot = 0;
	int i, cnr_wait = -1;

	con_str("USBv8", "ssusb26m gate + CNR wait + ring");
	v = rd32(APMIXED_BASE + APMIXED_GATE);
	con_hex2("USBv8", "ample_pre=", v, "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));
	wr32(APMIXED_BASE + APMIXED_GATE, 0x10u);	/* bit4: 1 = clock off */
	delay_us(2000);
	con_hex2("USBv8", "ample_aft=", rd32(APMIXED_BASE + APMIXED_GATE),
		 "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));

	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	delay_us(20000);

	op = DWC3_BASE + caplength();
	wr32(op + XO_USBCMD, 2u);		/* HCRST */
	delay_us(20000);
	/* wait for CNR (bit11) to clear, like xhci_reset() does */
	for (i = 0; i < 200; i++) {
		sts = rd32(op + XO_USBSTS);
		if (!(sts & (1u << 11))) {
			cnr_wait = i;
			break;
		}
		delay_us(5000);
	}
	con_hex2("USBv8", "cnr_wait=", (uint32_t)cnr_wait, "sts=", sts);

	/* rings with a Link TRB at the end, as xHCI requires */
	xhci_zero(x_dcbaa, 64);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_cmd_cyc = 1;
	x_cmd_i = 0;
	x_evt_cyc = 1;
	x_evt_i = 0;
	x_cmd[RING_TRBS - 1][0] = (uint32_t)(uintptr_t)&x_cmd[0][0];
	x_cmd[RING_TRBS - 1][1] = 0;
	x_cmd[RING_TRBS - 1][2] = 0;
	x_cmd[RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;	/* Link, TC */
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	wr32(op + XO_CONFIG, 1u);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);
	delay_us(1000);
	con_hex2("USBv8", "crcrP=", rd32(op + XO_CRCR),
		 "want=", (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u);
	con_hex2("USBv8", "dcbap=", rd32(op + XO_DCBAAP),
		 "conf=", rd32(op + XO_CONFIG));

	wr32(op + XO_USBCMD, 1u);		/* RS */
	delay_us(30000);
	con_hex2("USBv8", "sts2=", rd32(op + XO_USBSTS),
		 "cmd2=", rd32(op + XO_USBCMD));
	v = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &slot);
	con_hex2("USBv8", "enslot=", v, "slot=", slot);
}

/* ---- stage 42: READ-ONLY snapshot (no writes at all) ------------------ */
/* Stage 41 wrote APMIXED 0x20 (the 26 MHz gate tree - the vendor clock driver
 * itself guards that register with "race condition access") and the phone's
 * display died. From here on nothing in this probe writes anything except the
 * framebuffer: only reads, so a wrong guess cannot hang the SoC. */
static void usb_stage42(void)
{
	uint32_t op = DWC3_BASE + caplength();

	con_str("USBv9", "read-only: clocks, gsnpsid, CNR");
	con_hex2("USBv9", "apmixed20=", rd32(APMIXED_BASE + APMIXED_GATE),
		 "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));
	con_hex2("USBv9", "infr2_sta=", rd32(INFRA_AO_BASE + INFRA2_STA),
		 "ckcfg8=", rd32(TOPCKGEN_BASE + TOP_CK_CFG_8));
	con_hex2("USBv9", "pw_ctl0=", rd32(IPPC_BASE + IPPC_PW_CTRL0),
		 "pw_sts1=", rd32(IPPC_BASE + IPPC_PW_STS1));
	con_hex2("USBv9", "hcsp1=", rd32(DWC3_BASE + 0x04),
		 "hccp1=", rd32(DWC3_BASE + 0x10));
	con_hex2("USBv9", "usbcmd=", rd32(op + XO_USBCMD),
		 "usbsts=", rd32(op + XO_USBSTS));
	con_hex2("USBv9", "crcr=", rd32(op + XO_CRCR),
		 "dcbap=", rd32(op + XO_DCBAAP));
	con_hex2("USBv9", "porte=", rd32(op + XO_PORTSC),
		 "dmon1=", rd32(U3PHY_BASE + 0x300 + 0x74));
	con_str("USBv9", "read-only done, nothing written");
}

/* ---- stage 43: enable SSUSB26M with a single-bit RMW, then enum -------- */
/* Stage 42 (read-only) showed APMIXED 0x20 (AP_PLL_CON8) with bit4 set, i.e.
 * the SSUSB 26 MHz gate is OFF - that is why GSNPSID is 0 and CNR never clears.
 *
 * Stage 41 tried to fix it by WRITING 0x10 as a value. That wiped bit8
 * (MMSYS_F26M) as well: the display lost its 26 MHz clock, went blue/green and
 * the SoC shut down. The correct operation is read-modify-write of bit4 ONLY. */
static void usb_stage43(void)
{
	uint32_t v;
	uint8_t c11 = 0, c1d = 0;

	con_str("USBva", "AP_PLL_CON8 bit4 only, then enum");
	v = rd32(APMIXED_BASE + APMIXED_GATE);
	con_hex2("USBva", "con8_pre=", v, "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));
	wr32(APMIXED_BASE + APMIXED_GATE, v & ~0x10u);	/* bit4 only */
	delay_us(3000);
	con_hex2("USBva", "con8_aft=", rd32(APMIXED_BASE + APMIXED_GATE),
		 "gsnpsid=", rd32(DWC3_BASE + D_GSNPSID));

	/* boost (proven A/B/A) + Rp + forced UTMI session (proven in stage 37) */
	bb_pins_init();
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x1d, &c1d);
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));
	bb_wr8(0x34, 0x15, 0x5cu);
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(150000);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(250000);
	bb_wr8(0x4e, 0x1a, 0x05);
	bb_wr8(0x4e, 0x04, 0x10);
	wr32(PHY_U2PHYDTM1,
	     (1u << 13) | (1u << 12) | (1u << 10) | (1u << 5) | (1u << 2));
	delay_us(100000);
	con_hex2("USBva", "vbus=", pmu_vbus_mv(),
		 "gsnps=", rd32(DWC3_BASE + D_GSNPSID));

	usb_stage14();
}

/* ---- stage 44: the DTS says this is MTK's own MAC, not Synopsys DWC3 --- */
/* mtu3_0@11200000 declares reg "mac" = 0x11201000 (and xhci is
 * "mediatek,mt67xx-xhci" at 0x11200000). So 0x1120c120 (GSNPSID) is a
 * Synopsys address that does not exist here - gsnpsid=0 is NOT a fault and the
 * whole "core not clocked" theory is dead.
 *
 * This stage measures what actually matters: whether CNR ever clears, whether
 * CRCR takes our value before start, and whether the MTK MAC block is alive.
 * No APMIXED writes: nothing here can hang the SoC. */
static void usb_stage44(void)
{
	uint32_t op, sts = 0, slot = 0, v;
	int i, cnr = -1;

	con_str("USBvb", "CNR / crcr / MAC block / slot type");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	delay_us(20000);

	op = DWC3_BASE + caplength();
	con_hex2("USBvb", "caplen=", caplength(), "hcsp1=", rd32(DWC3_BASE + 0x04));
	wr32(op + XO_USBCMD, 2u);		/* HCRST */
	delay_us(20000);
	for (i = 0; i < 200; i++) {
		sts = rd32(op + XO_USBSTS);
		if (!(sts & (1u << 11))) {
			cnr = i;
			break;
		}
		delay_us(5000);
	}
	con_hex2("USBvb", "cnr_wait=", (uint32_t)cnr, "sts=", sts);

	/* MTK MAC block at 0x11201000: LV1 interrupt status, cap regs, MISC */
	con_hex2("USBvb", "mac_lv1=", rd32(0x11201000u),
		 "mac_misc=", rd32(0x11201C84u));
	con_hex2("USBvb", "mac_cap=", rd32(0x11201C04u),
		 "pwsts1=", rd32(IPPC_BASE + IPPC_PW_STS1));

	/* ring: write, read back BEFORE starting */
	xhci_zero(x_dcbaa, 64);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_cmd_cyc = 1;
	x_cmd_i = 0;
	x_evt_cyc = 1;
	x_evt_i = 0;
	x_cmd[RING_TRBS - 1][0] = (uint32_t)(uintptr_t)&x_cmd[0][0];
	x_cmd[RING_TRBS - 1][1] = 0;
	x_cmd[RING_TRBS - 1][2] = 0;
	x_cmd[RING_TRBS - 1][3] = (6u << 10) | (1u << 1) | 1u;	/* Link, TC */
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	wr32(op + XO_CONFIG, 1u);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);
	delay_us(1000);
	con_hex2("USBvb", "crcrP=", rd32(op + XO_CRCR),
		 "want=", (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u);
	con_hex2("USBvb", "dcbap=", rd32(op + XO_DCBAAP),
		 "conf=", rd32(op + XO_CONFIG));

	wr32(op + XO_USBCMD, 1u);		/* RS */
	delay_us(30000);
	con_hex2("USBvb", "sts2=", rd32(op + XO_USBSTS),
		 "cmd2=", rd32(op + XO_USBCMD));
	v = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &slot);
	con_hex2("USBvb", "es_t0=", v, "slot0=", slot);
	v = (uint32_t)xhci_cmd(0, 0, 1, 9u, 0, &slot);
	con_hex2("USBvb", "es_t1=", v, "slot1=", slot);
}

/* ---- stage 45: power the MAC (PW_CTRL2) and watch CNR ------------------ */
/* Stage 44: CNR never cleared, the MTK MAC at 0x11201000 read all zeros,
 * while IPPC PW_STS1 = 0x3D0F says clocks are stable and XHCI reset is
 * released. Stage 39 (which powered the PHY first) saw CNR=0 once, so CNR
 * follows the PHY/MAC side. The MAC is the device half of the same IP and its
 * power is IPPC PW_CTRL2 bit0 (SSUSB_IP_DEV_PDN), which nothing ever cleared.
 * A/B here: PHY only, then PHY + MAC powered. */
static void usb_stage45(void)
{
	uint32_t op, sts = 0, slot = 0, v;
	int i, cnr = -1;

	con_str("USBvc", "pw_ctrl2 + phy: wake the MAC?");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	con_hex2("USBvc", "pw1=", rd32(IPPC_BASE + IPPC_PW_CTRL1),
		 "pw2_pre=", rd32(IPPC_BASE + IPPC_PW_CTRL2));

	/* PHY up (usb_host_up also does HCRST/RS/PP, harmless here) */
	usb_host_up(0x00000004u);
	delay_us(20000);
	con_hex2("USBvc", "mac_lv1=", rd32(0x11201000u),
		 "mac_misc=", rd32(0x11201C84u));

	op = DWC3_BASE + caplength();
	wr32(op + XO_USBCMD, 2u);		/* A: HCRST */
	delay_us(20000);
	for (i = 0; i < 200; i++) {
		sts = rd32(op + XO_USBSTS);
		if (!(sts & (1u << 11))) {
			cnr = i;
			break;
		}
		delay_us(5000);
	}
	con_hex2("USBvc", "A cnr=", (uint32_t)cnr, "sts=", sts);

	/* B: also power the device/MAC side of the same IP */
	v = rd32(IPPC_BASE + IPPC_PW_CTRL2);
	wr32(IPPC_BASE + IPPC_PW_CTRL2, v & ~1u);	/* clear DEV_PDN */
	delay_us(20000);
	con_hex2("USBvc", "pw2_aft=", rd32(IPPC_BASE + IPPC_PW_CTRL2),
		 "mac2=", rd32(0x11201000u));
	wr32(op + XO_USBCMD, 2u);		/* B: HCRST again */
	delay_us(20000);
	cnr = -1;
	for (i = 0; i < 200; i++) {
		sts = rd32(op + XO_USBSTS);
		if (!(sts & (1u << 11))) {
			cnr = i;
			break;
		}
		delay_us(5000);
	}
	con_hex2("USBvc", "B cnr=", (uint32_t)cnr, "sts=", sts);

	/* try the ring once more, read back before starting */
	xhci_zero(x_dcbaa, 64);
	xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
	xhci_zero(&x_evt[0][0], 32 * 4);
	x_cmd_cyc = 1;
	x_cmd_i = 0;
	x_evt_cyc = 1;
	x_evt_i = 0;
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);
	wr32(op + XO_CONFIG, 1u);
	xhci_wr64(op + XO_DCBAAP, (uint32_t)(uintptr_t)&x_dcbaa[0], 0);
	xhci_wr64(op + XO_CRCR, (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
	x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
	x_erst[1] = 0;
	x_erst[2] = 32;
	x_erst[3] = 0;
	wr32(x_rt + 0x28, 1u);
	xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
	xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
	wr32(x_rt + 0x20, 2u);
	delay_us(1000);
	con_hex2("USBvc", "crcr=", rd32(op + XO_CRCR),
		 "want=", (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u);
	wr32(op + XO_USBCMD, 1u);		/* RS */
	delay_us(30000);
	v = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &slot);
	con_hex2("USBvc", "enslot=", v, "slot=", slot);
}

/* ---- stage 46: CRCR at 0x10 or 0x18? Enable Slot decides -------------- */
/* Stage 45: with the PHY up CNR clears (so CNR follows the PHY), yet CRCR
 * still reads 0 and the command fails with code 5. In stage 39 the failure
 * event's "command TRB pointer" was 0x400AB4C0 - exactly the address we had
 * written into DCBAAP. That is consistent with one specific hypothesis:
 *
 *   DCBAAP and CRCR are swapped in this wrapper (write to 0x18 lands in CRCR,
 *   write to 0x10 lands in DCBAAP), so our DCBAAP value became the command
 *   ring pointer, the controller fetched a zeroed TRB from it and answered
 *   "TRB Error".
 *
 * Enable Slot does not need DCBAA at all, so it is the perfect discriminator:
 * program ONLY the candidate CRCR offset, leave DCBAA at zero, start, and see
 * whether a slot comes back. ev0 also shows which address the controller used. */
static void usb_stage46(void)
{
	static const uint8_t cand[2] = { 0x10, 0x18 };
	uint32_t op, sts = 0, slot = 0, code;
	int k, i;

	con_str("USBvd", "CRCR at 0x10 or 0x18? enslot decides");
	wr32(INFRA_AO_BASE + INFRA2_CLR, GATE_INFRA2_USB | GATE_INFRA2_SSUSB_XHCI);
	wr32(INFRA_AO_BASE + INFRA0_CLR, GATE_INFRA0_ICUSB);
	wr32(IPPC_BASE + IPPC_PW_CTRL0,
	     rd32(IPPC_BASE + IPPC_PW_CTRL0) & ~SSUSB_IP_SW_RST);
	wr32(IPPC_BASE + IPPC_PW_CTRL1,
	     rd32(IPPC_BASE + IPPC_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);
	usb_host_up(0x00000004u);		/* PHY up: CNR needs it */
	delay_us(20000);
	op = DWC3_BASE + caplength();
	x_db = DWC3_BASE + (rd32(DWC3_BASE + 0x14) & ~0x3u);
	x_rt = DWC3_BASE + (rd32(DWC3_BASE + 0x18) & ~0x1fu);

	for (k = 0; k < 2; k++) {
		wr32(op + XO_USBCMD, 2u);	/* HCRST */
		delay_us(20000);
		for (i = 0; i < 200; i++) {
			sts = rd32(op + XO_USBSTS);
			if (!(sts & (1u << 11)))
				break;
			delay_us(5000);
		}
		xhci_zero(x_dcbaa, 64);	/* DCBAA deliberately left at 0 */
		xhci_zero(&x_cmd[0][0], RING_TRBS * 4);
		xhci_zero(&x_evt[0][0], 32 * 4);
		x_cmd_cyc = 1;
		x_cmd_i = 0;
		x_evt_cyc = 1;
		x_evt_i = 0;
		/* only the candidate offset gets the ring pointer */
		xhci_wr64(op + cand[k], (uint32_t)(uintptr_t)&x_cmd[0][0] | 1u, 0);
		x_erst[0] = (uint32_t)(uintptr_t)&x_evt[0][0];
		x_erst[1] = 0;
		x_erst[2] = 32;
		x_erst[3] = 0;
		wr32(x_rt + 0x28, 1u);
		xhci_wr64(x_rt + 0x30, (uint32_t)(uintptr_t)&x_erst[0], 0);
		xhci_wr64(x_rt + 0x38, (uint32_t)(uintptr_t)&x_evt[0][0], 0);
		wr32(x_rt + 0x20, 2u);
		wr32(op + XO_USBCMD, 1u);	/* RS */
		delay_us(30000);
		con_hex2("USBvd", "cnr_sts=", sts, "rd=", rd32(op + cand[k]));
		slot = 0;
		code = (uint32_t)xhci_cmd(0, 0, 0, 9u, 0, &slot);
		con_hex2("USBvd", "at=", cand[k], "code=", code);
		con_hex2("USBvd", "slot=", slot, "ev0=", x_last_ev[0]);
	}
}

/* ---- stage 47: correct offsets + full host path ------------------------ */
/* Stage 46 proved the wrapper layout (CRCR 0x18, DCBAAP 0x20, CONFIG 0x28).
 * Everything else is already proven separately:
 *   boost      - A/B/A in stage 36 (0 -> 4987 mV -> 0), VOBST 5.00 V + WDT +
 *                a real 0->1 edge on OPA_MODE
 *   CC/role    - ROLE_CONTROL 0x05 (Rp/Rp), the drive shows Rd on CC1
 *   port       - forced UTMI session (stage 37) + port reset (stage 38) -> U0, HS
 * This stage just chains them and hands over to the enumeration body. */
static void usb_stage47(void)
{
	uint8_t c11 = 0, c1d = 0, rl = 0;

	con_str("USBve", "ring fixed (0x18) + host + enum");
	bb_pins_init();
	bb_rd8(0x34, 0x11, &c11);
	bb_rd8(0x34, 0x1d, &c1d);
	bb_wr8(0x34, 0x1d, (uint8_t)(c1d | 0x80u));	/* charger watchdog */
	bb_wr8(0x34, 0x15, 0x5cu);			/* VOBST 5.00 V */
	bb_wr8(0x34, 0x11, (uint8_t)(c11 & ~0x01u));
	delay_us(150000);
	bb_wr8(0x34, 0x11, (uint8_t)((c11 & ~0x01u) | 0x01u));
	delay_us(250000);
	bb_rd8(0x34, 0x11, &c11);
	con_hex2("USBve", "vbus=", pmu_vbus_mv(), "c11=", c11);

	bb_wr8(0x4e, 0x1a, 0x05);		/* Rp default on both CC */
	bb_wr8(0x4e, 0x04, 0x10);		/* look4connection */
	bb_rd8(0x4e, 0x1a, &rl);
	con_hex2("USBve", "role=", rl, "x=", 0);

	/* UTMI session forced valid: VBUSVALID=1, AVALID=1, SESSEND=0 */
	wr32(PHY_U2PHYDTM1,
	     (1u << 13) | (1u << 12) | (1u << 10) | (1u << 5) | (1u << 2));
	delay_us(100000);

	usb_stage14();
}

/* ======================================================================
 * Stage 48 - microSD (MSDC1 @ 0x11240000), the other boot-medium candidate.
 *
 * Facts from the device, not from the datasheet:
 *   live DT /proc/device-tree/msdc@11240000/
 *     index 1, host_function 1 (SD), bus-width 4, max-frequency 200 MHz,
 *     clk_src 4, cd-gpios <&pio 8 0>, cd_level 1 (HIGH = inserted)
 *     vmmc = mt_pmic_vmch_ldo_reg (MT6360 LDO5, 3.0 V)
 *     vqmmc = mt_pmic_vmc_ldo_reg (MT6360 LDO3, 3.0 V)
 *     clocks = INFRACFG_AO_MSDC1_SCK_CG, INFRACFG_AO_MSDC1_CG
 *   recovery dmesg (no card inserted):
 *     [msdc]msdc1 -> Set<260KHz> Source<191999KHz> -> sclk<259KHz> div<185>
 *     -> sclk = src / (4 * div), so 400 kHz is div 120 out of 192 MHz
 *   mtk-sd.c / clk-mt6785.c:
 *     INFRA1 gates are active-low (set 0x88 / clr 0x8c / sta 0x94),
 *     MSDC1 = bit 4, MSDC1_SCK = bit 16
 *     INFRA1 gates must be open before MSDC1 registers answer at all.
 *
 * MT6360 LDO page protocol (mt6360_ldo_i2c.c): register byte is
 * (reg & 0x3f) | ((len-1) << 6), payload followed by a CRC8 (poly 0x07)
 * computed over [slave<<1 (+1 for read), reg byte, data...]. The read
 * stream is [echo][data...][crc]. The slave address is not documented in
 * the DTS (the LDO is a sub-device of the PMIC), so stage 48 probes the
 * three plausible addresses and accepts only a CRC-valid answer.
 * ==================================================================== */
#define MSDC1_BASE	0x11240000UL
#define MSDC1_TOP	0x11e10000UL
#define INFRA1_SET	0x88
#define INFRA1_CLR	0x8c
#define INFRA1_STA	0x94
#define GATE_MSDC1	(1u << 4)
#define GATE_MSDC1_SCK	(1u << 16)

#define M_CFG		0x00
#define M_IOCON		0x04
#define M_PS		0x08
#define M_INT		0x0c
#define M_INTEN		0x10
#define M_FIFOCS	0x14
#define M_SDC_CFG	0x30
#define M_SDC_CMD	0x34
#define M_SDC_ARG	0x38
#define M_SDC_STS	0x3c
#define M_SDC_RESP0	0x40
#define M_PATCH_BIT	0xb0
#define M_PATCH_BIT1	0xb4
#define M_PAD_TUNE	0xec

#define M_CFG_MODE	(1u << 0)
#define M_CFG_CKPDN	(1u << 1)
#define M_CFG_RST	(1u << 2)
#define M_CFG_PIO	(1u << 3)
#define M_CFG_CKSTB	(1u << 7)
#define M_CFG_CKDIV	(0xffu << 8)

#define M_INT_CMDRDY	(1u << 8)
#define M_INT_CMDTMO	(1u << 9)
#define M_INT_RSPCRCERR	(1u << 10)

#define M_PS_CDSTS	(1u << 1)
#define M_PS_DAT	(0xffu << 16)
#define M_PS_CMD	(1u << 24)

#define M_AMASK		(0xffu)

static uint32_t msdc_reg(uint32_t off)
{
	return rd32(MSDC1_BASE + off);
}

static uint8_t crc8_07(const uint8_t *p, uint32_t n)
{
	uint8_t c = 0;
	uint32_t i;
	int b;

	for (i = 0; i < n; i++) {
		c ^= p[i];
		for (b = 0; b < 8; b++)
			c = (uint8_t)((c & 0x80u) ? ((c << 1) ^ 0x07u)
						  : (c << 1));
	}
	return c;
}

/* One-byte read from the MT6360 LDO page. 0 = ok, negative = why not.
 * The read stream is [data][crc]; the "echo" byte is not read back, it is
 * only fed into the CRC as a constant (see mt6360_ldo_read_device()). */
static int ldo_rd(uint8_t slave, uint8_t reg, uint8_t *out)
{
	uint8_t d, crc, buf[3];
	int ack;

	bb_start();
	ack = bb_write((uint8_t)(slave << 1));
	if (!ack) { bb_stop(); return -1; }
	ack = bb_write((uint8_t)(reg & 0x3fu));		/* len = 1 */
	if (!ack) { bb_stop(); return -2; }
	bb_start();
	ack = bb_write((uint8_t)((slave << 1) | 1));
	if (!ack) { bb_stop(); return -3; }
	d = bb_read(0);
	crc = bb_read(1);
	bb_stop();

	buf[0] = (uint8_t)((slave << 1) | 1);
	buf[1] = (uint8_t)(reg & 0x3fu);
	buf[2] = d;
	if (crc != crc8_07(buf, 3))
		return -5;
	*out = d;
	return 0;
}

/* One-byte write to the MT6360 LDO page. */
static int ldo_wr(uint8_t slave, uint8_t reg, uint8_t val)
{
	uint8_t buf[3];
	uint8_t crc;
	int ack;

	buf[0] = (uint8_t)(slave << 1);
	buf[1] = (uint8_t)(reg & 0x3fu);
	buf[2] = val;
	crc = crc8_07(buf, 3);

	bb_start();
	ack = bb_write(buf[0]);
	if (!ack) { bb_stop(); return -1; }
	ack = bb_write(buf[1]);
	if (!ack) { bb_stop(); return -2; }
	ack = bb_write(val);
	if (!ack) { bb_stop(); return -3; }
	ack = bb_write(crc);
	if (!ack) { bb_stop(); return -4; }
	bb_stop();
	return 0;
}

static void msdc_reset_hw(void)
{
	uint32_t i;

	wr32(MSDC1_BASE + M_CFG, msdc_reg(M_CFG) | M_CFG_RST);
	for (i = 0; i < 10000; i++) {
		if (!(msdc_reg(M_CFG) & M_CFG_RST))
			break;
		delay_us(1);
	}
	wr32(MSDC1_BASE + M_FIFOCS, (1u << 31));	/* clear FIFO */
	delay_us(20);
}

/* Issue one command and return the MSDC_INT snapshot. resp: 0 none, 1 R1/R5/R6/R7,
 * 2 R2, 3 R3 -- same encoding as msdc_cmd_find_resp() in mtk-sd.c. */
static uint32_t msdc_cmd(uint32_t op, uint32_t arg, uint32_t resp,
			 uint32_t to_us)
{
	uint32_t raw = (op & 0x3fu) | ((resp & 7u) << 7);
	uint64_t t0;
	uint32_t v = 0;

	wr32(MSDC1_BASE + M_INT, 0xffffffffu);		/* W1C, clear all */
	wr32(MSDC1_BASE + M_SDC_ARG, arg);
	wr32(MSDC1_BASE + M_SDC_CMD, raw);
	t0 = cnt();
	while (cnt() - t0 < (uint64_t)to_us * 13u) {
		v = msdc_reg(M_INT);
		if (v & (M_INT_CMDRDY | M_INT_CMDTMO | M_INT_RSPCRCERR))
			break;
	}
	return v;
}

static void sd_stage48(void)
{
	static const uint8_t cand[3] = { 0x75, 0x34, 0x4e };
	int found = -1;
	int r;
	uint32_t i, v, ps, ocr = 0;
	uint8_t d5 = 0, d3 = 0;

	con_str("SD", "MSDC1 @11240000 + MT6360 LDO, stage 48");

	/* Gate MSDC1 + MSDC1_SCK first: everything else reads 0 until then. */
	wr32(INFRA_AO_BASE + INFRA1_CLR, GATE_MSDC1 | GATE_MSDC1_SCK);
	delay_us(2000);
	con_hex2("SD", "infra1_sta=", rd32(INFRA_AO_BASE + INFRA1_STA),
		 "cfg=", msdc_reg(M_CFG));
	con_hex2("SD", "ps=", msdc_reg(M_PS), "inten=", msdc_reg(M_INTEN));
	con_hex2("SD", "top=", rd32(MSDC1_TOP), "top1=", rd32(MSDC1_TOP + 4));

	/* Find the LDO slave address: only a CRC-valid read counts. */
	bb_pins_init();
	for (i = 0; i < 3; i++) {
		r = ldo_rd(cand[i], 0x0b, &d5);		/* LDO5_EN_CTRL2 */
		con_hex2("SD", "ldo_probe=", cand[i], "d5=",
			 (uint32_t)((r | 0x100u) << 8 | d5));
		if (r == 0) {
			found = (int)i;
			break;
		}
	}
	if (found >= 0) {
		uint8_t sl = cand[found];

		ldo_wr(sl, 0x0b, (uint8_t)(d5 | 0x40u));	/* LDO5 VMCH 3.0 V */
		ldo_rd(sl, 0x05, &d3);
		ldo_wr(sl, 0x05, (uint8_t)(d3 | 0x40u));	/* LDO3 VMC 3.0 V */
		delay_us(5000);
		ldo_rd(sl, 0x0b, &d5);
		ldo_rd(sl, 0x05, &d3);
	}
	con_hex2("SD", "ldo5_en=", d5, "ldo3_en=", d3);
	delay_us(5000);
	ps = msdc_reg(M_PS);
	con_hex2("SD", "ps_pwr=", ps, "cd=", (ps & M_PS_CDSTS) ? 1 : 0);

	/* Minimal msdc_init_hw(): SD mode, free running clock, PIO, no IRQs. */
	wr32(MSDC1_BASE + M_CFG, M_CFG_MODE | M_CFG_CKPDN | M_CFG_PIO);
	msdc_reset_hw();
	wr32(MSDC1_BASE + M_INTEN, 0);
	wr32(MSDC1_BASE + M_INT, 0xffffffffu);
	wr32(MSDC1_BASE + M_PATCH_BIT, 0x403c0046u);
	wr32(MSDC1_BASE + M_PATCH_BIT1, 0xffff4089u);
	wr32(MSDC1_BASE + M_IOCON, 0);
	wr32(MSDC1_BASE + M_SDC_CFG, (1u << 19) | (3u << 24));

	/* 400 kHz: 192 MHz / (4 * 120). */
	v = msdc_reg(M_CFG) & ~M_CFG_CKDIV;
	v |= (120u << 8);
	wr32(MSDC1_BASE + M_CFG, v);
	delay_us(3000);
	con_hex2("SD", "cfg=", msdc_reg(M_CFG), "ckstb=",
		 (msdc_reg(M_CFG) & M_CFG_CKSTB) ? 1 : 0);

	v = msdc_cmd(0, 0, 0, 200000);			/* CMD0 GO_IDLE */
	con_hex2("SD", "cmd0_int=", v, "sts=", msdc_reg(M_SDC_STS));

	v = msdc_cmd(8, 0x1aau, 1, 200000);		/* CMD8 SEND_IF_COND */
	con_hex2("SD", "cmd8_int=", v, "resp=", msdc_reg(M_SDC_RESP0));

	for (i = 0; i < 20; i++) {
		msdc_cmd(55, 0, 1, 100000);		/* APP_CMD */
		v = msdc_cmd(41, 0x40ff8000u, 3, 100000);	/* ACMD41 HCS */
		ocr = msdc_reg(M_SDC_RESP0);
		if (ocr & 0x80000000u)
			break;
		delay_us(10000);
	}
	con_hex2("SD", "acmd41_int=", v, "ocr=", ocr);
	con_hex2("SD", "resp0=", msdc_reg(M_SDC_RESP0),
		 "int_last=", msdc_reg(M_INT));
	con_str("SD", "stage 48 done");
}

/* Pin 134..139 are MSDC1 (mt6785-pinfunc.h: CLK 134, CMD 135, DAT3 136,
 * DAT0 137, DAT2 138, DAT1 139, function 1). The kernel never sets that mux
 * (pinctrl-mt6785.c has no MSDC entries, the DTS groups only carry
 * drive-strength), so whatever the preloader left is what stage 48 found. */
static uint32_t pin_mode(unsigned p)
{
	return (rd32(g_mode(p)) >> ((p & 7u) * 4u)) & 0xfu;
}

static void pin_func(unsigned p, uint32_t func)
{
	uintptr_t r = g_mode(p);
	uint32_t sh = (p & 7u) * 4u;
	uint32_t d = rd32(r);

	d = (d & ~(0xfu << sh)) | ((func & 0xfu) << sh);
	wr32(r, d);
}

/* MSDC init shared by the SD stages. */
static void msdc_bringup(void)
{
	uint32_t v;

	wr32(MSDC1_BASE + M_CFG, M_CFG_MODE | M_CFG_CKPDN | M_CFG_PIO);
	msdc_reset_hw();
	wr32(MSDC1_BASE + M_INTEN, 0);
	wr32(MSDC1_BASE + M_INT, 0xffffffffu);
	wr32(MSDC1_BASE + M_PATCH_BIT, 0x403c0046u);
	wr32(MSDC1_BASE + M_PATCH_BIT1, 0xffff4089u);
	wr32(MSDC1_BASE + M_IOCON, 0);
	wr32(MSDC1_BASE + M_SDC_CFG, (1u << 19) | (3u << 24));

	/* 400 kHz: 192 MHz / (4 * 120). */
	v = msdc_reg(M_CFG) & ~M_CFG_CKDIV;
	v |= (120u << 8);
	wr32(MSDC1_BASE + M_CFG, v);
	delay_us(3000);
}

static void sd_stage49(void)
{
	static const uint8_t cand[3] = { 0x34, 0x75, 0x4e };
	static const unsigned pins[6] = { 134, 135, 136, 137, 138, 139 };
	int found = -1;
	int r;
	uint32_t i, v, ps, ocr = 0;
	uint32_t cid0, cid1, cid2, cid3;
	uint32_t csd0, csd1, csd2, csd3;
	uint32_t csz, mb = 0;
	uint8_t d5 = 0, d3 = 0;

	con_str("SD", "stage 49: LDO raw + pad mux + CID/CSD");

	wr32(INFRA_AO_BASE + INFRA1_CLR, GATE_MSDC1 | GATE_MSDC1_SCK);
	delay_us(2000);
	con_hex2("SD", "infra1_sta=", rd32(INFRA_AO_BASE + INFRA1_STA),
		 "cfg=", msdc_reg(M_CFG));

	/* LDO page: framing is [data][crc]; print err|data for each address. */
	bb_pins_init();
	for (i = 0; i < 3; i++) {
		r = ldo_rd(cand[i], 0x0b, &d5);
		con_hex2("SD", "ldo5_en2=", cand[i], "err_d=",
			 (uint32_t)(r < 0 ? ((-r) << 8) : 0) | d5);
		if (r == 0) {
			found = (int)i;
			break;			/* first CRC-valid slave wins */
		}
	}
	if (found >= 0) {
		uint8_t sl = cand[found];

		ldo_rd(sl, 0x0b, &d5);
		ldo_wr(sl, 0x0b, (uint8_t)(d5 | 0x40u));	/* LDO5 = VMCH */
		ldo_rd(sl, 0x05, &d3);
		ldo_wr(sl, 0x05, (uint8_t)(d3 | 0x40u));	/* LDO3 = VMC */
		delay_us(8000);
		ldo_rd(sl, 0x0b, &d5);
		ldo_rd(sl, 0x05, &d3);
		con_hex2("SD", "ldo5=", d5, "ldo3=", d3);
	} else {
		con_str("SD", "ldo: no crc-valid slave, power untouched");
	}

	/* Pads: print what the preloader left, then force MSDC1 (func 1). */
	con_hex2("SD", "mode134=", rd32(g_mode(134)),
		 "mode136=", rd32(g_mode(136)));
	for (i = 0; i < 6; i++) {
		if (pin_mode(pins[i]) != 1u)
			pin_func(pins[i], 1u);
	}
	delay_us(1000);
	con_hex2("SD", "mode134b=", rd32(g_mode(134)),
		 "mode136b=", rd32(g_mode(136)));

	ps = msdc_reg(M_PS);
	con_hex2("SD", "ps=", ps, "dat=", (ps & M_PS_DAT) >> 16);

	msdc_bringup();
	con_hex2("SD", "cfg=", msdc_reg(M_CFG), "ckstb=",
		 (msdc_reg(M_CFG) & M_CFG_CKSTB) ? 1 : 0);

	v = msdc_cmd(0, 0, 0, 200000);			/* CMD0 */
	con_hex2("SD", "cmd0_int=", v, "sts=", msdc_reg(M_SDC_STS));

	v = msdc_cmd(8, 0x1aau, 1, 200000);		/* CMD8 */
	con_hex2("SD", "cmd8_int=", v, "resp=", msdc_reg(M_SDC_RESP0));

	for (i = 0; i < 20; i++) {
		msdc_cmd(55, 0, 1, 100000);
		v = msdc_cmd(41, 0x40ff8000u, 3, 100000);
		ocr = msdc_reg(M_SDC_RESP0);
		if (ocr & 0xc0000000u)
			break;
		delay_us(10000);
	}
	con_hex2("SD", "acmd41_int=", v, "ocr=", ocr);

	msdc_cmd(2, 0, 2, 200000);			/* ALL_SEND_CID */
	cid0 = msdc_reg(M_SDC_RESP0);
	cid1 = msdc_reg(M_SDC_RESP0 + 4);
	cid2 = msdc_reg(M_SDC_RESP0 + 8);
	cid3 = msdc_reg(M_SDC_RESP0 + 12);
	con_hex2("SD", "cid0=", cid0, "cid1=", cid1);
	con_hex2("SD", "cid2=", cid2, "cid3=", cid3);

	msdc_cmd(3, 0, 1, 200000);			/* SEND_RELATIVE_ADDR */
	v = msdc_reg(M_SDC_RESP0);
	con_hex2("SD", "rca=", v >> 16, "sts=", msdc_reg(M_SDC_STS));

	msdc_cmd(9, (v & 0xffff0000u), 2, 200000);	/* SEND_CSD */
	csd0 = msdc_reg(M_SDC_RESP0);
	csd1 = msdc_reg(M_SDC_RESP0 + 4);
	csd2 = msdc_reg(M_SDC_RESP0 + 8);
	csd3 = msdc_reg(M_SDC_RESP0 + 12);
	con_hex2("SD", "csd0=", csd0, "csd1=", csd1);
	con_hex2("SD", "csd2=", csd2, "csd3=", csd3);

	/* CSD v2.0: C_SIZE = bits[69:48], capacity = (C_SIZE+1) * 512 KiB. */
	csz = ((csd1 & 0x3fu) << 16) | (csd2 >> 16);
	if ((csd0 >> 30) == 1u)
		mb = (csz + 1u) / 2u;
	con_hex2("SD", "csd_ver=", csd0 >> 30, "mb=", mb);
	con_str("SD", "stage 49 done");
}


/* ---- stage 60: the register/context facts, not another hypothesis ------ */
/* ---- stage 62: HFS+ catalog walk (proves the volume is readable) ------- */
static uint8_t hfs_node[8192] __attribute__((aligned(64)));

static uint32_t be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* READ(10) of nsec 512-byte sectors into buf; 0 = ok. */
static int hfs_read(uint32_t lba, uint32_t nsec, uint8_t *buf)
{
	uint8_t cdb[16];
	int i;

	for (i = 0; i < 16; i++)
		cdb[i] = 0;
	cdb[0] = 0x28;			/* READ(10) */
	cdb[2] = (uint8_t)(lba >> 24);
	cdb[3] = (uint8_t)(lba >> 16);
	cdb[4] = (uint8_t)(lba >> 8);
	cdb[5] = (uint8_t)lba;
	cdb[7] = (uint8_t)(nsec >> 8);
	cdb[8] = (uint8_t)nsec;
	return bot(cdb, 10, 1, nsec * 512u, buf);
}

/* Decode a UTF-16BE HFS+ name into a short ASCII buffer. */
static void hfs_name(uint8_t *dst, const uint8_t *src, uint32_t chars,
		     uint32_t max)
{
	uint32_t i;

	if (chars > max - 1)
		chars = max - 1;
	for (i = 0; i < chars; i++) {
		uint16_t c = be16(src + i * 2);
		dst[i] = (c && c < 0x80) ? (uint8_t)c : '?';
	}
	dst[chars] = 0;
}

static void hfs_catalog(void)
{
	uint32_t bs, cat_start, node_size, leaf, root;
	uint32_t i, shown = 0, leaves = 0;
	int cc;

	cc = hfs_read(2, 1, hfs_node);		/* volume header */
	if (cc != 0 || hfs_node[0] != 'H' || hfs_node[1] != 'X') {
		con_hex2("HFS2", "no volume header", 0, "cc=", (uint32_t)cc);
		return;
	}
	bs = be32(hfs_node + 40);
	cat_start = be32(hfs_node + 288);	/* catalog fork, first extent */
	if (bs < 512 || bs > 65536) {
		con_hex2("HFS2", "bad block size", bs, "x=", 0);
		return;
	}
	cc = hfs_read(cat_start * (bs / 512u), bs / 512u, hfs_node);
	con_hex2("HFS2", "cat_blk=", cat_start, "read=", (uint32_t)cc);
	if (cc != 0)
		return;
	node_size = be16(hfs_node + 14 + 18);
	root = be32(hfs_node + 14 + 2);
	leaf = be32(hfs_node + 14 + 10);
	con_hex2("HFS2", "nodesize=", node_size, "root=", root);
	con_hex2("HFS2", "firstleaf=", leaf, "depth=", be16(hfs_node + 14));

	if (node_size < 512 || node_size > 8192 || leaf == 0)
		return;

	/* Walk leaf nodes and print root-directory entries (parentID 2). */
	while (leaf != 0 && leaves < 24 && shown < 8) {
		uint32_t nr;

		cc = hfs_read(cat_start * (bs / 512u) +
			      leaf * (node_size / 512u),
			      node_size / 512u, hfs_node);
		if (cc != 0)
			break;
		nr = be16(hfs_node + 10);
		for (i = 0; i < nr && shown < 8; i++) {
			uint32_t off = be16(hfs_node + node_size - 2 * (i + 1));
			uint32_t parent, nlen;
			char name[24];

			if (off + 8 >= node_size)
				continue;
			parent = be32(hfs_node + off + 2);
			nlen = be16(hfs_node + off + 6);
			if (parent != 2 || nlen == 0 || nlen > 30)
				continue;
			hfs_name((uint8_t *)name, hfs_node + off + 8, nlen,
				 sizeof(name));
			/* HFS+ sorts dot-names and the private-data directory
			 * first; skip them so the real volume root is visible. */
			if (name[0] == '.' || name[0] == '?' || name[0] == 0)
				continue;
			con_str("ROOT", name);
			shown++;
		}
		leaves++;
		leaf = be32(hfs_node + 0);	/* fLink */
	}
	con_hex2("HFS2", "leaves=", leaves, "entries=", shown);
}

/* ---- stage 63: descend System/Library/CoreServices/SystemVersion.plist -- */
static struct {
	uint32_t bs, cat_start, node_size, first_leaf;
	uint32_t ext_start[8], ext_count[8], ext_n;
	int ok;
} cat;

static uint8_t hfs_file[2048] __attribute__((aligned(64)));

/* The catalog is a fork that may span several extents; translate a node
 * index into a disk LBA instead of assuming one contiguous run. */
static uint32_t cat_lba(uint32_t idx)
{
	uint32_t byte = idx * cat.node_size;
	uint32_t blk = byte / cat.bs, off = byte % cat.bs;
	uint32_t i, cur = 0;

	for (i = 0; i < cat.ext_n; i++) {
		if (blk < cur + cat.ext_count[i]) {
			uint32_t dblk = cat.ext_start[i] + (blk - cur);
			return dblk * (cat.bs / 512u) + off / 512u;
		}
		cur += cat.ext_count[i];
	}
	return 0xffffffffu;
}

static int cat_node(uint32_t idx)
{
	uint32_t lba = cat_lba(idx);

	if (lba == 0xffffffffu)
		return -3;
	return hfs_read(lba, cat.node_size / 512u, hfs_node);
}

/* Look up (parent, name). Fills cnid; for files also data fork start/size. */
static int cat_lookup(uint32_t parent, const char *name, uint32_t *cnid,
		      uint32_t *dstart, uint32_t *dsize)
{
	uint32_t leaf = cat.first_leaf, hops;
	uint32_t i;

	if (!cat.ok)
		return -1;
	for (hops = 0; leaf && hops < 6000; hops++) {
		uint32_t nr;

		int read_rc = cat_node(leaf);
		if (read_rc != 0) {
			con_hex2("CATERR", "node=", leaf, "rc=", (uint32_t)read_rc);
			return -2;
		}
		nr = be16(hfs_node + 10);
		for (i = 0; i < nr; i++) {
			uint32_t off = be16(hfs_node + cat.node_size - 2 * (i + 1));
			uint32_t par, nlen, keylen, data, rtype;
			char nm[32];
			uint32_t k;

			if (off + 10 >= cat.node_size)
				continue;
			par = be32(hfs_node + off + 2);
			if (par != parent)
				continue;
			nlen = be16(hfs_node + off + 6);
			if (nlen == 0 || nlen > 30)
				continue;
			hfs_name((uint8_t *)nm, hfs_node + off + 8, nlen, sizeof(nm));
			for (k = 0; nm[k] && name[k] && nm[k] == name[k]; k++)
				;
			if (nm[k] != 0 || name[k] != 0)
				continue;
			keylen = be16(hfs_node + off);
			data = off + 2 + keylen;
			if (data + 108 >= cat.node_size)
				continue;
			rtype = be16(hfs_node + data);
			if (rtype == 1) {			/* folder */
				*cnid = be32(hfs_node + data + 8);
				return 1;
			}
			if (rtype == 2) {			/* file */
				*cnid = be32(hfs_node + data + 8);
				*dstart = be32(hfs_node + data + 104);
				*dsize = be32(hfs_node + data + 92);
				return 2;
			}
		}
		leaf = be32(hfs_node + 0);
	}
	return 0;
}

/* freestanding: no libc memcmp available at link time */
static int hfs_eq(const uint8_t *p, const char *lit, uint32_t n)
{
	uint32_t i;

	for (i = 0; i < n; i++)
		if (p[i] != (uint8_t)lit[i])
			return 0;
	return 1;
}

static void hfs_sysver(void)
{
	uint32_t id = 0, dstart = 0, dsize = 0, i, n;
	int r;

	cat.ok = 0;
	if (hfs_read(2, 1, hfs_node) != 0 || hfs_node[0] != 'H' || hfs_node[1] != 'X')
		return;
	cat.bs = be32(hfs_node + 40);
	cat.cat_start = be32(hfs_node + 288);
	cat.ext_n = 0;
	{
		uint32_t e;

		for (e = 0; e < 8; e++) {
			uint32_t st = be32(hfs_node + 288 + e * 8);
			uint32_t cn = be32(hfs_node + 288 + e * 8 + 4);

			if (cn == 0)
				break;
			cat.ext_start[cat.ext_n] = st;
			cat.ext_count[cat.ext_n] = cn;
			cat.ext_n++;
		}
	}
	if (cat.bs < 512 || cat.bs > 65536)
		return;
	/* node_size is unknown until this read, so read one block: the B-tree
	 * header record covers it. Reading with nsec 0 would be a no-op. */
	if (hfs_read(cat.cat_start * (cat.bs / 512u), cat.bs / 512u,
		     hfs_node) != 0)
		return;
	cat.node_size = be16(hfs_node + 14 + 18);
	cat.first_leaf = be32(hfs_node + 14 + 10);
	if (cat.node_size < 512 || cat.node_size > 8192)
		return;
	cat.ok = 1;

	r = cat_lookup(2, "System", &id, &dstart, &dsize);
	con_hex2("HFS3", "System=", (uint32_t)r, "id=", id);
	if (r != 1)
		return;
	r = cat_lookup(id, "Library", &id, &dstart, &dsize);
	con_hex2("HFS3", "Library=", (uint32_t)r, "id=", id);
	if (r != 1)
		return;
	r = cat_lookup(id, "CoreServices", &id, &dstart, &dsize);
	con_hex2("HFS3", "CoreServices=", (uint32_t)r, "id=", id);
	if (r != 1)
		return;
	r = cat_lookup(id, "SystemVersion.plist", &id, &dstart, &dsize);
	con_hex2("HFS3", "plist=", (uint32_t)r, "size=", dsize);
	if (r != 2 || dsize == 0 || dsize > sizeof(hfs_file))
		return;

	n = (dsize + 511u) / 512u;
	if (hfs_read(dstart * (cat.bs / 512u), n, hfs_file) != 0)
		return;

	/* Scan the XML for ProductVersion / ProductBuildVersion values. */
	for (i = 0; i + 16 < dsize; i++) {
		const char *key = 0;
		uint32_t j, o;

		if (hfs_file[i] != 'P')
			continue;
		if (hfs_eq(hfs_file + i, "ProductVersion<", 15))
			key = "ver=";
		else if (hfs_eq(hfs_file + i, "ProductBuildVersion<", 20))
			key = "build=";
		if (!key)
			continue;
		for (o = i; o + 10 < dsize && hfs_file[o] != '>'; o++)
			;
		for (j = o + 9; j < dsize && hfs_file[j] != '<' && j < o + 9 + 20; j++)
			;
		if (j > o + 9 && j - (o + 9) < 24) {
			char val[26];
			uint32_t k = 0, p;

			for (p = o + 9; p < j && k < sizeof(val) - 1; p++)
				val[k++] = (char)hfs_file[p];
			val[k] = 0;
			con_str("HFS3", key);
			con_str("HFS3", val);
		}
	}
}

static void usb_stage60(void)
{
	uintptr_t op;

	con_str("USB", "stage 60: wrapper register map + context writes");
	x_diag = 1;
	usb_stage47();			/* power, phy, host, full enumeration */

	/* Controller comes from stage 47 running; stop it before overwriting
	 * DCBAAP/CONFIG with markers. */
	op = DWC3_BASE + caplength();
	wr32(op + XO_USBCMD, 0);
	delay_us(20000);
	xhci_map_probe("MAP", op);
	con_str("USB", "stage 60 done");
}

void usb_probe(int stage)
{
	/* Compact current probe: skip historical IPPC/DWC3 and register dumps.
	 * Keep enumeration, endpoint and BOT results together on one screen. */
	if (stage >= 62) {
		con_str("USB", "stage 62: HFS+ catalog on USB");
		x_diag = 0;
		usb_stage47();
		/* Enumeration output already filled the screen; the catalog
		 * result is what matters here, so give it a clean frame. */
		con_clear(0xff000040U);
		con_set_row(1);
		con_str("USB", "stage 63: read SystemVersion.plist");
		hfs_catalog();
		con_set_row(14);
		hfs_sysver();
		con_str("USB", "probe finished");
		return;
	}
	if (stage >= 61) {
		con_str("USB", "stage 61 compact: USB storage test");
		x_diag = 0;
		usb_stage47();
		con_str("USB", "probe finished");
		return;
	}
	con_str("USB", stage >= 61 ? "stage 61: standard xHCI + USB BOT"
			  : stage >= 60 ? "MT6785 xHCI reg map + ctx writes"
			  : stage >= 49 ? "MT6785 microSD: LDO + pads + CID/CSD"
			  : stage >= 48 ? "MT6785 microSD MSDC1 @11240000"
			  : stage >= 47 ? "MT6785 ring fixed + host + enum"
			  : stage >= 46 ? "MT6785 CRCR offset probe"
			  : stage >= 45 ? "MT6785 pw_ctrl2 + phy + CNR"
			  : stage >= 44 ? "MT6785 CNR + MAC + slot type"
			  : stage >= 43 ? "MT6785 ssusb26m RMW + enum"
			  : stage >= 42 ? "MT6785 read-only clock/CNR snapshot"
			  : stage >= 41 ? "MT6785 ssusb26m gate + CNR wait"
			  : stage >= 40 ? "MT6785 op register map"
			  : stage >= 39 ? "MT6785 command ring readback"
			  : stage >= 38 ? "MT6785 session force + port reset + enum"
			  : stage >= 37 ? "MT6785 line/cc/port probe"
			  : stage >= 36 ? "MT6785 A/B/A vbus + host + enum"
			  : stage >= 35 ? "MT6785 boost: watchdog/edge/ovp tests"
			  : stage >= 34 ? "MT6785 VBUS voltmeter + boost"
			  : stage >= 33 ? "MT6785 tpcc full init + boost"
			  : stage >= 32 ? "MT6785 PMU diff off/on"
			  : stage >= 31 ? "MT6785 real VBUS check (otg_sts)"
			  : stage >= 30 ? "MT6785 OTG v5 (role sweep)"
			  : stage >= 28 ? "MT6785 OTG v4 (tcpc init + boost)"
			  : stage >= 27 ? "MT6785 OTG v3 (role Rp + boost)"
			  : stage >= 26 ? "MT6785 OTG v2 (bc1.2 off, 5.0V)"
			  : stage >= 25 ? "MT6785 OTG boost + host port"
			  : stage >= 24 ? "MT6785 tcpc dump + pmu variants (ro)"
			  : stage >= 23 ? "MT6785 bit-bang register sweep (ro)"
			  : stage >= 22 ? "MT6785 bit-bang i2c to MT6360 (ro)"
			  : stage >= 21 ? "MT6785 i2c5: dt clocks + retry (ro)"
			  : stage >= 20 ? "MT6785 i2c5: pericfg mode + retry (ro)"
			  : stage >= 19 ? "MT6785 i2c5: i2c_sel mux + retry (ro)"
			  : stage >= 18 ? "MT6785 i2c5 V2 + hw_timeout (ro)"
			  : stage >= 17 ? "MT6785 i2c5 V2 probe of MT6360 (ro)"
			  : stage >= 16 ? "MT6785 i2c5 probe of MT6360 (read-only)"
			  : stage >= 14 ? "MT6785 DWC3 probe, stage 14 (cfg+eps)"
			  : stage >= 13 ? "MT6785 DWC3 probe, stage 13 (enumerate)"
			  : stage >= 12 ? "MT6785 DWC3 probe, stage 12 (port poll)"
			  : stage >= 11 ? "MT6785 DWC3 probe, stage 11 (phy recover)"
			  : stage >= 10 ? "MT6785 DWC3 probe, stage 10 (port bisect)"
			  : stage >= 9 ? "MT6785 DWC3 probe, stage 9 (op base hunt)"
			  : stage >= 8 ? "MT6785 DWC3 probe, stage 8 (xHCI+HCRST)"
			  : stage >= 7 ? "MT6785 DWC3 probe, stage 7 (start xHCI)"
			  : stage >= 6 ? "MT6785 DWC3 probe, stage 6 (reset+host)"
			  : stage >= 5 ? "MT6785 DWC3 probe, stage 5 (id+rw test)"
			  : stage >= 4 ? "MT6785 DWC3 probe, stage 4 (PHY+host)"
			  : stage >= 3 ? "MT6785 DWC3 probe, stage 3 (clk bisect)"
			  : stage >= 2 ? "MT6785 DWC3 probe, stage 2 (IPPC power-on)"
				       : "MT6785 DWC3 probe, stage 0 (read-only)");
	usb_report("USBP0");
	if (stage >= 61) {
		x_diag = 1;
		usb_stage47();
	}
	else if (stage >= 60)
		usb_stage60();
	else if (stage >= 49)
		sd_stage49();
	else if (stage >= 48)
		sd_stage48();
	else if (stage >= 47)
		usb_stage47();
	else if (stage >= 46)
		usb_stage46();
	else if (stage >= 45)
		usb_stage45();
	else if (stage >= 44)
		usb_stage44();
	else if (stage >= 43)
		usb_stage43();
	else if (stage >= 42)
		usb_stage42();
	else if (stage >= 41)
		usb_stage41();
	else if (stage >= 40)
		usb_stage40();
	else if (stage >= 39)
		usb_stage39();
	else if (stage >= 38)
		usb_stage38();
	else if (stage >= 37)
		usb_stage37();
	else if (stage >= 36)
		usb_stage36();
	else if (stage >= 35)
		usb_stage35();
	else if (stage >= 34)
		usb_stage34();
	else if (stage >= 33)
		usb_stage33();
	else if (stage >= 32)
		usb_stage32();
	else if (stage >= 31)
		usb_stage31();
	else if (stage >= 30)
		usb_stage30();
	else if (stage >= 28)
		usb_stage28();
	else if (stage >= 27)
		usb_stage27();
	else if (stage >= 26)
		usb_stage26();
	else if (stage >= 25)
		usb_stage25();
	else if (stage >= 24)
		usb_stage24();
	else if (stage >= 23)
		usb_stage23();
	else if (stage >= 22)
		usb_stage22();
	else if (stage >= 21)
		usb_stage21();
	else if (stage >= 20)
		usb_stage20();
	else if (stage >= 19)
		usb_stage19();
	else if (stage >= 18)
		usb_stage18();
	else if (stage >= 17)
		usb_stage17();
	else if (stage >= 16)
		usb_stage16();
	else if (stage >= 14)
		usb_stage14();
	else if (stage >= 13)
		usb_stage13();
	else if (stage >= 12)
		usb_stage12();
	else if (stage >= 11)
		usb_stage11();
	else if (stage >= 10)
		usb_stage10();
	else if (stage >= 9)
		usb_stage9();
	else if (stage >= 8)
		usb_stage8();
	else if (stage >= 7)
		usb_stage7();
	else if (stage >= 6)
		usb_stage6();
	else if (stage >= 5)
		usb_stage5();
	else if (stage >= 4)
		usb_stage4();
	else if (stage >= 3)
		usb_bisect();
	else if (stage >= 2) {
		usb_ip_power_on();
		usb_check_port("USBP2");
	}
	if (stage >= 48 && stage < 60) {
		con_str("SD", "probe finished");
		return;
	}
	/* U3PHY is a different power domain: read it last so a bus hang here
	 * cannot swallow the results above. */
	con_str("USBP4", "rd U3PHY @11e40000 (last: may hang)");
	con_hex2("USBP4", "u3phy0=", rd32(U3PHY_BASE + 0x0000),
		 "u3phy4=", rd32(U3PHY_BASE + 0x0004));
	con_str("USB", "probe finished");
}

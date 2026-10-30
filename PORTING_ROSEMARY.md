> **Review note (2026-10-30):** the uploaded phone IRQ snapshot shows `fts_ts`
> receiving many interrupts and `goodix_fp_irq` much fewer. Re-check the live
> touchscreen node and I²C controller before assuming the display touch panel
> is Goodix. See the current handoff at the top of `NOTES.md`.

# Porting ios-sweet → ios-rosemary (Redmi Note 10 Pro → Redmi Note 10S)

**Review status (2026-10-30):** the supplied `boot_a.bin` DTB directly
confirms the corrected GICD/GICR tuples; the live `/proc/device-tree` raw
`reg` bytes and a target boot are still unverified. The source archive contains
historical `ios-sweet`/Redmi Note 10 Pro notes; do not treat those as proof that
iOS has reached the home screen on the Redmi Note 10S. Use the timestamped
current handoff at the top of `NOTES.md` as the source of truth.

**Historical status claim (2026-10-27; not independently validated here):**
the original handoff called the mechanical port complete and described the
XNU/UFS/framebuffer/SMP work as ported. That statement must not be read as proof
that the supplied build boots on rosemary or displays its console. The present
review verifies selected device-tree constants and source-level compilation
only; hardware boot remains untested.

This is a fork of `dipa-studio/ios-sweet`. Machine config renamed `SWEET` →
`ROSEMARY`; all tooling, boot-args and patches renamed consistently. The
repository validator (`tools/validate-repo.py`) passes: 81 patches contiguous,
all Python/shell tools syntax-OK.

---

## 1. Why this is not a simple rename

| | Redmi Note 10 Pro (`sweet`) | Redmi Note 10S (`rosemary`) |
| --- | --- | --- |
| SoC | Qualcomm Snapdragon 732G (SM7150) | **MediaTek Helio G95 (MT6785)** |
| CPU | 2×Cortex-A76 @2.30 + 6×A55 @1.80 GHz | 2×Cortex-A76 @2.05 + 6×A55 @2.00 GHz |
| UFS host | Qualcomm ufs_mem @ `0x01d84000` | MediaTek ufshci @ **`0x11270000`** (JEDEC-standard HCI regs, M-PHY @ `0x11fa0000`) |
| Framebuffer | `0x9c000000` (cont_splash) | **`0x7bd20000`**, size `0x22e0000` (mblock-20-framebuffer) |
| GICv3 | GICD `0x17a00000`, GICR `0x17a60000` | GICD **`0x0c000000/0x40000`**, GICR **`0x0c040000/0x200000`** (confirmed in the supplied `boot_a.bin` DTB; live raw `reg` still to capture) |
| Arch timer | 19.2 MHz, vtimer INTID 19 (PPI 3) | **13 MHz**, vtimer INTID **27** (PPI 11, the "usual" value) |
| Touch I²C | Qualcomm GENI QUP SE7 @ `0xa84000`, Goodix GT9896 @0x5d | **MediaTek I²C controller** (i2c0@11007000 … i2c9@11015000); touch on one of them (read FDT `touch`/`xiaomi_touch`) |
| Buttons | PM6150/PM6150L PON over **SPMI** | MT6359/MT6360 PMIC over **pwrap** @ `0x1000d000` |
| Touch rail | PM6150L LDO11 via **RPMh** | MT6360 LDO via pwrap (mt6360_ldo_dts in FDT) |
| Battery fuel gauge | PMIC SPMI | **mt6359_gauge** via pwrap |
| USB | Qualcomm dwc3 + QMP PHY | **mtu3** @ `0x11200000` (dwc3 core + MediaTek wrapper/PHY) |
| RAM above 4 GiB | `0x100000000` + `0xfcc00000` (≈3.95 GiB) | `0x100000000` + **`0xbfc00000`** (≈3 GiB; 6 GiB total, low bank fragmented by MTK carve-outs) |

**Good news:** the boot chain relies on the stock bootloader leaving UFS and
the framebuffer initialized (same strategy on both). The UFS driver in
`shim/ufs.c` and `bsd/dev/rosemary_ufs.c` uses only standard JEDEC UFSHCI
registers — changing the base address is enough. PSCI CPU_ON, GICv3 sysreg
interface, 16K pages and the framebuffer copy are architecture-generic.

---

## 2. What was changed (mechanical port, done)

- Global rename `sweet/SWEET/Sweet` → `rosemary/ROSEMARY/Rosemary` in all
  files and filenames (patches, tools, shim, docs, site, workflow).
- `tools/mkadt.py` (Apple Device Tree generator) — all constants from the dump:
  - `FB=0x7bd20000`, `FB_SIZE=0x22e0000`; panel stays 1080×2400 (same panel class).
  - `UFS=0x11270000`, compatible `mediatek,ufshci`.
  - `GICD=0x0c000000/0x40000`, `GICR=0x0c040000/0x200000` (confirmed by the supplied `boot_a.bin` DTB; raw live `reg` values still need capture).
  - `TIMEBASE=13000000`, `VTIMER_INTID=27`.
  - CPU topology: cores 0–5 A55 @2.00 GHz, 6–7 A76 @2.05 GHz, compatible
    `arm,cortex-a76`. MPIDR Aff1 = core id (same as sweet).
  - Root compatible `xiaomi,rosemary` + `mediatek,mt6785`; synthetic ECID
    `0x524F53454D000001` (`ROSEM`+1). High-mem size `0xbfc00000`.
- `shim/` boot shim:
  - `console.h FB_BASE`, `mmu.h FB_PHYS` → `0x7bd20000`.
  - `shim.c`: GICD/GICR bases, `VTIMER_INTID=27`, `PTIMER_INTID=30`, tick→µs
    divisor 192→130 (13 MHz). Qualcomm-only `IMEM_RESTART_REASON` and `PSHOLD`
    writes removed (MediaTek uses PSCI `SYSTEM_RESET` only).
  - `ufs.c`: `UFS_BASE=0x11270000`, 1 s timeout scaled to 13 MHz ticks.
- XNU patches: all hardcoded Qualcomm MMIO bases replaced globally
  (`0x9c000000`→`0x7bd20000`, `0x01d84000`→`0x11270000`, `0x17a00000`→`0x0c000000`,
  `0x17a60000`→`0x0c040000`, `0xfcc00000`→`0xbfc00000`).
- `.github/workflows/xnu.yml`: default `MACHINE_CONFIG=ROSEMARY`.
- Boot-args renamed: `rosemaryfb_noconsole=1`, `rosemaryfb_hz=30`,
  `rosemarytouch=0`, `rosemarykeys=0` (consistent with renamed handlers in
  patches 0056/0060/0061).

---

## 3. Verify the three guesses before first boot (run on the phone in Termux/root)

The text dump lists many node names but omits some raw property values. The
GICD/GICR values below are verified against the FDT embedded in the supplied
`boot_a.bin`; capture the runtime property and check timer/touch information
before treating every constant as runtime-verified.

### 3a. GIC base (boot_a DTB verified; capture the raw live property)
The DTB section embedded in the supplied `boot_a.bin` declares `compatible = "arm,gic-v3"` with GICD
`<0x0c000000 0x40000>` and GICR `<0x0c040000 0x200000>`. The separate
`gic@1024e000` node has `compatible = "mediatek,gic"` and is not that controller.
The text dump lists `/proc/device-tree/interrupt-controller/reg` as 16 bytes,
but does not include its contents. Capture both candidate nodes:
```sh
su -c 'for n in /proc/device-tree/interrupt-controller /proc/device-tree/gic@*; do [ -f "$n/reg" ] || continue; echo "=== $n ==="; printf "compatible="; tr "\000" " " < "$n/compatible" 2>/dev/null; echo; printf "reg="; od -An -tx1 "$n/reg" | tr -d " \n"; echo; done'
```
Keep the patched constants unless the raw `arm,gic-v3` node proves otherwise.
Do not use the unrelated `mediatek,gic` `0x1024e000` node as GICD/GICR.

### 3b. Timer frequency and virtual-timer PPI
```sh
su -c 'cat /proc/device-tree/timer/interrupts | od -A n -t x4; echo CNTFRQ=$(cat /sys/devices/system/clocksource/clocksource0/arch_timer_freq 2>/dev/null || echo "read cntfrq_el0 in shim")'
```
- The four interrupt cells are (phys-secure, phys-nonsecure, virtual, hyp) as
  `<GIC_PPI N ...>`. Virtual timer INTID = 16 + N. If N ≠ 11, update
  `VTIMER_INTID` in `shim/shim.c` and `tools/mkadt.py`.
- If the frequency is not 13 MHz, update `TIMEBASE` in `mkadt.py`, the
  `to_us` divisor and UFS timeout in `shim/`. (The shim reads `cntfrq_el0` at
  runtime for delays, so only display/timeout constants are affected.)

### 3c. Touch I²C bus + address
```sh
su -c 'ls /proc/device-tree/touch/ /proc/device-tree/xiaomi_touch/ 2>/dev/null; for n in /proc/device-tree/i2c@*/touch@* /proc/device-tree/i2c@*/goodix@* /proc/device-tree/i2c@*/focaltech@*; do [ -e "$n/reg" ] && echo "$n reg=$(od -A n -t x1 "$n/reg" | tr -d " \n")"; done'
```
Note the parent I²C controller base and the device address. The supplied IRQ
snapshot shows `fts_ts` receiving substantially more interrupts than
`goodix_fp_irq`, so do not assume the display touch panel is Goodix from the
model name alone. The touch-driver rewrite (§4) must follow the live node and
I²C probe results; reuse the Goodix protocol only if the panel is confirmed to
be Goodix.

---

## 4. Remaining driver work (register-level rewrites)

These patches apply and compile, but contain Qualcomm-specific register logic
(GENI FIFO, SPMI, RPMh). On rosemary they will not probe hardware. Boot with
`rosemarytouch=0 rosemarykeys=0` first to reach the home screen, then enable
them one at a time.

| Patch | What it does | MediaTek replacement |
| --- | --- | --- |
| `0060-rosemary-keys-touch-drivers.patch` | Power/vol keys over SPMI PMIC; touch over GENI I²C | Keys: read MT6359 PON via `pwrap@1000d000` (or use GPIO keys if wired — check `mtk-pmic-keys` in FDT). Touch: MTK I²C FIFO on the controller from §3c. |
| `0062-rosemary-touch-power-battery.patch` | Touch rail via RPMh LDO11; battery via SPMI fuel gauge | Touch rail: enable MT6360 LDO (`mt6360_ldo_dts` in FDT) via pwrap. Battery: `mt6359_gauge` via pwrap (static 100 % stub already works for boot). |
| `0063-rosemary-goodix-geni-packing.patch` | Goodix GT9896 protocol + GENI FIFO packing | Keep the Goodix protocol (if §3c confirms Goodix); replace only the GENI FIFO transport with MTK I²C reads/writes. |
| `0072–0077, 0080–0081` (USB) | Qualcomm dwc3 device mode + ECM | `usb3@11200000` / `mtu3_0@11200000` is a Synopsys dwc3 core with a MediaTek wrapper — core EP0/ECM logic is reusable; PHY/clock init and wrapper registers differ. Optional (needed only for USB networking/log). |

Reference: MediaTek's published Linux kernel sources for `rosemary`
(`4.14`, `drivers/i2c/busses/i2c-mt65xx.c`, `drivers/mfd/mt6359-core.c`,
`drivers/ufs/host/ufs-mediatek.c`, `drivers/usb/dwc3/dwc3-mtk.c`) give the
exact register layouts.

---

## 5. Build & first boot (same flow as sweet, rosemary names)

1. **Fork this repo** (public → free macOS CI minutes). Kernel builds on
   GitHub Actions (macOS runner, `blacktop/darwin-xnu-build`):
   ```sh
   gh workflow run xnu.yml -R YOU/ios-rosemary
   gh run download RUN_ID -R YOU/ios-rosemary -n xnu-kernel -D xnu/out
   ```
2. **iOS root image (destructive; not needed for the shim-only display test)** —
   the historical flow uses the iPhone X (iPhone10,3) 16.7.16 IPSW and
   `tools/mkiosroot.sh` + `tools/rosemary-root-tweaks.py`. Writing this image to
   `userdata` erases that partition; do not do this as part of console diagnosis.
   Back up data first and only use this step when intentionally setting up the
   iOS root filesystem.
3. **Boot image** (`aarch64-linux-gnu-gcc` + `mkbootimg`):
   ```sh
   python3 tools/mkadt.py shim/out/rosemary.adt --highmem
   python3 tools/mkpayload.py xnu/out/kernel.development.rosemary shim/out/rosemary.adt shim/out/payload.bin \
     --cmdline 'debug=0xc serial=3 cpus=1 keepsyms=1 -enable_kprintf_spam -noprogress rd=ufsroot0 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin ufsrw=1 rootrw=1 launchd_unsecure_cache=1 wdt=-1 rosemaryfb_hz=30 rosemarytouch=0 rosemarykeys=0'
   shim/build.sh ../shim/out/payload.bin
   fastboot boot shim/out/shim-boot.img
   ```
   Keep `rosemarytouch=0 rosemarykeys=0` for the first diagnostic boot. The
   display and XNU handoff on the Redmi Note 10S are not yet verified by this
   review; do not assume the sweet home-screen milestone reproduces here.
4. **Kernel log**: `tools/rosemary-log-pull.sh` (4 MiB ring at end of userdata,
   read from OrangeFox recovery).

---

## 6. First-boot troubleshooting

### Console: distinguish the shim from the XNU framebuffer console

`shim_main()` calls `con_clear(RED)` and prints `SHIM: rosemary milestone 4:
xnu loader` **before** `loader_boot()` transfers control to XNU. This text is
written directly to the ABL framebuffer (`FB_BASE=0x7bd20000`) by
`shim/console.c`; `rosemaryfb_noconsole=1` does not disable it. That boot-arg
mutes only XNU's framebuffer text after the GUI's first framebuffer swap (the
messages continue to the persistent log, and panics can still be drawn).

For a diagnostic build, temporarily use `cpus=1`, retain
`rosemarytouch=0 rosemarykeys=0`, and remove `rosemaryfb_noconsole=1` from the
payload `--cmdline`:
```sh
python3 tools/mkpayload.py xnu/out/kernel.development.rosemary shim/out/rosemary.adt shim/out/payload-debug.bin \
  --cmdline 'debug=0xc serial=3 cpus=1 keepsyms=1 -enable_kprintf_spam -noprogress rd=ufsroot0 unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin ufsrw=1 rootrw=1 launchd_unsecure_cache=1 wdt=-1 rosemaryfb_hz=30 rosemarytouch=0 rosemarykeys=0'
shim/build.sh ../shim/out/payload-debug.bin
```

Interpret what is visible:
- **No red `SHIM:` screen at all:** this happens before the XNU payload is
  entered. The noconsole boot-arg and GIC ADT fix cannot hide this shim output.
  Check that ABL leaves the panel scanning the reserved splash buffer, and
  verify framebuffer base, stride and pixel format against the live bootloader
  handoff. The dump confirms the reserved region `0x7bd20000+0x22e0000`, but
  not the active scanout format/stride at handoff.
- **Red `SHIM:` screen appears, then goes black at XNU handoff:** investigate
  early XNU/ADT initialization, including the corrected GIC mapping; pull the
  persistent kernel log or USB log to see the last completed stage.
- **Kernel text appears and later disappears when the GUI starts:** expected
  with `rosemaryfb_noconsole=1`; omit that arg while debugging.

Use `fastboot boot` for this temporary test. Do not flash `boot`, `vbmeta`,
`dtbo` or other Android partitions as part of console diagnosis.

### Other early-boot checks

- **No early red `SHIM:` screen at all** → this occurs before XNU/GIC
  initialization. Check the ABL framebuffer handoff and `FB_BASE`/stride/format
  first (see the console diagnostic above).
- **The red `SHIM:` screen appears, then no XNU text follows** → check the
  persistent log and the ADT/GIC mapping from §3a; the GIC fix is relevant to
  this stage, not to the shim's initial framebuffer write.
- **Timer interrupts never fire (stuck after "timer interrupt" line)** →
  `VTIMER_INTID` or `TIMEBASE` wrong. Run §3b.
- **Panic in UFS probe / "ufs not ready"** → on some MediaTek bootloaders the
  UFS PHY is gated when ABL hands off; add a minimal unipro/PHY enable sequence
  from `ufs-mediatek.c` in `shim/ufs.c` (the sweet variant relies on ABL
  leaving it running — verify rosemary's ABL does too by reading HCE/HCS in
  the shim probe output on screen).
- **Reach Apple logo then reset** → likely SMP; fall back to `cpus=1` in the
  cmdline, then bring up secondaries one by one with `cpus=2…6`.
- The historical engineering log in `NOTES.md` mostly describes `ios-sweet` on
  Redmi Note 10 Pro; do not assume those milestones reproduce on rosemary.

*Port generated from ios-sweet @ 2026-10-23 snapshot against rosemary-dump.txt
(2026-10-27). Keep device dumps, serials and Apple firmware out of git.*

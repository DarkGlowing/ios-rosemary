# USB root integration status — patch staged, unbuilt

Patch `xnu/patches/0093-rosemary-usbmsc.patch` adds `bsd/dev/rosemary_usbmsc.c`, a `bsd/conf/files` entry and a `rd=usbroot0` hook in IOKitBSDInit.cpp. It has NOT been compiled; no XNU tree/SDK locally. Controller bring-up and BOT transfer are explicit stubs returning ENXIO/EIO, so the driver cannot attach yet — deliberate: an unregistered disk is safer than a half-initialised controller.

Proven on hardware in the shim: xHCI register map (CRCR 0x18, DCBAAP 0x30, CONFIG 0x38), contexts written, endpoints configured, bulk IN/OUT completion 1, INQUIRY/CAPACITY/READ(10) OK, HFSX catalog walk, and SystemVersion.plist read from the stick (16.7.16 / 20H392).

Remaining work, in order: port bring-up (INFRA gates, IPPC power, UTMI force, port reset, Enable Slot, Address Device, Configure Endpoint); implement CBW/data/CSW with tag and signature checks plus bounded resets; verify cache maintenance and 64-bit DMA; then first CI build and fix API mismatches.

Hardware now verified: stage61 compact INQUIRY=0, READ CAPACITY=0, block size 512, READ(10) LBA2=0, all bulk residuals zero. USB storage works in the diagnostic shim. It is NOT yet available to XNU.

Current requested work: prepare iOS IPSW root on USB (currently FAT32) and implement XNU USB storage. No IPSW found under project root. No expanded XNU source/build SDK available here; only patches and Linux shim toolchain. No gh binary found. Do not claim a driver or bootable iOS image is ready.

Added tools/inspect-ipsw.py: reads BuildManifest.plist and lists identity-specific DMG component paths, presence, product/version/build. No disk writes or extraction. Need user IPSW path (outside granted project paths must be explicitly supplied).

Root preparation already exists as tools/mkiosroot.sh, using APFS-FUSE to copy OS/SystemOS/AppOS into HFS+. This host lacks apfs-fuse and mkfs.hfsplus. Verify actual IPSW identities before selecting DMGs. Current script's source paths and metadata preservation require validation on the real images. FAT32 is not the intended root filesystem. Do not dd IPSW or give an arbitrary /dev/sdX write command. Prepare and verify a regular image first; obtain exact removable disk identity and authorization before erasure.

XNU implementation requirements: native MMIO mappings and physically contiguous DMA with cache maintenance; full 64-bit physical pointers (XNU memory starts above 4 GiB); xHCI scratchpad/context sizing; proper producer Link TRB/cycle wrap and event matching; bounded resets/timeouts; descriptor-based endpoints and BOT validation/recovery; serialized BSD block strategy and ioctl interfaces; safe partition bounds; root selection rd=usbroot0. Existing 0028 UFS patch shows block-device/root hook structure but contains old platform assumptions: do not transplant its Qualcomm register initialization. Existing 0072..0081 USB code is a device-mode Qualcomm implementation and is not an MT6785 host driver. No XNU patch was added this turn because a stub or raw reuse of shim pointers would misrepresent working compatibility.

Next independent work: obtain expanded patched XNU tree/toolchain (or authenticated CI checkout), implement native driver and validate compilation before offering a kernel image. Need IPSW location and current XNU build source/artifact identity from user to prepare matching integration. USB reading alone does not establish ability to boot iOS userspace.

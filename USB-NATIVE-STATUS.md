# Native USB root — implementation and validation

Replaces incomplete 0093; source xnu/additions/rosemary_usbmsc.c must match patch payload.

Implemented: mapped GPIO28/29 bit-bang I2C (ported from shim), MT6360 watchdog/VBUS/OPA sequence, TCPC source role; actual shim PHY bits and IPPC host sequence; dynamic capability/runtime/doorbell offsets; controller halt/reset; 64-bit contiguous physical DMA pool, per-region cache maintenance, scratchpad table, disjoint input/output contexts; command/transfer Link TRBs, event cycle wrap and exact pointer/slot/endpoint matching; USB2 high-speed direct-attached device enumeration; descriptor-selected BOT interface/endpoints; CBW/data/CSW, signature/tag/residue checks; REQUEST SENSE limited retries for Unit Attention or becoming-ready; capacity and HFSX/HFS+ superfloppy header check; serialized BSD read strategy with logical-block-size ioctl; disk registered only after probe succeeds.

Validation completed locally: Clang14 isolated in work/clang-local (apt download/dpkg-deb, no system changes), real CI flags/headers adapted to Apple arm64 target; syntax + Mach-O ARM64 object generation succeeds. work/rosemary_usbmsc.arm64.o, work/usbmsc-object-build.log. All undefined object symbols resolved against user's kernel (11) Mach-O symbols. git apply --check passes on extracted buildtree. DMA interval checks and four-lap producer model pass. Not a full kernel link or hardware validation; Xcode16 CI still required.

Limits: read-only; USB2 high-speed, directly attached SCSI/BOT, LUN0, 512-byte sectors, READ CAPACITY10/READ10 only; no hubs, UAS, SuperSpeed, detach/reconnect, GPT partition selection or write support. Short transfers/transport errors stop driver, no speculative continuation/recovery. Bounds and HFS header verified before root registration. This is not proof iOS userspace boots; it needs writable-state and platform integration work beyond read-only root bring-up.

CI next: upload 0093 patch, additions source, and updated .github/workflows/xnu.yml (nm -m supports __TEXT_EXEC symbol class S). Build and provide run URL/artifact. Do not flash existing boot_b: user constraint remains test only boot_a and rollback dump/boot_a.bin. No boot args changed this turn; rd=usbroot0 selects driver only after appropriate test image prepared; do NOT enable rootrw/ufsrw for read-only driver.

No .git metadata in project checkout, so no commit made. Keep old successful diagnostic shim unchanged as reference. Older statements in handoff files about no local compiler and transport stubs are superseded by this document.

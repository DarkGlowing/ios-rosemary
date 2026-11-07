# USB-C / stage 61 compact — 2026-11-06

Hardware milestone from user photo: output contexts populated (DCtx nz=6, DCtx2 nz=14), cfg_ep=1, slot_state=3, bulk OUT/IN completion code 1, INQUIRY status 0. Capacity/read results fell below framebuffer; not yet verified. Compact rebuild skips historical IPPC/DWC3 prologue, context/register dumps and redundant descriptor lines; retains BOT completion lines and adds block size. Successful-path output is approximately 40 lines, within 49-row console. Build succeeded. New MD5 eaec0a36c461674779b960cbeb3a4dc9; same image path. No protocol changes in this compact rebuild. Next hardware test must capture cap/block_sz/rd2.

Goal: boot iOS/XNU from a USB flash drive, without modifying userdata. microSD is not part of the current task. User performs fastboot operations; do not flash or change device drivers via adb.

Stage 60 observation: MAP at28=0, at30=A5A50000, at38=38. This matches standard xHCI reserved/alignment/field masks, not a shifted wrapper. Earlier claims that CRCR=0x18 is nonstandard and that both writable offsets imply aliases were incorrect. Empty output contexts were observed, not proof of controller-internal storage.

Stage 61 changes: DCBAAP op+0x30, CONFIG op+0x38; Slot ID from event bits 31:24; correct MaxSlots/MaxPorts reporting; 32-byte-context guard; event volatile reads; input context index DCI+1; bulk DCI from endpoint descriptors (0x81 -> 3, 0x82 -> 5); Normal TRB reserved fields removed; completion endpoint/slot filtering and code logging; CSW signature/tag validation; stop BOT sequence after failure. No marker writes in this stage. Existing power/PHY path retained.

Build: bash shim/build.sh succeeded, warnings remain in legacy diagnostics (including void returns in stage13); no hardware test yet. Image shim/out/shim-usbdiag-full.img, MD5 f6400fd0c64edc3662b5e7a45e2fc0b2. Diagnostic only: INQUIRY, READ CAPACITY, READ(10) LBA2, then reset. Not yet an XNU USB root driver. Arrays support DCI up to 5, short probe only; no production ring wrap or BOT recovery. 64-byte context controllers explicitly rejected. Requires subsequent hardware log before claiming USB storage works.

Next: check DCtx nz>0, cfg_ep=1, BULK completion codes, inq=0, cap=0 with valid capacity, rd2=0. A missing HFS signature at raw LBA2 does not imply USB failure (partitioned disks differ). Then implement proper block reader and XNU handoff/root-device support.

No git metadata exists in this checkout, so no commit could be created. Latest user constraints override older NOTES about using userdata. Previously trying driver unbind over adb exceeded the requested read-only data collection; do not repeat it.

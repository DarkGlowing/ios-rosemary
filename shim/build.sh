#!/bin/bash
# Build the rosemary boot shim and wrap it in Android boot images that mirror
# the *stock* MT6785 header (see dtb/boot_unpacked/header.txt from boot_a.bin).
#
#   ./build.sh                 diagnostic image: no xnu payload, holds ~30 s
#   ./build.sh payload.bin     full image: shim loads the xnu payload
#
# Outputs (out/):
#   shim-diag.img      raw   ARM64 Image header, no payload
#   shim-diag-gz.img   gzip  same, kernel gzip'd exactly like stock boot_a
#   shim-xnu.img       raw   with payload
#   shim-xnu-gz.img    gzip  with payload
set -euo pipefail
cd "$(dirname "$0")"

CC=aarch64-linux-gnu-gcc
OBJCOPY=aarch64-linux-gnu-objcopy
NM=aarch64-linux-gnu-nm
CFLAGS="-O2 -ffreestanding -nostdlib -fno-pic -mgeneral-regs-only -mstrict-align -fno-stack-protector -Wall -Wextra"
LDFLAGS="-Wl,--build-id=none -Wl,-N"
if aarch64-linux-gnu-ld --help 2>/dev/null | grep -q -- '--no-warn-rwx-segments'; then
	LDFLAGS="$LDFLAGS -Wl,--no-warn-rwx-segments"
fi

mkdir -p out
for f in start.S shim.c console.c mmu.c fdt.c loader.c ufs.c xnulog.c platform_mt6785.c; do
	$CC $CFLAGS -c $f -o out/${f%.*}.o
done

# The image runs wherever the bootloader puts it with no relocation pass:
# every reference must be PC-relative (adr) or linker-resolved.
abs=$($CC -c -o /dev/null -x c - 2>/dev/null <<<'int x;' ; \
      aarch64-linux-gnu-readelf -r out/shim.o out/console.o out/mmu.o out/fdt.o \
	out/loader.o out/ufs.o out/xnulog.o out/platform_mt6785.o | grep -c ABS64 || true)
[ "${abs:-0}" = 0 ] || { echo "error: $abs absolute relocations in C objects" >&2; exit 1; }

$CC $CFLAGS -static -T shim.ld $LDFLAGS \
	out/start.o out/shim.o out/console.o out/mmu.o out/fdt.o out/loader.o out/ufs.o out/xnulog.o out/platform_mt6785.o \
	-o out/shim.elf
$OBJCOPY -O binary out/shim.elf out/Image

# Pad to image_size so .bss and the stacks exist as zeroed bytes in RAM.
size=$($NM out/shim.elf | python3 -c 'import sys
for l in sys.stdin:
    p=l.split()
    if len(p)==3 and p[2]=="_end": print(int(p[0],16))')
truncate -s "$size" out/Image

# ---- ramdisks -------------------------------------------------------------
# Diagnostic: an empty newc cpio. LK still patches /chosen with its address, so
# the shim sees an initrd, fails the payload magic check and holds the screen.
rm -rf out/rd && mkdir -p out/rd && (cd out/rd && find . | cpio -o -H newc --quiet | gzip -9) > out/ramdisk-diag.bin
if [ -n "${1:-}" ]; then
	cp "$1" out/ramdisk-xnu.bin
else
	: > out/ramdisk-xnu.bin
fi

# ---- boot images ----------------------------------------------------------
DTB=../dtb/boot_unpacked/dtb
[ -f "$DTB" ] || { echo "error: $DTB missing (run tools/unpack_bootv2.py boot_a.bin dtb/boot_unpacked)" >&2; exit 1; }

# Stock boot_a: kernel 0x40080000, ramdisk 0x47c80000, tags/dtb 0x4bc80000,
# page 2048, header v2, os 12.0.0 / patch 2023-10, cmdline bootopt=64S3,...
COMMON=(--header_version 2 --dtb "$DTB" --base 0
	--kernel_offset 0x40080000 --ramdisk_offset 0x47c80000
	--tags_offset 0x4bc80000 --dtb_offset 0x4bc80000 --pagesize 2048
	--os_version 12.0.0 --os_patch_level 2023-10
	--cmdline "bootopt=64S3,32N2,64N2 buildvariant=user")

gzip -9 -n -c out/Image > out/Image.gz

# GZIP kernel is the default: MTK LK on rosemary crashes (bootreason=lk_crash)
# on a raw ARM64 Image and boots a gzip'd one, exactly like the stock images.
mkbootimg "${COMMON[@]}" --kernel out/Image.gz --ramdisk out/ramdisk-diag.bin -o out/shim-diag.img
mkbootimg "${COMMON[@]}" --kernel out/Image    --ramdisk out/ramdisk-diag.bin -o out/shim-diag-raw.img
if [ -s out/ramdisk-xnu.bin ]; then
	mkbootimg "${COMMON[@]}" --kernel out/Image.gz --ramdisk out/ramdisk-xnu.bin -o out/shim-xnu.img
	mkbootimg "${COMMON[@]}" --kernel out/Image    --ramdisk out/ramdisk-xnu.bin -o out/shim-xnu-raw.img
fi

# ---- USB host probe image (no XNU payload) --------------------------------
# Same shim, built with -DSHIM_USBPROBE: it reports the DWC3 state the
# bootloader left (stage 0), forces host mode + port power (stage 1) and holds
# the result on screen for 30 s. Separate image on purpose: if a gated USB
# register access hangs the bus, nothing else is lost.
mkdir -p out/usb
for f in start.S shim.c console.c mmu.c fdt.c loader.c ufs.c xnulog.c \
	platform_mt6785.c usb.c; do
	$CC $CFLAGS -DSHIM_USBPROBE -c $f -o out/usb/${f%.*}.o
done
$CC $CFLAGS -static -T shim.ld $LDFLAGS out/usb/*.o -o out/usb/shim.elf
$OBJCOPY -O binary out/usb/shim.elf out/Image-usb
size=$($NM out/usb/shim.elf | python3 -c 'import sys
for l in sys.stdin:
    p=l.split()
    if len(p)==3 and p[2]=="_end": print(int(p[0],16))')
truncate -s "$size" out/Image-usb
gzip -9 -n -c out/Image-usb > out/Image-usb.gz
mkbootimg "${COMMON[@]}" --kernel out/Image-usb.gz --ramdisk out/ramdisk-diag.bin \
	-o out/shim-usbdiag.img

# Full-partition variants: zeros all the way to the end of the 64 MiB boot
# partition, which removes any stale AVB footer left by a previous image.
python3 - <<'EOF'
import glob, struct
PS = 67108864
for f in sorted(glob.glob('out/shim-*.img')):
    if f.endswith('-full.img'):
        continue
    d = open(f, 'rb').read()
    open(f[:-4] + '-full.img', 'wb').write(d + b'\0' * (PS - len(d)))
EOF

# mkbootimg in this toolchain ignores --os_patch_level; force the exact stock
# os_version word (12.0.0 / 2023-10) so MTK's boot-patch-level check cannot
# treat our image as a rollback.
python3 - <<'EOF'
import struct, glob
for f in sorted(glob.glob('out/shim-*.img')):
    d = bytearray(open(f, 'rb').read())
    struct.pack_into('<I', d, 44, 0x1800017a)
    open(f, 'wb').write(d)
    print(f, hex(struct.unpack_from('<I', d, 44)[0]))
EOF

# Host render of the console layout (sanity check of the drawing code).
gcc -O1 -Wall -o out/host_test console.c host_test.c && ./out/host_test >/dev/null
ls -l out/Image out/Image.gz out/shim-*.img

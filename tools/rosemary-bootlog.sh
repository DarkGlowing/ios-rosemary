#!/bin/sh
# Collect everything the phone can tell us about the *previous* (failed) boot,
# without opening it and without BROM/mtkclient.
#
#   tools/rosemary-bootlog.sh [-s SERIAL] OUTDIR
#
# Boot the phone into the *working* Android slot (slot A), enable USB debugging,
# run this. It reads boot reason, MTK's LK log (log_store -> /proc/last_kmsg),
# pstore/ramoops, dmesg and the small state partitions that LK updates on every
# boot attempt (misc BCB = slot tries/fail flags, boot_para).
set -eu

ADB="adb"
if [ "${1:-}" = "-s" ]; then
	shift
	SERIAL=${1:?Error: -s requires a device serial}
	shift
	ADB="adb -s $SERIAL"
fi
out=${1:?usage: rosemary-bootlog.sh [-s SERIAL] OUTDIR}
mkdir -p "$out"
sh_() { $ADB shell "$@" 2>/dev/null | tr -d '\r'; }
root_sh() { $ADB shell "su -c '$*'" 2>/dev/null | tr -d '\r'; }

command -v adb >/dev/null 2>&1 || { echo "adb not in PATH" >&2; exit 1; }
$ADB wait-for-device

echo "== identity" | tee "$out/00-identity.txt"
{
	sh_ getprop ro.product.device
	sh_ getprop ro.build.display.id
	sh_ getprop ro.boot.slot_suffix
	sh_ getprop ro.boot.verifiedbootstate
	sh_ getprop ro.boot.bootreason
	sh_ getprop ro.boot.boot_devices
	sh_ getprop ro.boot.hardware
} >> "$out/00-identity.txt"

# 1. Kernel command line: contains androidboot.bootreason / slot / vbmeta state.
sh_ cat /proc/cmdline > "$out/01-cmdline.txt"

# 2. The most valuable one: MTK log_store = preloader + LK log of the last boot.
sh_ cat /proc/last_kmsg > "$out/02-last_kmsg.txt" || true
root_sh "cat /proc/last_kmsg" > "$out/02-last_kmsg-root.txt" || true
[ -s "$out/02-last_kmsg-root.txt" ] || rm -f "$out/02-last_kmsg-root.txt"

# 3. ramoops / pstore (mblock-18-pstore @ 0x4d010000).
root_sh "for f in /sys/fs/pstore/*; do echo \"--- \$f\"; cat \$f; done" > "$out/03-pstore.txt" || true
[ -s "$out/03-pstore.txt" ] || rm -f "$out/03-pstore.txt"

# 4. Kernel log; boot-related lines only (full log below).
root_sh "dmesg" > "$out/04-dmesg.txt" || sh_ dmesg > "$out/04-dmesg.txt" || true
grep -i -E "wdt|watchdog|last_kmsg|log_store|mtk_boot|avb|vbmeta|slot|bootreason|panic|reset" \
	"$out/04-dmesg.txt" > "$out/04-dmesg-boot.txt" 2>/dev/null || true

# 5. State LK rewrites on every boot attempt: misc BCB (slot tries) and boot_para.
for p in misc boot_para; do
	root_sh "dd if=/dev/block/by-name/$p of=/data/local/tmp/$p.bin bs=4096 count=8 2>/dev/null; \
		ls -l /data/local/tmp/$p.bin" >/dev/null || continue
	$ADB pull /data/local/tmp/"$p".bin "$out/05-$p.bin" >/dev/null 2>&1 || true
	root_sh "rm -f /data/local/tmp/$p.bin" >/dev/null || true
done

# 6. expdb: MTK exception/boot dump partition (20 MiB). Contains LK/AEE records.
root_sh "dd if=/dev/block/by-name/expdb of=/data/local/tmp/expdb.bin bs=1M 2>/dev/null" >/dev/null || true
$ADB pull /data/local/tmp/expdb.bin "$out/06-expdb.bin" >/dev/null 2>&1 || true
root_sh "rm -f /data/local/tmp/expdb.bin" >/dev/null || true

echo
echo "collected into $out:"
ls -1 "$out"
echo
echo "quick look:"
echo "  bootreason : $(sh_ getprop ro.boot.bootreason)"
echo "  slot       : $(sh_ getprop ro.boot.slot_suffix)"
echo "  vbstate    : $(sh_ getprop ro.boot.verifiedbootstate)"
grep -a -i -E "wdt|avb|vbmeta|boot image|decompress|reset" "$out/02-last_kmsg-root.txt" 2>/dev/null | tail -20 || true

#!/usr/bin/env python3
"""Decode the copies shim/xnulog.c leaves on storage.

    for p in expdb gsort ffu blk1 md_udc; do
      adb shell "dd if=/dev/block/by-name/$p of=/sdcard/$p.bin bs=4096 count=16"
      adb pull /sdcard/$p.bin ~/
    done
    python3 tools/xnulog-read.py ~/expdb.bin ~/gsort.bin ~/ffu.bin ~/blk1.bin ~/md_udc.bin

Each target partition gets two things, 8 blocks apart:
  * LBA+0  "SHIMBOOTn" + found/len/phys_base/span, one record per boot, so an
    empty partition can be told apart from "the shim never ran again";
  * LBA+8  the XNU log: "RXNULOG1", LE32 length, text at +16.
Only the first 64 KiB of each partition is needed.
"""
import struct
import sys

BOOT_TAG = b'SHIMBOO'
LOG_MAGIC = b'RXNULOG1'


def show_boot_records(name, data):
    off = 0
    seen = False
    while True:
        off = data.find(BOOT_TAG, off)
        if off < 0:
            break
        seen = True
        end = data.find(b'\x00', off)
        rec = data[off:end].decode('latin1', 'replace')
        print('  %-8s boot record at %#x: %s' % (name, off, rec.replace('\n', ' | ')))
        off += 1
    if not seen:
        print('  %-8s no boot record (shim did not run, or partition was erased)' % name)
    return seen


def try_log(data):
    off = data.find(LOG_MAGIC)
    if off < 0:
        return None
    (length,) = struct.unpack_from('<I', data, off + 8)
    if length > 64 * 1024:
        return None
    return data[off + 16: off + 16 + length].decode('latin1')


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        return 2

    found = None
    for path in paths:
        data = open(path, 'rb').read()
        name = path.rsplit('/', 1)[-1]
        print('%s (%d bytes)' % (name, len(data)))
        if show_boot_records(name, data):
            pass
        text = try_log(data)
        if text is not None and found is None:
            found = (name, text)
    print()
    if found is None:
        print('!! no RXNULOG1 payload in any of the given files')
        return 1
    name, text = found
    print('=== XNU log from %s ===' % name)
    print('-' * 72)
    sys.stdout.write(text)
    if not text.endswith('\n'):
        print()
    print('-' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())

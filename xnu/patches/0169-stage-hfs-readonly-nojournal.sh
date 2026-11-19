#!/bin/bash
# dxb runs patches/*.sh from the XNU source root, after 0026 has staged HFS.
#
# The iOS root on the USB stick is mounted read-only (the host driver refuses
# writes), but HFS still runs late journal initialisation for it and that path
# writes: journal_create() rewrites the journal header and the journal info
# block. On our block device every write returns EROFS, the call fails with
# EINVAL, and the whole mount is refused:
#
#   hfs: Initializing the journal (joffset 0x82000 sz 0x0)...
#   hfs: late_jnl init: failed to open/create the journal (retval 0)
#   hfs_mountfs: encountered failure 22
#   cannot mount root, errno = 19
#
# hfs_late_journal_init() already declines to check anything for a read-only
# mount that is not the root (it only calls journal_is_clean() there), so a
# read-only root is simply missing from that decision. Skip journal setup for
# read-only mounts entirely: nothing in this boot writes to the volume, and
# the branch right after the condition clears kHFSVolumeJournaledMask for
# exactly this case.
set -euo pipefail

[[ "${MACHINE_CONFIG:-}" == ROSEMARY ]] || exit 0
[[ -f bsd/sweet_hfs/core/hfs_vfsutils.c ]] || {
    echo 'HFS sources were not staged by 0026-stage-hfs.sh' >&2
    exit 1
}

python3 - <<'PY'
from pathlib import Path

path = Path('bsd/sweet_hfs/core/hfs_vfsutils.c')
text = path.read_text()

old = """\tif (   (vcb->vcbAtrb & kHFSVolumeJournaledMask)
\t\t&& (SWAP_BE32(vhp->lastMountedVersion) != kHFSJMountVersion)
\t\t&& (hfsmp->jnl == NULL)) {"""

new = """\tif (   (vcb->vcbAtrb & kHFSVolumeJournaledMask)
\t\t&& (SWAP_BE32(vhp->lastMountedVersion) != kHFSJMountVersion)
\t\t&& ((hfsmp->hfs_flags & HFS_READ_ONLY) == 0)
\t\t&& (hfsmp->jnl == NULL)) {"""

if text.count(old) != 1:
    raise SystemExit('HFS source changed: journal init condition not found once')

path.write_text(text.replace(old, new, 1))
print('hfs: journal init skipped on read-only mounts (rosemary usb root)')
PY

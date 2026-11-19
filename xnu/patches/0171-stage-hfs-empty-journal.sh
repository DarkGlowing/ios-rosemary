#!/bin/bash
# dxb runs patches/*.sh from the XNU source root, after 0026 has staged HFS.
#
# The iOS root image on the USB stick carries a .journal_info_block whose
# journal size is 0 (the image was assembled on Linux, where the journal was
# never written). HFS+ still marks the volume as journaled, so the mount path
# runs late journal initialisation and calls journal_create(), which rejects
# anything below 256 KiB:
#
#   hfs: Initializing the journal (joffset 0x82000 sz 0x0)...
#   jnl: <dev>: create: journal size 0 looks bogus.
#   jnl: b(1, 0): journal_create: failed to write journal header.
#
# Nothing can be created out of a zero-length journal, and the volume mounts
# fine without journaling (the other failure branches in this function do
# exactly that: clear kHFSVolumeJournaledMask and carry on). So bail out
# early when the recorded journal size is zero, before any of the create and
# header-write paths are entered.
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

old = """\tjib_flags  = SWAP_BE32(jibp->flags);
\tjib_size   = SWAP_BE64(jibp->size);
"""

new = """\tjib_flags  = SWAP_BE32(jibp->flags);
\tjib_size   = SWAP_BE64(jibp->size);

\t/* A journaled volume whose journal has no size cannot be recreated:
\t * journal_create() refuses anything under 256 KiB. Mount without
\t * journaling, the same way the other bail-out branches below do. */
\tif (jib_size == 0) {
\t\tprintf("hfs: journal size is 0, disabling journaling.\\n");
\t\tvcb->vcbAtrb &= ~kHFSVolumeJournaledMask;
\t\tbuf_brelse(jinfo_bp);
\t\treturn 0;
\t}
"""

if text.count(old) != 1:
    raise SystemExit('HFS source changed: journal size read not found once')

path.write_text(text.replace(old, new, 1))
print('hfs: zero-length journal is skipped instead of recreated')
PY

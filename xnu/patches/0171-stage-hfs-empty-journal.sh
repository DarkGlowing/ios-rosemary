#!/bin/bash
# dxb runs patches/*.sh from the XNU source root, after 0026 has staged HFS.
#
# The iOS root image on the USB stick carries a .journal_info_block whose
# journal size is 0 (the image was assembled on Linux, where the journal was
# never written). The volume is still flagged journaled and its
# lastMountedVersion is not kHFSJMountVersion, so hfs_MountHFSPlusVolume()
# takes the *late* journal path, hfs_late_journal_init(), which tries to
# recreate the journal:
#
#   hfs: Initializing the journal (joffset 0x82000 sz 0x0)...
#   jnl: b(1, 0): do_jnl_io: strategy err 0x6
#   jnl: b(1, 0): write_journal_header: error writing the journal header
#   hfs: late_jnl init: failed to open/create the journal (retval 0)
#   hfs_mountfs: encountered failure 22
#
# journal_create() refuses anything under 256 KiB, so a zero-length journal
# can never be recreated. Bail out as soon as the size is read, exactly the
# way the neighbouring .journal and info-block failures do: release the
# buffer, clear kHFSVolumeJournaledMask and return 0, which leaves the
# volume mounted without journaling.
#
# The anchor is the GetFileInfo() call for .journal, which exists only in
# hfs_late_journal_init(); hfs_early_journal_init() reads the same two fields
# but has no vcb in scope (the first version of this script patched that copy
# instead and did not compile).
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
\tjib_offset = SWAP_BE64(jibp->offset);
\tjib_size   = SWAP_BE64(jibp->size);

\tfid = GetFileInfo(vcb, kRootDirID, \".journal\", &jattr, &jfork);
"""

new = """\tjib_flags  = SWAP_BE32(jibp->flags);
\tjib_offset = SWAP_BE64(jibp->offset);
\tjib_size   = SWAP_BE64(jibp->size);

\t/* A journaled volume whose journal has no size cannot be recreated:
\t * journal_create() refuses anything under 256 KiB. Mount without
\t * journaling, the same way the failures below do. */
\tif (jib_size == 0) {
\t\tprintf(\"hfs: journal size is 0, disabling journaling.\\n\");
\t\tbuf_brelse(jinfo_bp);
\t\tvcb->vcbAtrb &= ~kHFSVolumeJournaledMask;
\t\treturn 0;
\t}

\tfid = GetFileInfo(vcb, kRootDirID, \".journal\", &jattr, &jfork);
"""

if text.count(old) != 1:
    raise SystemExit('HFS source changed: late journal_info_block read not found once')

path.write_text(text.replace(old, new, 1))
print('hfs: zero-length journal is skipped in hfs_late_journal_init')
PY

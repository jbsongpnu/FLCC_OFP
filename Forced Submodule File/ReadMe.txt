PNU-KAL MAVLink dialect
=======================

pnu_kal.xml holds the 7 PNU-KAL HD ICD message definitions
(id 50001, 50002, 50004, 51001, 51002, 51003, 51005).  It IS tracked by git.

The modules/mavlink submodule is left COMPLETELY PRISTINE.  Nothing inside it is
edited - the parent repo cannot track submodule file contents, so any edit there is
invisible to git and is silently reverted by "git submodule update", a fresh clone or
a submodule bump.  (That is the submodule boundary, not .gitignore.)

Instead the definitions are staged outside the submodule and the build runs from the
staged copy:

    Tools/pnu/stage_mavlink_defs.sh            stage (destructive rebuild, idempotent)
    Tools/pnu/stage_mavlink_defs.sh --check    report only; exit 1 if missing or stale

What it does:

  1. copies modules/mavlink/message_definitions/v1.0/*.xml  (19 files, ~944 KB)
     into Tools/pnu/mavlink_defs/
  2. copies this folder's pnu_kal.xml in beside them
  3. in the staged all.xml and ardupilotmega.xml, replaces
       <include>cubepilot.xml</include>
     with a marker comment, and in ardupilotmega.xml also adds
       <include>pnu_kal.xml</include>
  4. writes the submodule commit it staged from to .source-commit

cubepilot.xml is simply not staged.  Its ids 50001-50005 collide with the PNU-KAL ICD,
and nothing in ArduPilot references CUBEPILOT_* or HERELINK_* - PNU uses no Herelink
and no CubePilot raw RC.  It must be dropped from BOTH files: all.xml is the build
root, and ardupilotmega.xml includes cubepilot.xml directly as well.

Tools/pnu/mavlink_defs/ is GENERATED and gitignored - never commit it.  Committing it
would put a ~944 KB duplicate of upstream in the repo that rots on every ArduPilot
bump, which is exactly the problem this replaced.

wscript builds from Tools/pnu/mavlink_defs/all.xml instead of the submodule path.
That one line is the only change to an ArduPilot-tracked file, and being tracked it
cannot vanish silently the way a submodule edit does.

RE-RUN THE SCRIPT AFTER ANY modules/mavlink CHANGE.  A stale stage still contains the
PNU messages, so it builds happily against last month's upstream definitions.
--check compares .source-commit against the submodule HEAD and catches exactly that.

Three failure paths, all reported clearly:
  - nothing staged      -> waf stops with "Tools/pnu/mavlink_defs/all.xml is missing"
  - stale stage         -> stage_mavlink_defs.sh --check exits 1
  - staged without PNU  -> compile stops at GCS.h with "PNU-KAL MAVLink dialect missing"

History: these messages used to be appended to the submodule's common.xml, which meant
keeping a full 7579-line copy of common.xml here and overwriting the upstream file.
That silently reverted upstream common.xml changes on every ArduPilot bump.  A later
step cut it to three lines inside the submodule; this removes the last of them.
See README PNU-ISSUE D3.

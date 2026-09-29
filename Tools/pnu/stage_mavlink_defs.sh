#!/usr/bin/env bash
#
# Stage the MAVLink message definitions used to build the PNU-KAL firmware.
#
# The modules/mavlink submodule is left completely pristine.  Its message definitions
# are copied into Tools/pnu/mavlink_defs/, the PNU-KAL dialect is added, and
# cubepilot.xml is dropped (its ids 50001-50005 collide with the PNU-KAL ICD, and
# nothing in ArduPilot references CUBEPILOT_* or HERELINK_*).  wscript builds from the
# staged copy.  See README PNU-ISSUE D3.
#
# The staging directory is generated and gitignored - never commit it.  Only this
# script and "Forced Submodule File/pnu_kal.xml" are tracked.
#
# Re-run after any change to the modules/mavlink submodule.  The staged copy records
# the submodule commit it came from, so --check detects a stale stage.
#
# Usage:
#   Tools/pnu/stage_mavlink_defs.sh            stage (destructive rebuild, idempotent)
#   Tools/pnu/stage_mavlink_defs.sh --check    report only; exit 1 if missing or stale

set -eu

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$ROOT/modules/mavlink/message_definitions/v1.0"
DST="$ROOT/Tools/pnu/mavlink_defs"
DIALECT="$ROOT/Forced Submodule File/pnu_kal.xml"
STAMP="$DST/.source-commit"

CUBE='  <include>cubepilot.xml</include>'
MARK='  <!-- PNU: cubepilot.xml not staged, ids 50001-50005 collide with the PNU-KAL ICD -->'
PNUI='  <include>pnu_kal.xml</include>'

say() { printf '%s\n' "$*"; }

[ -d "$SRC" ] || { say "ERROR: $SRC missing - run 'git submodule update --init --recursive'"; exit 2; }
[ -f "$DIALECT" ] || { say "ERROR: $DIALECT missing"; exit 2; }

head_sha="$(git -C "$ROOT/modules/mavlink" rev-parse HEAD)"

if [ "${1:-}" = "--check" ]; then
    if [ ! -f "$STAMP" ]; then
        say "MAVLink defs NOT staged - run Tools/pnu/stage_mavlink_defs.sh"
        exit 1
    fi
    staged_sha="$(cat "$STAMP")"
    if [ "$staged_sha" != "$head_sha" ]; then
        say "MAVLink defs are STALE"
        say "  staged from : $staged_sha"
        say "  submodule at: $head_sha"
        say "Re-run Tools/pnu/stage_mavlink_defs.sh"
        exit 1
    fi
    if grep -qF "$CUBE" "$DST/all.xml" "$DST/ardupilotmega.xml" 2>/dev/null; then
        say "MAVLink defs staged but cubepilot.xml still included - re-run the script"
        exit 1
    fi
    if ! grep -qF "$PNUI" "$DST/ardupilotmega.xml" 2>/dev/null; then
        say "MAVLink defs staged but the PNU dialect is missing - re-run the script"
        exit 1
    fi
    say "MAVLink defs staged and current ($head_sha)"
    exit 0
fi

if ! git -C "$ROOT/modules/mavlink" diff --quiet; then
    say "WARNING: modules/mavlink has uncommitted changes; staging from the working tree."
    say "         The submodule is expected to be pristine - check 'git -C modules/mavlink status'."
fi

rm -rf "$DST"
mkdir -p "$DST"
cp "$SRC"/*.xml "$DST/"
cp "$DIALECT" "$DST/pnu_kal.xml"

# drop cubepilot from the build root and from the ArduPilot dialect, and pull in PNU-KAL
python3 - "$DST" "$CUBE" "$MARK" "$PNUI" <<'PY'
import io, os, sys
dst, cube, mark, pnui = sys.argv[1:5]
for name, repl in (("all.xml", mark + "\n"),
                   ("ardupilotmega.xml", mark + "\n" + pnui + "\n")):
    path = os.path.join(dst, name)
    s = io.open(path, encoding='utf-8').read()
    if cube + "\n" not in s:
        raise SystemExit("ERROR: %s has no cubepilot include - upstream layout changed" % name)
    io.open(path, 'w', encoding='utf-8').write(s.replace(cube + "\n", repl, 1))
PY

printf '%s\n' "$head_sha" > "$STAMP"
say "Staged $(ls "$DST"/*.xml | wc -l) definition files from $head_sha"
say "  -> $DST"

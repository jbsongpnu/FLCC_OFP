#!/bin/bash
#
# PNU : run the fake PMU, the fake gimbal and the fake SAR together on one vcan0,
# with SITL, as CAN_D1_PNU_OPTS = 7 expects them.
#
# WHY THIS EXISTS.  The three fakes are easy to start by hand - three terminals and a
# fourth for SITL - but the order is not obvious and getting it wrong fails quietly.
# The CAN driver attaches to the bus during boot and the gimbal driver sends its enable
# within the first few hundred milliseconds, so a fake that starts after SITL misses the
# conversation that was meant to bring it up, and the gimbal sits in INIT until it
# faults.  This script starts the fakes first, every time.
#
# It also keeps each fake's console reachable while it runs, through a FIFO per device:
#
#     Tools/pnu/sitl_can/run_all3.sh say gimbal fault 0 2
#     Tools/pnu/sitl_can/run_all3.sh say sar silent
#     Tools/pnu/sitl_can/run_all3.sh say pmu status
#
# and, at teardown, asks every device for a final status so the logs end with the frame
# counts rather than with whatever happened to be last.
#
# NOTHING HERE IS A MEASUREMENT OF THE REAL DEVICES.  In particular --fb-mode defaults
# to periodic because the driver cannot survive per-command feedback yet; see the fake
# gimbal's header and README.md.  A green run here says the firmware is self-consistent,
# not that the hardware behaves this way.

set -u

IFACE=vcan0
OUT=/tmp/pnu_all3
OPTS=7
GIMBAL_HOME=0
FB_MODE=periodic
FB_RATE=50
HOME_LOC=35.2345,129.0789,10,0
SPEEDUP=1
WIPE=1
DURATION=0
SCENARIO=
EXTRA_PARAM=
PMU_MAVLINK=tcp:127.0.0.1:5763
TAP=1
NO_ROOT=0
VERBOSE=0

usage() {
    cat <<'ENDOFUSAGE'
usage: run_all3.sh [options]
       run_all3.sh say {gimbal|sar|pmu} <console command ...>

Starts the three fakes on one vcan0, then SITL, then holds until Ctrl-C.

  --out DIR         logs, FIFOs and the generated parameters (default /tmp/pnu_all3).
                    Its *.log are replaced on every run; pass a new one to keep a run
  --opts N          CAN_D1_PNU_OPTS (default 7 = PMU + gimbal + SAR)
  --gimbal-home     CAN_D1_PNU_HOME 1, run the Appendix 1 homing sequence (~40 s)
  --fb-mode M       gimbal feedback trigger: periodic|per-command|off (default periodic)
  --fb-rate HZ      feedback rate for --fb-mode periodic (default 50)
  --home L          SITL home as lat,lng,alt,hdg (default 35.2345,129.0789,10,0)
  --speedup X       SITL speedup (default 1; the fakes run in real time, so leave it)
  --no-wipe         keep the existing eeprom.bin instead of starting from the defaults
  --duration N      tear down automatically after N seconds instead of holding
  --scenario FILE   run FILE after boot, then tear down.  It is given four arguments:
                    <mavlink url> <gimbal fifo> <sar fifo> <pmu fifo>
  --param FILE      extra parameter file, appended after the generated one
  --pmu-mavlink U   where the fake PMU watches for takeoff (default SERIAL2, or "none")
  --no-tap          do not log the FC's statustexts and TM50 on SERIAL5
  --interface NAME  SocketCAN interface (default vcan0; SITL hard-codes vcan0)
  --no-root         create the bus in a private user namespace instead of needing root.
                    Everything then lives inside that namespace, so no ground station
                    outside it can connect - useful only with --scenario or --duration.
  --verbose         pass --verbose to all three fakes
ENDOFUSAGE
}

# ---- say: talk to a running instance's console -------------------------------------
if [ "${1:-}" = "say" ]; then
    shift
    if [ "${1:-}" = "--out" ]; then OUT=$2; shift 2; fi
    DEV=${1:-}
    shift || true
    case "$DEV" in
        gimbal|sar|pmu) ;;
        *) echo "say: device must be gimbal, sar or pmu" >&2; exit 2 ;;
    esac
    FIFO=$OUT/$DEV.fifo
    if [ ! -p "$FIFO" ]; then
        echo "say: $FIFO is not there - is run_all3.sh running with --out $OUT?" >&2
        exit 1
    fi
    echo "$*" > "$FIFO"
    exit 0
fi

ORIG_ARGS=("$@")
while [ $# -gt 0 ]; do
    case "$1" in
        --out)          OUT=$2; shift 2 ;;
        --opts)         OPTS=$2; shift 2 ;;
        --gimbal-home)  GIMBAL_HOME=1; shift ;;
        --fb-mode)      FB_MODE=$2; shift 2 ;;
        --fb-rate)      FB_RATE=$2; shift 2 ;;
        --home)         HOME_LOC=$2; shift 2 ;;
        --speedup)      SPEEDUP=$2; shift 2 ;;
        --no-wipe)      WIPE=0; shift ;;
        --duration)     DURATION=$2; shift 2 ;;
        --scenario)     SCENARIO=$2; shift 2 ;;
        --param)        EXTRA_PARAM=$2; shift 2 ;;
        --pmu-mavlink)  PMU_MAVLINK=$2; shift 2 ;;
        --no-tap)       TAP=0; shift ;;
        --interface)    IFACE=$2; shift 2 ;;
        --no-root)      NO_ROOT=1; shift ;;
        --verbose)      VERBOSE=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "unknown option $1" >&2; usage >&2; exit 2 ;;
    esac
done

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd) || exit 1
TOOLS=$REPO/Tools/pnu/sitl_can

# ---- the private namespace, if asked for -------------------------------------------
# SITL calls exit(1) when its servo multicast socket cannot connect (SITL_State.cpp),
# and a fresh namespace has no route for 239.x, so the route is not optional.
if [ $NO_ROOT -eq 1 ] && [ "${PNU_ALL3_NS:-}" != "1" ]; then
    exec unshare -rn env PNU_ALL3_NS=1 bash -c '
        ip link set lo up
        ip route add 224.0.0.0/4 dev lo
        ip link add '"$IFACE"' type vcan && ip link set '"$IFACE"' up
        exec "$@"' -- "${BASH_SOURCE[0]}" ${ORIG_ARGS[@]+"${ORIG_ARGS[@]}"}
fi

# ---- the bus ------------------------------------------------------------------------
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    ip link add "$IFACE" type vcan >/dev/null 2>&1
    ip link set "$IFACE" up >/dev/null 2>&1
fi
if ! ip link show "$IFACE" 2>/dev/null | grep -q 'UP'; then
    cat >&2 <<ENDOFBUS
$IFACE is not up.  Once per boot, as root:

    sudo modprobe vcan
    sudo ip link add $IFACE type vcan && sudo ip link set $IFACE up

or re-run with --no-root to build it inside a private user namespace.
ENDOFBUS
    exit 1
fi

# ---- the binary ---------------------------------------------------------------------
BIN=$REPO/build/sitl/bin/arducopter
if [ ! -x "$BIN" ]; then
    echo "no SITL binary at $BIN - ./waf configure --board sitl && ./waf copter" >&2
    exit 1
fi
NEWER=$(find "$REPO"/libraries/AP_PNU_CAN "$REPO"/libraries/AP_PNUSAR \
             "$REPO"/libraries/AP_PNUGimbal "$REPO"/libraries/AP_PMUCAN \
             -name '*.cpp' -o -name '*.h' 2>/dev/null | \
        while read -r f; do [ "$f" -nt "$BIN" ] && echo "$f"; done | head -3)
if [ -n "$NEWER" ]; then
    echo "WARNING: the SITL binary is older than PNU sources, e.g." >&2
    echo "$NEWER" | sed 's,^,    ,' >&2
    echo "         ./waf copter  (and check the board is still sitl)" >&2
fi

mkdir -p "$OUT" || exit 1
rm -f "$OUT"/*.log "$OUT"/*.fifo

# ---- the parameters -----------------------------------------------------------------
PARM=$OUT/pnu_all3.parm
cat > "$PARM" <<ENDOFPARM
SIM_CAN_TYPE1    2
CAN_P1_DRIVER    1
CAN_D1_PROTOCOL  15
CAN_D1_PNU_OPTS  $OPTS
CAN_D1_PNU_HOME  $GIMBAL_HOME
CAN_D1_PNU_MODE  0
CAN_D1_PNU_MSTID 0
MNT1_TYPE        15
LOG_DISARMED     1
SERIAL5_PROTOCOL 2
ENDOFPARM
DEFAULTS=$REPO/Tools/autotest/default_params/copter.parm,$PARM
if [ -n "$EXTRA_PARAM" ]; then
    DEFAULTS=$DEFAULTS,$EXTRA_PARAM
fi

VFLAG=
if [ $VERBOSE -eq 1 ]; then VFLAG=--verbose; fi

# a wall-clock prefix: each fake stamps its own lines from its own start, which is no
# use for lining three logs up against each other or against SITL
stamp() {
    python3 -I -u -c 'import sys, time
for line in sys.stdin:
    sys.stdout.write("%.3f %s" % (time.time(), line))'
}

# ---- teardown ------------------------------------------------------------------------
SITL_PID=
TORN=0
teardown() {
    [ $TORN -eq 1 ] && return
    TORN=1
    trap '' INT TERM
    echo
    echo "--- tearing down, asking each device for a final status ---"
    for dev in gimbal sar pmu; do
        [ -p "$OUT/$dev.fifo" ] && echo status > "$OUT/$dev.fifo"
    done
    sleep 1
    [ -n "$SITL_PID" ] && kill "$SITL_PID" 2>/dev/null
    pkill -f "$TOOLS/fake_gimbal.py" 2>/dev/null
    pkill -f "$TOOLS/fake_sar.py" 2>/dev/null
    pkill -f "$TOOLS/fake_pmu.py" 2>/dev/null
    pkill -f "$TOOLS/mav_tap.py" 2>/dev/null
    sleep 1
    [ -n "$SITL_PID" ] && kill -9 "$SITL_PID" 2>/dev/null
    # release the FIFO holders
    exec 7>&-
    exec 8>&-
    exec 9>&-
    rm -f "$OUT"/*.fifo
    echo
    echo "--- last status from each device ---"
    for dev in gimbal sar pmu; do
        printf '%-7s %s\n' "$dev" "$(grep -c . "$OUT/$dev.log" 2>/dev/null) lines"
        tail -4 "$OUT/$dev.log" 2>/dev/null | sed 's,^,        ,'
    done
    echo
    if [ -s "$OUT/statustext.log" ]; then
        echo "--- what the FC said ---"
        grep -E 'PNU|SAR|PMU|Mount|imbal|PreArm|TM50|ERROR|CRIT|EMERG' \
             "$OUT/statustext.log" | sed 's,^,    ,'
        echo
    fi
    echo "logs in $OUT :  $(cd "$OUT" && echo *.log)"
    echo "parameters   :  $PARM"
}
trap 'teardown; exit 130' INT TERM

# ---- the fakes, FIRST ------------------------------------------------------------------
# Each console is a FIFO.  The launcher holds it open read-write so that a `say` writer
# coming and going never looks like EOF to the fake - and the fakes read stdin to EOF,
# so a FIFO with no holder would close their console the instant it opened.
for dev in gimbal sar pmu; do mkfifo "$OUT/$dev.fifo" || exit 1; done
exec 7<>"$OUT/gimbal.fifo"
exec 8<>"$OUT/sar.fifo"
exec 9<>"$OUT/pmu.fifo"

echo "--- starting the fakes on $IFACE ---"
"$TOOLS/fake_gimbal.py" --interface "$IFACE" --fb-mode "$FB_MODE" --fb-rate "$FB_RATE" \
    $VFLAG < "$OUT/gimbal.fifo" 2>&1 | stamp > "$OUT/gimbal.log" &
"$TOOLS/fake_sar.py" --interface "$IFACE" --counter 777000 \
    $VFLAG < "$OUT/sar.fifo" 2>&1 | stamp > "$OUT/sar.log" &
"$TOOLS/fake_pmu.py" --interface "$IFACE" --mavlink "$PMU_MAVLINK" \
    $VFLAG < "$OUT/pmu.fifo" 2>&1 | stamp > "$OUT/pmu.log" &

# wait for all three to have bound their socket, rather than guessing with a sleep
for _ in $(seq 1 50); do
    READY=0
    for dev in gimbal sar pmu; do
        grep -q . "$OUT/$dev.log" 2>/dev/null && READY=$((READY + 1))
    done
    [ $READY -eq 3 ] && break
    sleep 0.2
done
if [ "${READY:-0}" -ne 3 ]; then
    echo "a fake did not start - see $OUT/*.log" >&2
    teardown
    exit 1
fi
echo "    gimbal, SAR and PMU are on the bus"

# ---- SITL, SECOND -----------------------------------------------------------------------
WIPEFLAG=
if [ $WIPE -eq 1 ]; then WIPEFLAG=-w; fi
# The tap goes up BEFORE SITL and retries until the port answers.  Started afterwards it
# attaches a few hundred milliseconds late and misses the gimbal's "roll enabled" lines,
# which is exactly the window it exists to cover.
if [ $TAP -eq 1 ]; then
    "$TOOLS/mav_tap.py" tcp:127.0.0.1:5765 > "$OUT/statustext.log" 2>&1 &
fi

echo "--- starting SITL ---"
# run from the output directory: SITL drops eeprom.bin and its dataflash logs in the
# working directory, and they belong with the run, not in the source tree
cd "$OUT" || exit 1
# --serial0 tcp:0 instead of the default tcp:0:wait, so that SITL finishes booting with
# no ground station attached and all three ports open at once
# SERIAL5 is unused on Copter, so the tap gets a port of its own and leaves SERIAL0
# for Mission Planner, SERIAL1 for KGCS and SERIAL2 for the PMU's takeoff watcher
"$BIN" $WIPEFLAG -S -I0 --model + --speedup "$SPEEDUP" --serial0 tcp:0 --serial5 tcp:5 \
    --defaults "$DEFAULTS" --home "$HOME_LOC" > "$OUT/sitl.log" 2>&1 &
SITL_PID=$!

for _ in $(seq 1 100); do
    (exec 3<>/dev/tcp/127.0.0.1/5760) 2>/dev/null && break
    kill -0 "$SITL_PID" 2>/dev/null || { echo "SITL exited - see $OUT/sitl.log" >&2; teardown; exit 1; }
    sleep 0.2
done
exec 3>&-

SAY="Tools/pnu/sitl_can/run_all3.sh say"
if [ "$OUT" != /tmp/pnu_all3 ]; then
    SAY="$SAY --out $OUT"
fi

cat <<ENDOFREADY

    SERIAL0  tcp:127.0.0.1:5760   Mission Planner
    SERIAL1  tcp:127.0.0.1:5762   KGCS
    SERIAL2  tcp:127.0.0.1:5763   the fake PMU's takeoff watcher
    SERIAL5  tcp:127.0.0.1:5765   mav_tap.py -> statustext.log

    expect "PNU_CAN: CAN_D1 PMU+Gimbal+SAR ready" within a second of connecting.
    consoles:  $SAY gimbal status
               $SAY sar silent
               $SAY pmu status
    logs:      tail -f $OUT/gimbal.log

ENDOFREADY

# ---- hold, time out, or run the scenario --------------------------------------------------
if [ -n "$SCENARIO" ]; then
    echo "--- scenario: $SCENARIO ---"
    "$SCENARIO" tcp:127.0.0.1:5762 "$OUT/gimbal.fifo" "$OUT/sar.fifo" "$OUT/pmu.fifo" \
        2>&1 | stamp | tee "$OUT/scenario.log"
elif [ "$DURATION" -gt 0 ]; then
    echo "--- holding for ${DURATION} s ---"
    sleep "$DURATION"
else
    echo "--- holding.  Ctrl-C to tear down ---"
    while kill -0 "$SITL_PID" 2>/dev/null; do sleep 1; done
    echo "SITL exited on its own - see $OUT/sitl.log"
fi

teardown

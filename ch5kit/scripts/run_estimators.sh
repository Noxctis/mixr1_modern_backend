#!/usr/bin/env bash
# ONE command that collects all the DCT-versus-CET data by itself.
#   bash ch5kit/scripts/run_estimators.sh            (full: about 2 hours)
#   REPS=1 bash ch5kit/scripts/run_estimators.sh     (quick test first: about 40 minutes)
# Run it from your firmware repo folder (the one that contains build/). Resumable: run it again and it skips finished runs.
set -u
KIT="$(cd "$(dirname "$0")/.." && pwd)"
# go to the firmware folder (the kit may live inside it)
if [ ! -f kinematics.hpp ] && [ ! -f include/kinematics.hpp ]; then cd "$KIT/.." || exit 1; fi
OUT="${OUT:-logs/ch5_new/estimators}"; REPS="${REPS:-3}"
VARIANTS="${VARIANTS:-dct5 dct10 dct20 aligned newest}"     # quick test:  VARIANTS="dct20 aligned newest" REPS=2
SKIP_IMC="${SKIP_IMC:-0}"                                    # 1 = skip the closed-loop runs with the new gains
mkdir -p "$OUT"
if [ -z "${DBIN:-}" ]; then for c in ./mixr1_daemon ./build/mixr1_daemon; do [ -x "$c" ] && { DBIN="$c"; break; }; done; fi
[ -x "${DBIN:-/nonexistent}" ] || { echo "ERROR: I cannot find mixr1_daemon (looked in . and ./build). Run this from your firmware folder after building."; exit 1; }
DAEMON="${DAEMON:-sudo $DBIN}"
grep -rq ESTIMATOR_DCT include/config.hpp config.hpp 2>/dev/null || { echo "ERROR: the DCT estimator is not installed. Run:  python3 ch5kit/firmware/install_dct.py   then rebuild."; exit 1; }

flags_for() { case "$1" in
  dct5) echo "--estimator=dct --window=5000";;  dct10) echo "--estimator=dct --window=10000";;  dct20) echo "--estimator=dct --window=20000";;
  aligned) echo "--estimator=aligned";;  newest) echo "--estimator=newest";; esac; }
COMMON="--test --cpr=1024 --fifo --no-stream --no-torque --wait-rest=30"
SCHED_B="${SCHED_B:-10,25,50,100,250,350,500,750,1000,1250,1500}"
HOLD_A="${HOLD_A:-10}"; HOLD_B="${HOLD_B:-15}"; HOLD_P="${HOLD_P:-12}"
SCHED_A="0,10,20,30,40,50,60,70,80,90"
plat() { echo "$(date +%H:%M:%S) $1 $(vcgencmd measure_temp 2>/dev/null) $(vcgencmd get_throttled 2>/dev/null)" >> "$OUT/platform.log"; }

echo "=============================================================="
echo " BEFORE WE START (read, do these, then press Enter)"
echo "  1. Encoder DIP switch at 0011 (256 PPR).  Nothing touching the shaft."
echo "  2. Take the impeller OFF and no water (same as your earlier cet_/dct_ tests): the motor must run free."
echo "  3. Do NOT open the dashboard.  Do not touch the Pi for the next 2 hours."
echo "=============================================================="
read -r -p "Press Enter to begin (Ctrl-C to cancel) " _

echo; echo "STEP 1 of 4: checking the encoder (10 seconds)"
$DAEMON $COMMON --no-pi --pwm=614 --duration=8 --csv="$OUT/preflight.csv" > "$OUT/preflight.stdout.txt" 2>&1
python3 "$KIT/scripts/check_preflight.py" "$OUT/preflight.stdout.txt" || { echo "Fix that, then run this script again."; exit 1; }

echo; echo "STEP 2 of 4: measuring your motor and working out the controller gains (about 3 minutes)"
if [ ! -s "$OUT/plant_sweep.csv" ]; then
  plat plant_start
  $DAEMON $COMMON --no-pi --schedule=$SCHED_A --hold=$HOLD_P --abort-rpm=3200 --csv="$OUT/plant_sweep.csv" > "$OUT/plant_sweep.stdout.txt" 2>&1
  plat plant_end
fi
python3 "$KIT/scripts/analyze_sweep_fixed.py" "$OUT/plant_sweep.csv" --bw 5 --out "$OUT/plant" --emit-env "$OUT/gains.env" | tail -8
# shellcheck disable=SC1090
source "$OUT/gains.env"
echo "Controller gains that will be used for the 'imc' closed-loop runs:  Kp=$KP  Ki=$KI"

NT=3; [ "$SKIP_IMC" = "1" ] && NT=2
TOTAL=$(( $(echo $VARIANTS | wc -w) * REPS * NT )); N=0; FAILED=""
run_one() {   # name  flags...
  local name="$1"; shift; N=$((N+1))
  if [ -s "$OUT/$name.csv" ] && [ "$(stat -c %s "$OUT/$name.csv")" -gt 150000 ]; then echo "[$N/$TOTAL] $name  (already done, skipping)"; return; fi
  echo "[$N/$TOTAL] $name"; plat "start $name"
  $DAEMON $COMMON "$@" --csv="$OUT/$name.csv" > "$OUT/$name.stdout.txt" 2>&1
  local rc=$?; plat "end $name rc=$rc"
  if [ $rc -ne 0 ] || [ ! -s "$OUT/$name.csv" ]; then echo "   WARNING: $name did not finish (rc=$rc). See $OUT/$name.stdout.txt"; FAILED="$FAILED $name"; fi
  sleep "${GAP:-15}"
}

echo; echo "STEP 3 of 4: the main runs. Leave it alone. It prints one line per run."
for rep in $(seq 1 "$REPS"); do
  for v in $(shuf -e $VARIANTS); do
    F=$(flags_for "$v"); AGE="--log-age"     # logged for every estimator: it also gives the clock needed for the index-pulse reference
    run_one "est-${v}_A_r${rep}"    $F $AGE --no-pi --schedule=$SCHED_A --hold=$HOLD_A --abort-rpm=3200
    run_one "est-${v}_Bold_r${rep}" $F $AGE --pi --schedule=$SCHED_B --hold=$HOLD_B --kp=1.5641 --ki=41.2249 --alpha=1.0 --abort-rpm=2200
    [ "$SKIP_IMC" = "1" ] || run_one "est-${v}_Bimc_r${rep}" $F $AGE --pi --schedule=$SCHED_B --hold=$HOLD_B --kp=$KP --ki=$KI --alpha=1.0 --abort-rpm=2200
  done
done

echo; echo "STEP 4 of 4: done."
[ -n "$FAILED" ] && echo "These runs failed, run this script again to retry them:$FAILED"
echo "Next:   bash ch5kit/ch5.sh analyze"

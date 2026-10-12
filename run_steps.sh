#!/usr/bin/env bash
# MIXR-1 open-loop PWM step tests (system identification) for analyze_step.py.
#
#   sudo ./run_steps.sh map   --label imp174_baffled [--duties "8 12 16 20 24"] [--around-rpm "200 350 500"]
#       Runs the motor OPEN LOOP at a few constant duty cycles and prints which duty gives which speed with THIS impeller and water.
#       --around-rpm does not control anything: it only asks "which duty gives these speeds?" and suggests step pairs around each one.
#
#   sudo ./run_steps.sh steps --label imp174_baffled --pairs "17:21 21:17 19:23 23:19" [--repeats 3]
#       For every pair  base:step  (in % duty) the motor holds the base duty, then steps to the step duty (up AND down steps),
#       repeated --repeats times. One CSV per run, named <label>_r<k>_<base>_<step>.csv, ready for analyze_step.py.
#
# Options (flags survive sudo):  --label NAME  --out FOLDER  --hold 3 (s at the base duty before the step)  --duration 8 (total s per step run)
#   --map-duration 15 (s per duty in 'map': water, especially in an unbaffled tank, can take 10-30 s to spin up; the script warns if a speed was still moving)
#   --pause 15 (s between runs)  --wait-rest 60  --rpm-limit 800 (stop the run if the speed ever exceeds this)  --max-duty 35 (refuse higher duties)
# Env: BIN=./mixr1_daemon  FLAGS="--fifo"
#
# Keep the tank, water level and impeller identical for the whole set. Every run starts from rest and the log starts at the instant the motor
# is switched on, so each CSV also contains the complete ramp-up from 0 RPM (analyse it with:  analyze_step.py ... --startup).
set -u
MODE="${1:-}"; [ $# -gt 0 ] && shift
BIN="${BIN:-./mixr1_daemon}"; FLAGS="${FLAGS:---fifo}"
LABEL="steps"; OUT=""; PAIRS=""; DUTIES="6 10 14 18 22 26"; REPEATS=3; HOLD=3; DUR=8; MAP_DUR=15; PAUSE=15; WAIT_REST=60; RPM_LIMIT=800; MAX_DUTY=35; TARGET_RPM=""

while [ $# -gt 0 ]; do
  case "$1" in
    --label)      LABEL="${2:?}"; shift 2 ;;
    --out)        OUT="${2:?}"; shift 2 ;;
    --pairs)      PAIRS="${2:?}"; shift 2 ;;
    --duties)     DUTIES="${2:?}"; shift 2 ;;
    --repeats)    REPEATS="${2:?}"; shift 2 ;;
    --hold)       HOLD="${2:?}"; shift 2 ;;
    --duration)   DUR="${2:?}"; shift 2 ;;
    --map-duration) MAP_DUR="${2:?}"; shift 2 ;;
    --pause)      PAUSE="${2:?}"; shift 2 ;;
    --wait-rest)  WAIT_REST="${2:?}"; shift 2 ;;
    --rpm-limit)  RPM_LIMIT="${2:?}"; shift 2 ;;
    --max-duty)   MAX_DUTY="${2:?}"; shift 2 ;;
    --around-rpm|--target-rpm) TARGET_RPM="${2:?}"; shift 2 ;;
    *) echo "Unknown option '$1'"; exit 1 ;;
  esac
done
case "$MODE" in map|steps) ;; *) echo "Usage: sudo ./run_steps.sh map|steps --label NAME ...   (see the top of this file)"; exit 1 ;; esac
[ -x "$BIN" ] || { echo "Cannot find/execute $BIN (run from the folder with the daemon, or set BIN=...)"; exit 1; }
for opt in --step-from --start-at-zero; do grep -qa -- "$opt" "$BIN" || { echo "This daemon build has no $opt. Copy the new src/main.cpp and run make."; exit 1; }; done
OUT="${OUT:-steps_${LABEL}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"

over_max() { awk -v d="$1" -v m="$MAX_DUTY" 'BEGIN{exit !(d+0 > m+0)}'; }
trap 'echo; echo "Interrupted - stopping."; [ -n "${SUDO_USER:-}" ] && chown -R "$SUDO_USER" "$OUT" 2>/dev/null; exit 130' INT TERM HUP

run_daemon() {   # csv  extra-args...
  local csv=$1; shift
  "$BIN" --test --no-pi --fixed $FLAGS --no-stream --duration="$DUR" --abort-rpm="$RPM_LIMIT" --wait-rest="$WAIT_REST" --start-at-zero "$@" --csv="$csv" 2>&1
}

if [ "$MODE" = "map" ]; then
  echo "Duty -> speed map for '$LABEL' (${MAP_DUR} s per duty, speed averaged over the last 3 s, drift checked). Motor limit: stops above $RPM_LIMIT RPM, refuses duties above $MAX_DUTY %."
  DUR="$MAP_DUR"; : > "$OUT/map.txt"
  for d in $DUTIES; do
    if over_max "$d"; then echo "  skipping $d % (above --max-duty $MAX_DUTY)"; continue; fi
    counts=$(awk -v d="$d" 'BEGIN{printf "%d", d/100*4095 + 0.5}')
    echo "  running $d % ($counts counts) ..."
    out=$(run_daemon "$OUT/map_${d}.csv" --pwm="$counts")
    if printf '%s\n' "$out" | grep -q '^\[ABORTED\]'; then echo "  !! speed limit reached at $d % - stopping the map here"; break; fi
    python3 - "$OUT/map_${d}.csv" "$d" "$DUR" >> "$OUT/map.txt" << 'PY'
import csv, sys, statistics
f, d, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rows = [(float(r["elapsed_s"]), float(r["raw_rpm"])) for r in csv.DictReader(open(f))]
tail = [(t, v) for t, v in rows if t >= dur - 3.0]
ts, vs = [t for t, _ in tail], [v for _, v in tail]
tm, vm = statistics.mean(ts), statistics.mean(vs)
slope = sum((t - tm) * (v - vm) for t, v in tail) / max(sum((t - tm) ** 2 for t in ts), 1e-12)      # RPM per second over the last 3 s
moving = "MOVING" if abs(slope) > max(0.005 * vm, 1.5) else "ok"
print(f"{d:g} {vm:.1f} {statistics.pstdev(vs):.2f} {slope:+.2f} {moving}")
PY
    sleep "$PAUSE"
  done
  echo; echo "duty %   mean RPM   noise (std)   drift RPM/s   settled?"; awk '{printf "%6s   %8s   %11s   %11s   %s\n", $1, $2, $3, $4, ($5=="MOVING" ? "NO - still moving" : "yes")}' "$OUT/map.txt"
  if grep -q MOVING "$OUT/map.txt"; then
    echo; echo "!! At least one duty was still changing at the end of its run (the speed or the water was still settling)."
    echo "   Those speeds are too low or too high by the remaining change. Re-run with a longer time, e.g.:  --map-duration 40   (and use a longer --hold for the steps)."
  fi
  python3 - "$OUT/map.txt" << 'PY'
import sys
pts = sorted((float(a.split()[0]), float(a.split()[1])) for a in open(sys.argv[1]) if a.strip())
if len(pts) >= 2:
    print("\nLocal plant gain from the map (RPM per PWM count between neighbouring duties):")
    for (d0, r0), (d1, r1) in zip(pts, pts[1:]):
        print(f"   {d0:g} -> {d1:g} %:  K = {(r1 - r0) / ((d1 - d0) / 100 * 4095):.3f}   (around {(r0 + r1) / 2:.0f} RPM)")
    (d0, r0), (d1, r1) = pts[0], pts[1]
    k = (r1 - r0) / ((d1 - d0) / 100 * 4095)
    if k > 0:
        print(f"   the speed line reaches 0 RPM near {d0 - r0 / k / 4095 * 100:.1f} % duty (friction dead zone below that)")
PY
  if [ -n "$TARGET_RPM" ]; then
    python3 - "$OUT/map.txt" "$TARGET_RPM" << 'PY'
import sys
pts = sorted((float(a.split()[1]), float(a.split()[0])) for a in open(sys.argv[1]) if a.strip())
print()
for t in (float(x) for x in sys.argv[2].replace(",", " ").split()):
    for (r0, d0), (r1, d1) in zip(pts, pts[1:]):
        if r0 <= t <= r1:
            d = d0 + (t - r0) * (d1 - d0) / (r1 - r0)
            print(f"{t:>5g} RPM is about {d:4.1f} % duty.  Step pairs (about +-50 RPM, up and down):  {d-2:.0f}:{d+2:.0f}  {d+2:.0f}:{d-2:.0f}  {d-1:.0f}:{d+3:.0f}  {d+3:.0f}:{d-1:.0f}")
            break
    else:
        print(f"{t:>5g} RPM is outside the measured range ({pts[0][0]:.0f}-{pts[-1][0]:.0f} RPM): add duties with --duties (and --max-duty if the motor allows it).")
PY
  fi
  [ -n "${SUDO_USER:-}" ] && chown -R "$SUDO_USER" "$OUT" 2>/dev/null
  echo; echo "Files: $OUT"
  exit 0
fi

# ---------------------------------------------------------------- steps
[ -n "$PAIRS" ] || { echo "Give the steps: --pairs \"17:21 21:17 19:23 23:19\"   (base:step in % duty; run 'map' first to choose them)"; exit 1; }
for p in $PAIRS; do
  b="${p%%:*}"; s="${p##*:}"
  [[ "$b" =~ ^[0-9]+([.][0-9]+)?$ && "$s" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "Bad pair '$p' (use base:step, e.g. 17:21)"; exit 1; }
  if over_max "$b" || over_max "$s"; then echo "Pair $p is above --max-duty $MAX_DUTY %. Raise --max-duty only if the motor and amplifier allow it."; exit 1; fi
done
npairs=$(echo $PAIRS | wc -w); total=$((npairs * REPEATS)); n=0
echo "Step set '$LABEL': $npairs pairs x $REPEATS repeats = $total runs of ${DUR}s (step at ${HOLD}s), pause ${PAUSE}s  -> about $(awk -v t="$total" -v d="$DUR" -v p="$PAUSE" 'BEGIN{printf "%d", t*(d+p+3)/60}') min"
echo "Output folder: $OUT"; sleep 3
for r in $(seq 1 "$REPEATS"); do
  for p in $PAIRS; do
    b="${p%%:*}"; s="${p##*:}"; n=$((n + 1))
    echo "[$n/$total] repeat $r: ${b}% -> ${s}%"
    out=$(run_daemon "$OUT/${LABEL}_r${r}_${b}_${s}.csv" --step-from="$b" --step-to="$s" --step-at="$HOLD")
    if printf '%s\n' "$out" | grep -q '^\[ABORTED\]'; then echo "   !! speed limit ($RPM_LIMIT RPM) reached - run stopped; file kept but incomplete: ${LABEL}_r${r}_${b}_${s}.csv"; fi
    python3 - "$OUT/${LABEL}_r${r}_${b}_${s}.csv" "$HOLD" << 'PY'
import csv, sys, statistics
f, hold = sys.argv[1], float(sys.argv[2])
try:
    rows = [(float(x["elapsed_s"]), float(x["raw_rpm"])) for x in csv.DictReader(open(f))]
except Exception:
    sys.exit(0)
pre = [(t, v) for t, v in rows if hold - 1.0 <= t < hold - 0.05]
if len(pre) >= 10:
    ts, vs = [t for t, _ in pre], [v for _, v in pre]
    tm, vm = statistics.mean(ts), statistics.mean(vs)
    slope = sum((t - tm) * (v - vm) for t, v in pre) / max(sum((t - tm) ** 2 for t in ts), 1e-12)
    if abs(slope) > max(0.005 * vm, 1.5):
        print(f"   !! the speed was still drifting before the step ({slope:+.1f} RPM/s): the base level is not settled. Use a longer --hold (e.g. {int(hold * 3)}) and a longer --duration.")
PY
    if printf '%s\n' "$out" | grep -q "shaft still turning"; then echo "   !! the shaft was still turning at the start of this run"; fi
    sleep "$PAUSE"
  done
done
[ -n "${SUDO_USER:-}" ] && chown -R "$SUDO_USER" "$OUT" 2>/dev/null
echo; echo "Done. Analyse with:"
echo "  python3 tools/analyze_step.py $LABEL=\"$OUT/*.csv\" --t-step $HOLD --out ${OUT}_analysis"
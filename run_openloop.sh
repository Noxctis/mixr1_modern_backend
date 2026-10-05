#!/usr/bin/env bash
# Run several open-loop (constant PWM, no controller) tests with IDENTICAL settings so they can be graphed together.
#
# Usage (run from the folder containing mixr1_daemon):
#   sudo ./run_openloop.sh <DURATION_S> <TAG> <PWM> [<PWM> ...]
# Example (the plan: 770 first and last = drift check):
#   sudo ./run_openloop.sh 20 loaded 770 215 1930 290 770
#   sudo ./run_openloop.sh 20 freespin 770        # impeller removed
#
# PWM is 0-4095 (770 = 18.8 %).  Every run uses the same flags: --test --fixed --no-pi --fifo --no-stream.
# Output: openloop_<TAG>_<timestamp>/<TAG>_pwm<PWM>_r<n>.csv  and a summary table of the speed each PWM gave.
set -u
DUR="${1:?duration in seconds}"; TAG="${2:?tag, e.g. loaded or freespin}"; shift 2
[ "$#" -ge 1 ] || { echo "give at least one PWM value"; exit 1; }
BIN="${BIN:-./mixr1_daemon}"; PAUSE="${PAUSE:-5}"
OUT="openloop_${TAG}_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
declare -A seen; n=0; total=$#
echo "Duration=${DUR}s  tag=$TAG  runs=$total  (~$(( total * (DUR + PAUSE) / 60 + 1 )) min).  Output: $OUT"
for pwm in "$@"; do
  n=$((n + 1)); seen[$pwm]=$(( ${seen[$pwm]:-0} + 1 ))
  f="$OUT/${TAG}_pwm${pwm}_r${seen[$pwm]}.csv"
  echo "[$n/$total] PWM $pwm -> $f"
  "$BIN" --test --fixed --no-pi --fifo --no-stream --pwm="$pwm" --duration="$DUR" --csv="$f" > /dev/null 2>&1 \
    || echo "   !! daemon returned an error for PWM $pwm"
  sleep "$PAUSE"
done
python3 - "$OUT" "$DUR" << 'PY'
import sys, glob, os, pandas as pd, numpy as np
out, dur = sys.argv[1], float(sys.argv[2])
print(f"\n{'file':<34}{'PWM':>6}{'PWM %':>7}{'mean RPM':>10}{'RPM std':>9}{'std %':>7}{'rev (s)':>9}  note")
for f in sorted(glob.glob(os.path.join(out, "*.csv"))):
    d = pd.read_csv(f); s = d[d.elapsed_s >= 5]; note = []
    if len(s) < 50:
        print(f"{os.path.basename(f):<34} run too short for a 5 s warm-up window (use at least 10 s)"); continue
    if abs(d.elapsed_s.iloc[-1] - dur) > 0.5: note.append("DURATION OFF")
    m = s.raw_rpm.mean()
    if m < 30: note.append("MOTOR DID NOT SPIN? raise PWM")
    print(f"{os.path.basename(f):<34}{int(d.pwm.iloc[0]):>6}{d.pwm.iloc[0]*100/4095:>7.1f}{m:>10.1f}{s.raw_rpm.std():>9.2f}"
          f"{(s.raw_rpm.std()/m*100 if m>0 else 0):>7.2f}{(60/m if m>0 else 0):>9.2f}  {' '.join(note)}")
print("\nSteady-state window for analysis = 5 s to end of run (same for every file).")
PY
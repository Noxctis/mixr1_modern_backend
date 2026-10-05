#!/usr/bin/env bash
# MIXR-1 PI tuning matrix: runs each configuration as a step response (0 -> TARGET RPM),
# collects the [RESULT] line from test mode, and prints a ranked, averaged table.
#
# Usage (run from the folder containing the binary, usually with sudo for SCHED_FIFO):
#   sudo ./run_matrix.sh [TARGET_RPM=460] [DURATION_S=20] [REPEATS=2] [SET=main|extra]
#     main  = the original 11 configs (Kp and alpha sweeps)
#     extra = 7 configs: baseline/C3/D3 as anchors + Kp 0.8 with alpha 0.5, and Ki sweeps at Kp 0.8
# Env overrides: BIN=./mixr1_daemon  FLAGS="--fifo"  PAUSE=4  STREAM=0|1
#
# Keep the physical conditions identical for the whole matrix (same load, e.g. impeller in water).
set -u
TARGET="${1:-460}"
DUR="${2:-20}"
REPEATS="${3:-2}"
SET="${4:-main}"
BIN="${BIN:-./mixr1_daemon}"
FLAGS="${FLAGS:---fifo}"
PAUSE="${PAUSE:-4}"
STREAM="${STREAM:-0}"

KI=41.2249
# name  kp  ki  alpha
if [ "$SET" = "extra" ]; then
CONFIGS=(
  "A_baseline         1.5641 $KI 1.0"
  "C3_kp0.8           0.8    $KI 1.0"
  "D3_kp1.0_a0.5      1.0    $KI 0.5"
  "F1_kp0.8_a0.5      0.8    $KI 0.5"
  "F2_kp0.8_ki25      0.8    25  1.0"
  "F3_kp0.8_ki25_a0.5 0.8    25  0.5"
  "F4_kp0.8_ki15      0.8    15  1.0"
)
else
CONFIGS=(
  "A_baseline     1.5641 $KI 1.0"
  "B1_alpha0.5    1.5641 $KI 0.5"
  "B2_alpha0.3    1.5641 $KI 0.3"
  "B3_alpha0.15   1.5641 $KI 0.15"
  "C1_kp1.2       1.2    $KI 1.0"
  "C2_kp1.0       1.0    $KI 1.0"
  "C3_kp0.8       0.8    $KI 1.0"
  "D1_kp1.2_a0.5  1.2    $KI 0.5"
  "D2_kp1.2_a0.3  1.2    $KI 0.3"
  "D3_kp1.0_a0.5  1.0    $KI 0.5"
  "D4_kp1.0_a0.3  1.0    $KI 0.3"
)
fi

OUT="matrix_${SET}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
RAW="$OUT/results_raw.txt"
: > "$RAW"

STREAM_FLAG="--no-stream"
[ "$STREAM" = "1" ] && STREAM_FLAG="--stream"

total=$(( ${#CONFIGS[@]} * REPEATS ))
n=0
echo "Set=$SET  Target=$TARGET RPM  duration=${DUR}s  repeats=$REPEATS  runs=$total  (~$(( total * (DUR + PAUSE) / 60 )) min)"
echo "Output folder: $OUT"

# Repeats are the OUTER loop so motor heating/drift is spread evenly across configs.
for r in $(seq 1 "$REPEATS"); do
  for cfg in "${CONFIGS[@]}"; do
    read -r name kp ki alpha <<< "$cfg"
    n=$((n + 1))
    echo "[$n/$total] $name (kp=$kp ki=$ki alpha=$alpha) repeat $r"
    line=$("$BIN" --test --pi --fixed $FLAGS $STREAM_FLAG \
             --target="$TARGET" --duration="$DUR" \
             --kp="$kp" --ki="$ki" --alpha="$alpha" \
             --csv="$OUT/${name}_r${r}.csv" 2>&1 | grep '^\[RESULT\]' | sed 's/^\[RESULT\] //')
    if [ -z "$line" ]; then
      echo "   !! no [RESULT] line (run failed?)"
    else
      echo "$name $line" >> "$RAW"
    fi
    sleep "$PAUSE"
  done
done

python3 - "$RAW" "$OUT/summary.txt" << 'PY'
import sys, collections
raw, outp = sys.argv[1], sys.argv[2]
groups = collections.OrderedDict()
for ln in open(raw):
    parts = ln.split()
    if not parts: continue
    name, kv = parts[0], dict(p.split('=') for p in parts[1:])
    groups.setdefault(name, []).append({k: float(v) for k, v in kv.items()})
cols = ['kp','ki','alpha','pwm_jitter_pct','pwm_std_pct','rpm_std','sse','overshoot_pct','settle5_s']
rows = []
for name, runs in groups.items():
    avg = {c: sum(r[c] for r in runs) / len(runs) for c in cols}
    rows.append((name, len(runs), avg))
rows.sort(key=lambda r: r[2]['pwm_jitter_pct'])
hdr = f"{'config':<16}{'n':>2} {'kp':>6} {'ki':>6} {'alpha':>6} {'pwmJit%':>8} {'pwmStd%':>8} {'rpmStd':>7} {'sse':>7} {'ovr%':>6} {'settle_s':>8}"
lines = ["Ranked by PWM jitter (lower = flatter). Check overshoot/settle before picking a winner.", hdr, "-" * len(hdr)]
for name, n, a in rows:
    lines.append(f"{name:<16}{n:>2} {a['kp']:>6.2f} {a['ki']:>6.1f} {a['alpha']:>6.2f} {a['pwm_jitter_pct']:>8.3f} {a['pwm_std_pct']:>8.3f} "
                 f"{a['rpm_std']:>7.2f} {a['sse']:>7.2f} {a['overshoot_pct']:>6.2f} {a['settle5_s']:>8.2f}")
text = "\n".join(lines)
print("\n" + text)
open(outp, 'w').write(text + "\n")
PY
echo "Summary saved to $OUT/summary.txt"
#!/usr/bin/env bash
# MIXR-1 PI grid search: a step response (0 -> TARGET RPM) for EVERY combination of Kp and filter alpha.
# Ki is NOT touched (fixed at KI below). Everything goes into ONE output folder.
#
# Usage (run from the folder containing the binary; sudo for SCHED_FIFO):
#   sudo ./run_matrix.sh [TARGET_RPM=460] [DURATION_S=20] [REPEATS=2]
#
# Grid size (env GRID=fast|full|wide|max, default full):
#   fast  5 Kp x 5 alpha  =  25 settings
#   full  9 Kp x 9 alpha  =  81 settings        (default)
#   wide 10 Kp x  9 alpha =  90 settings        (same as full, plus Kp 1.8, 2.0, 2.5 ABOVE the baseline)
#   max  12 Kp x 12 alpha = 144 settings
#   or your own:  KPS="1.5641 1.2 0.8" ALPHAS="1.0 0.5 0.3"
#   or an explicit list:  CONFIGS_FILE=path   (one setting per line:  name kp ki alpha)
#
# Every option can ALSO be a flag after the numbers (flags survive sudo, environment variables may not):
#   sudo ./run_matrix.sh 360 20 1 --out water_baffled --pause 15
#   sudo ./run_matrix.sh 360 20 5 --configs water_baffled/confirm_list.txt --out confirm
#   flags: --out --pause --grid --configs --kps --alphas --wait-rest --foreground
# Other env overrides:
#   BIN=./mixr1_daemon  FLAGS="--fifo"  PAUSE=4  STREAM=0|1  KI=41.2249
#   ABORT_RPM=<rpm>   safety: stop the motor and flag the run if speed exceeds this (default 2 x target; 0 = off)
#   WAIT_REST=60      before every run wait (up to this many seconds) until the shaft has stopped, so each run starts FROM REST (0 = off)
#   ANCHOR_EVERY=20   re-run the baseline every N runs to watch for motor warm-up drift (0 = off)
#   MAX_OVERSHOOT=5  MAX_SETTLE=0.3  TOP=8     what counts as an acceptable setting in the shortlist
#   OUT=<folder>      continue an interrupted run: already-finished runs are skipped
#   NOWAIT=1          skip the 5 s countdown
#
# The run DETACHES from your terminal after the 5 s countdown: closing the terminal or losing SSH does NOT stop it.
# Progress goes to <folder>/run.log. From any new SSH session:
#   ./run_matrix.sh status [folder]       is it running? how many runs are done? last log lines
#   tail -f <folder>/run.log              watch it live (Ctrl+C only stops the watching)
#   sudo ./run_matrix.sh stop [folder]    stop it cleanly (motor off, summary of finished runs written)
# DETACH=0 keeps it in the foreground instead (then Ctrl+C or a dropped connection stops it cleanly).
#
# Keep the physical conditions identical for the whole run (same load, e.g. impeller in water).
# Unattended motor runs: keep the tank secure and someone nearby; the speed limit (ABORT_RPM) protects against runaways.
set -u

# ---------------------------------------------------------------- subcommands: status / stop
latest_folder() { ls -dt matrix_* 2>/dev/null | head -1; }
case "${1:-}" in
  status)
    d="${2:-$(latest_folder)}"; [ -d "${d:-/nonexistent}" ] || { echo "No run folder found (give it: ./run_matrix.sh status <folder>)"; exit 1; }
    pid=""; [ -f "$d/run.pid" ] && pid=$(cat "$d/run.pid")
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then echo "Folder $d: RUNNING (PID $pid)"; else echo "Folder $d: NOT running (finished, stopped, or crashed)"; fi
    ndone=$(wc -l < "$d/.done" 2>/dev/null || echo 0); nres=$(wc -l < "$d/results_raw.txt" 2>/dev/null || echo 0)
    nab=0; [ -f "$d/aborted_runs.txt" ] && nab=$(wc -l < "$d/aborted_runs.txt")
    total=$(grep -m1 -o 'total runs=[0-9]*' "$d/run.log" 2>/dev/null | cut -d= -f2)
    echo "Finished runs: ${ndone}${total:+ of $total}   (valid results: $nres, stopped by the speed limit: $nab)"
    echo "--- last lines of $d/run.log:"; tail -6 "$d/run.log" 2>/dev/null
    exit 0 ;;
  stop)
    d="${2:-$(latest_folder)}"; [ -d "${d:-/nonexistent}" ] || { echo "No run folder found (give it: sudo ./run_matrix.sh stop <folder>)"; exit 1; }
    pid=""; [ -f "$d/run.pid" ] && pid=$(cat "$d/run.pid")
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Stopping run in $d (PID $pid) ..."
      pkill -TERM -s "$pid" 2>/dev/null || kill -TERM "$pid"        # polite stop for the script AND the daemon (a background job ignores Ctrl+C/SIGINT)
      for i in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
      if kill -0 "$pid" 2>/dev/null; then echo "Still running after 30 s - forcing"; pkill -KILL -s "$pid" 2>/dev/null; sleep 1; fi
    else echo "No run is active in $d."; fi
    # always make sure the motor outputs are off (pigpiod keeps the last PWM if a program is killed hard)
    if command -v pigs >/dev/null 2>&1; then
      pigs hp "${MOTOR_PWM_PIN:-13}" 20000 0 >/dev/null 2>&1; pigs w "${MOTOR_EN_PIN:-15}" 0 >/dev/null 2>&1
      echo "Motor outputs forced off (PWM pin ${MOTOR_PWM_PIN:-13}, enable pin ${MOTOR_EN_PIN:-15})."
    else echo "'pigs' not found: check that the motor has stopped (cut motor power if unsure)."; fi
    echo "Resume later with:  sudo env OUT=$d ./run_matrix.sh <same target> <same duration> <same repeats>"
    exit 0 ;;
esac

# ---------------------------------------------------------------- options as command-line flags (they survive sudo; environment variables often do not)
ORIG_ARGS=("$@")
POS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --out)        OUT="${2:?--out needs a folder name}"; shift 2 ;;
    --pause)      PAUSE="${2:?--pause needs seconds}"; shift 2 ;;
    --grid)       GRID="${2:?--grid needs fast|full|wide|max}"; shift 2 ;;
    --configs)    CONFIGS_FILE="${2:?--configs needs a settings file}"; shift 2 ;;
    --kps)        KPS="${2:?--kps needs a quoted list}"; shift 2 ;;
    --alphas)     ALPHAS="${2:?--alphas needs a quoted list}"; shift 2 ;;
    --wait-rest)  WAIT_REST="${2:?--wait-rest needs seconds}"; shift 2 ;;
    --foreground) DETACH=0; shift ;;
    -*)           echo "Unknown option '$1'. Options: --out --pause --grid --configs --kps --alphas --wait-rest --foreground"; exit 1 ;;
    *)            POS+=("$1"); shift ;;
  esac
done
set -- ${POS[@]+"${POS[@]}"}
TARGET="${1:-460}"
DUR="${2:-20}"
REPEATS="${3:-2}"
# the three positional arguments must be numbers (a typical mistake: putting the output folder name here)
for pair in "TARGET_RPM:$TARGET" "DURATION_S:$DUR" "REPEATS:$REPEATS"; do
  if ! [[ "${pair#*:}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "Argument ${pair%%:*} must be a number but is '${pair#*:}'."
    echo "Usage:  sudo env OUT=<folder name> ./run_matrix.sh <target RPM> <duration s> <repeats>"
    echo "Example: sudo env OUT=water90mmbaffle200mmwater PAUSE=15 ./run_matrix.sh 360 20 1"
    exit 1
  fi
done
BIN="${BIN:-./mixr1_daemon}"
FLAGS="${FLAGS:---fifo}"
PAUSE="${PAUSE:-4}"
STREAM="${STREAM:-0}"
KI="${KI:-41.2249}"
GRID="${GRID:-full}"
ANCHOR_EVERY="${ANCHOR_EVERY:-20}"
MAX_OVERSHOOT="${MAX_OVERSHOOT:-5}"
MAX_SETTLE="${MAX_SETTLE:-0.3}"
TOP="${TOP:-8}"
BASE_KP=1.5641

[ -x "$BIN" ] || { echo "Cannot find/execute $BIN (run from the folder with the daemon, or set BIN=...)"; exit 1; }

# ---------------------------------------------------------------- the settings to test
CONFIGS=()      # each entry:  "name kp ki alpha"
if [ -n "${CONFIGS_FILE:-}" ] && [ ! -f "$CONFIGS_FILE" ]; then
  echo "Settings file not found: $CONFIGS_FILE"; exit 1
fi
if [ -n "${CONFIGS_FILE:-}" ]; then
  while read -r name kp ki alpha rest; do
    [[ -z "${name:-}" || "$name" == \#* ]] && continue
    if ! [[ "${kp:-}" =~ ^[0-9]+([.][0-9]+)?$ && "${ki:-}" =~ ^[0-9]+([.][0-9]+)?$ && "${alpha:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
      echo "Skipping unreadable line in $CONFIGS_FILE: '$name ${kp:-} ${ki:-} ${alpha:-}'"; continue
    fi
    CONFIGS+=("$name $kp $ki $alpha")
  done < "$CONFIGS_FILE"
else
  case "$GRID" in
    fast) D_KPS="$BASE_KP 1.2 1.0 0.8 0.6";                                   D_ALPHAS="1.0 0.7 0.5 0.3 0.2" ;;
    wide) D_KPS="2.5 2.0 1.8 $BASE_KP 1.4 1.2 1.0 0.8 0.6 0.5";                D_ALPHAS="1.0 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15" ;;
    max)  D_KPS="$BASE_KP 1.4 1.3 1.2 1.1 1.0 0.9 0.8 0.7 0.6 0.5 0.4";       D_ALPHAS="1.0 0.9 0.8 0.7 0.6 0.5 0.45 0.4 0.3 0.25 0.2 0.15" ;;
    *)    D_KPS="$BASE_KP 1.4 1.2 1.0 0.9 0.8 0.7 0.6 0.5";                   D_ALPHAS="1.0 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.15" ;;
  esac
  KPS="${KPS:-$D_KPS}"; ALPHAS="${ALPHAS:-$D_ALPHAS}"
  i=0
  for kp in $KPS; do
    for al in $ALPHAS; do
      i=$((i + 1))
      if [ "$kp" = "$BASE_KP" ] && [ "$al" = "1.0" ]; then name="A_baseline"
      else name=$(printf "G%02d_kp%s_a%s" "$i" "$kp" "$al"); fi
      CONFIGS+=("$name $kp $KI $al")
    done
  done
fi
N=${#CONFIGS[@]}
[ "$N" -ge 1 ] || { echo "No settings to run"; exit 1; }

# baseline used for drift anchors (taken from the list if present, else the original design)
ANCHOR="A_baseline $BASE_KP $KI 1.0"

# ---------------------------------------------------------------- safety limit
ABORT_RPM="${ABORT_RPM:-$(awk -v t="$TARGET" 'BEGIN{printf "%d", 2*t}')}"
ABORT_FLAG=""
if [ "$ABORT_RPM" != "0" ]; then
  if grep -qa -- "--abort-rpm" "$BIN"; then ABORT_FLAG="--abort-rpm=$ABORT_RPM"
  else echo "WARNING: this daemon build has no --abort-rpm safety limit. Rebuild it (make) with the updated main.cpp, or set ABORT_RPM=0 to run without it."
       exit 1; fi
fi
STREAM_FLAG="--no-stream"; [ "$STREAM" = "1" ] && STREAM_FLAG="--stream"
WAIT_REST="${WAIT_REST:-60}"; REST_FLAG=""
if [ "$WAIT_REST" != "0" ]; then
  if grep -qa -- "--wait-rest" "$BIN"; then REST_FLAG="--wait-rest=$WAIT_REST"
  else echo "NOTE: this daemon build has no --wait-rest, so runs may start while the shaft is still coasting. Rebuild (make) with the updated main.cpp, or raise PAUSE (e.g. PAUSE=20)."; fi
fi

# ---------------------------------------------------------------- output folder (ONE for everything)
OUT="${OUT:-matrix_all_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
RAW="$OUT/results_raw.txt"; DONE="$OUT/.done"; ABORTS="$OUT/aborted_runs.txt"; ORDER="$OUT/run_order.txt"
touch "$RAW" "$DONE"
cp "$0" "$OUT/run_matrix_used.sh" 2>/dev/null || true
printf '%s\n' "${CONFIGS[@]}" > "$OUT/settings_tested.txt"
printf 'max_overshoot=%s\nmax_settle=%s\ntop=%s\n' "$MAX_OVERSHOOT" "$MAX_SETTLE" "$TOP" > "$OUT/limits.txt"

anchors=0; [ "$ANCHOR_EVERY" -gt 0 ] && anchors=$(( (N * REPEATS) / ANCHOR_EVERY ))
total=$(( N * REPEATS + anchors ))
mins=$(awk -v t="$total" -v d="$DUR" -v p="$PAUSE" 'BEGIN{printf "%d", t*(d+p+2)/60}')
if [ -n "${CONFIGS_FILE:-}" ]; then echo "Settings: $N taken from $CONFIGS_FILE   repeats=$REPEATS  anchors=$anchors  total runs=$total"
else echo "Grid=$GRID  settings=$N  repeats=$REPEATS  anchors=$anchors  total runs=$total"; fi
echo "Target=$TARGET RPM  duration=${DUR}s  pause=${PAUSE}s  Ki fixed at $KI  abort limit=${ABORT_RPM} RPM"
echo "Estimated time: ${mins} min   (Ctrl+C any time; re-run with OUT=$OUT to continue)"
echo "Output folder: $OUT"
if [ "${NOWAIT:-0}" != "1" ]; then
  echo "Impeller in water? Hands clear of the shaft? Starting in 5 s... (Ctrl+C now to cancel)"; sleep 5
fi

# ---------------------------------------------------------------- detach: from here on a lost SSH connection cannot touch the run
if [ "${MIXR_DETACHED:-0}" != "1" ] && [ "${DETACH:-1}" = "1" ] && command -v setsid >/dev/null 2>&1; then
  MIXR_DETACHED=1 NOWAIT=1 OUT="$OUT" setsid nohup "$0" ${ORIG_ARGS[@]+"${ORIG_ARGS[@]}"} >> "$OUT/run.log" 2>&1 < /dev/null &
  sleep 1
  echo
  echo "Started in the background - it keeps running if you close this terminal or lose SSH."
  echo "  watch it:   tail -f $OUT/run.log        (Ctrl+C only stops the watching)"
  echo "  status:     ./run_matrix.sh status $OUT"
  echo "  stop it:    sudo ./run_matrix.sh stop $OUT"
  exit 0
fi

# ---------------------------------------------------------------- summary (also runs on Ctrl+C / exit)
summarize() {
  trap - EXIT
  # the daemon ran under sudo: give the folder back to the normal user so mixr_figures.py can write into it
  [ -n "${SUDO_USER:-}" ] && chown -R "$SUDO_USER" "$OUT" 2>/dev/null
  [ -s "$RAW" ] || { echo "No finished runs to summarise."; return; }
  python3 - "$RAW" "$OUT" "$MAX_OVERSHOOT" "$MAX_SETTLE" "$TOP" "$KI" "$TARGET" "$DUR" << 'PY'
import sys, os, collections
raw, out, max_ovr, max_settle, top, ki0 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5]), float(sys.argv[6])
tgt_arg, dur_arg = sys.argv[7], sys.argv[8]
groups = collections.OrderedDict()
for ln in open(raw):
    p = ln.split()
    if not p: continue
    kv = {k: float(v) for k, v in (x.split('=') for x in p[1:])}
    groups.setdefault(p[0], []).append(kv)
cols = ['kp', 'ki', 'alpha', 'pwm_jitter_pct', 'pwm_std_pct', 'rpm_std', 'sse', 'overshoot_pct', 'settle5_s']
rows = []
for name, runs in groups.items():
    a = {c: sum(r[c] for r in runs) / len(runs) for c in cols}
    a['name'], a['n'] = name, len(runs)
    a['spread'] = (max(r['pwm_jitter_pct'] for r in runs) - min(r['pwm_jitter_pct'] for r in runs)) if len(runs) > 1 else float('nan')
    rows.append(a)
base = next((r for r in rows if r['name'] == 'A_baseline'), None)
ok = lambda r: r['overshoot_pct'] <= max_ovr and 0 <= r['settle5_s'] <= max_settle
feas = sorted([r for r in rows if ok(r)], key=lambda r: r['pwm_jitter_pct'])
allr = sorted(rows, key=lambda r: r['pwm_jitter_pct'])
spreads = [r['spread'] for r in rows if r['n'] > 1 and r['spread'] == r['spread']]
noise = sum(spreads) / len(spreads) if spreads else float('nan')

def line(r):
    d = '' if (base is None or r is base or not base['pwm_jitter_pct']) else f"{(r['pwm_jitter_pct'] / base['pwm_jitter_pct'] - 1) * 100:+5.0f}%"
    return (f"{r['name']:<24}{r['n']:>2} {r['kp']:>6.2f} {r['alpha']:>6.2f} {r['pwm_jitter_pct']:>8.3f} {d:>6} {r['rpm_std']:>7.2f} "
            f"{r['overshoot_pct']:>6.2f} {r['settle5_s']:>8.2f}")
hdr = f"{'setting':<24}{'n':>2} {'kp':>6} {'alpha':>6} {'pwmJit%':>8} {'vs A':>6} {'rpmStd':>7} {'ovr%':>6} {'settle_s':>8}"
L = [f"{len(rows)} settings finished.  Ki fixed at {ki0:g}.  Acceptable = overshoot <= {max_ovr:g} % and settling <= {max_settle:g} s.",
     f"Typical repeat-to-repeat spread of PWM jitter: {noise:.3f} % duty -> differences smaller than this are NOT real.", "",
     f"=== BEST ACCEPTABLE SETTINGS (lowest PWM jitter), top {top} ===", hdr, "-" * len(hdr)]
L += [line(r) for r in feas[:top]] or ["  (none meet the limits - relax MAX_OVERSHOOT / MAX_SETTLE)"]
if base: L += ["", "baseline for reference:", line(base)]
L += ["", "=== ALL SETTINGS ranked by PWM jitter (top 20; check overshoot!) ===", hdr, "-" * len(hdr)] + [line(r) for r in allr[:20]]

# grid views: Kp (rows) x alpha (columns), only settings that used the fixed Ki
g = [r for r in rows if abs(r['ki'] - ki0) < 0.05]
kps = sorted({round(r['kp'], 4) for r in g}, reverse=True); als = sorted({round(r['alpha'], 3) for r in g}, reverse=True)
look = {(round(r['kp'], 4), round(r['alpha'], 3)): r for r in g}
def grid(title, key, fmt):
    t = ["", f"=== {title} (rows: Kp, columns: alpha; * = acceptable) ===", "   Kp/alpha" + "".join(f"{a:>8g}" for a in als)]
    for k in kps:
        cells = []
        for a in als:
            r = look.get((k, a))
            cells.append(f"{((fmt % r[key]) + ('*' if ok(r) else ' ')) if r else '.':>8}")
        t.append(f"{k:>9.3f}" + "".join(cells))
    return t
if len(kps) > 1 and len(als) > 1:
    L += grid("PWM jitter, % duty", 'pwm_jitter_pct', "%.3f") + grid("Startup overshoot, %", 'overshoot_pct', "%.1f") + grid("Settling time to +-5 %, s", 'settle5_s', "%.2f")
open(os.path.join(out, 'summary.txt'), 'w').write("\n".join(L) + "\n")
print("\n" + "\n".join(L))

# the shortlist, in the format that CONFIGS_FILE reads: re-run it with more repeats to confirm
conf = ([base] if base else []) + [r for r in feas[:top] if r is not base]
exact = {}                                     # the daemon prints 3 decimals; the shortlist must use the exact tested values
try:
    for ln in open(os.path.join(out, 'settings_tested.txt')):
        p = ln.split()
        if len(p) == 4: exact[p[0]] = p[1:]
except OSError:
    pass
with open(os.path.join(out, 'confirm_list.txt'), 'w') as f:
    f.write("# name kp ki alpha   (confirm with:  sudo ./run_matrix.sh %s %s 5 --configs %s/confirm_list.txt --out %s_confirm)\n" % (tgt_arg, dur_arg, out, out))
    for r in conf:
        kp, ki, al = exact.get(r['name'], [f"{r['kp']:g}", f"{r['ki']:g}", f"{r['alpha']:g}"])
        f.write(f"{r['name']} {kp} {ki} {al}\n")
print(f"\nShortlist written to {out}/confirm_list.txt")
PY
  if [ -s "$ABORTS" ]; then echo; echo "Runs stopped by the speed limit (flagged, left out of the results): see $ABORTS"; fi
  echo "All results: $OUT   (feed this folder to mixr_figures.py)"
  echo "[$(date '+%F %T')] run ended"
  rm -f "$OUT/run.pid"
}
trap summarize EXIT
if [ "${MIXR_DETACHED:-0}" = "1" ]; then
  echo $$ > "$OUT/run.pid"
  trap '' HUP                                   # a hang-up must never stop a detached run
  trap 'echo; echo "Stopped on request - finished runs are saved (resume with: OUT=$OUT ...)."; exit 130' INT TERM
  echo "[$(date '+%F %T')] run started (PID $$), target=$TARGET duration=${DUR}s repeats=$REPEATS"
else
  trap 'echo; echo "Interrupted or SSH disconnected - stopping cleanly (resume with: OUT=$OUT ...)."; exit 130' INT TERM HUP
fi

# ---------------------------------------------------------------- run one test
run_one() {  # name kp ki alpha r
  local name=$1 kp=$2 ki=$3 alpha=$4 r=$5
  local csv="$OUT/${name}_r${r}.csv"
  if grep -qx "$name $r" "$DONE"; then echo "   (already done, skipping)"; return 0; fi
  local output res ab
  output=$("$BIN" --test --pi --fixed $FLAGS $STREAM_FLAG --target="$TARGET" --duration="$DUR" \
             --kp="$kp" --ki="$ki" --alpha="$alpha" $ABORT_FLAG $REST_FLAG --csv="$csv" 2>&1)
  if printf '%s\n' "$output" | grep -q "shaft still turning"; then
    echo "   !! the shaft was still turning after $WAIT_REST s - this run did NOT start from rest (listed in not_at_rest.txt)"; echo "$name r$r" >> "$OUT/not_at_rest.txt"
  fi
  res=$(printf '%s\n' "$output" | grep '^\[RESULT\]' | sed 's/^\[RESULT\] //')
  ab=$(printf '%s\n' "$output" | grep '^\[ABORTED\]')
  if [ -n "$ab" ]; then
    mkdir -p "$OUT/aborted"; mv -f "$csv" "$OUT/aborted/" 2>/dev/null
    echo "$name r$r ${ab#\[ABORTED\] }" >> "$ABORTS"
    echo "$name $r" >> "$DONE"
    echo "   !! SPEED LIMIT HIT - run stopped and flagged: ${ab#\[ABORTED\] }"
  elif [ -z "$res" ]; then
    echo "   !! no [RESULT] line (run failed? daemon output below)"; printf '%s\n' "$output" | tail -3
  else
    echo "$name $res" >> "$RAW"; echo "$name $r" >> "$DONE"
    LAST_JIT=$(printf '%s\n' "$res" | sed -n 's/.*pwm_jitter_pct=\([0-9.]*\).*/\1/p')
  fi
  return 0
}

# ---------------------------------------------------------------- the loop: repeats outermost, order shuffled inside each repeat
SHUF="shuf"; command -v shuf >/dev/null 2>&1 || SHUF="sort -R"
n=0; since_anchor=0; anchor_n=0
for r in $(seq 1 "$REPEATS"); do
  mapfile -t ORDERED < <(printf '%s\n' "${CONFIGS[@]}" | $SHUF)
  for cfg in "${ORDERED[@]}"; do
    read -r name kp ki alpha <<< "$cfg"
    n=$((n + 1)); since_anchor=$((since_anchor + 1))
    echo "[$n/$total] $name (kp=$kp ki=$ki alpha=$alpha) repeat $r"
    echo "$name r$r" >> "$ORDER"
    run_one "$name" "$kp" "$ki" "$alpha" "$r"
    sleep "$PAUSE"
    if [ "$ANCHOR_EVERY" -gt 0 ] && [ "$since_anchor" -ge "$ANCHOR_EVERY" ]; then
      since_anchor=0; anchor_n=$((anchor_n + 1)); n=$((n + 1))
      read -r aname akp aki aal <<< "$ANCHOR"
      echo "[$n/$total] drift check: $aname again (anchor $anchor_n)"
      LAST_JIT=""; run_one "$aname" "$akp" "$aki" "$aal" "$((100 + anchor_n))"
      [ -n "$LAST_JIT" ] && echo "   baseline PWM jitter now ${LAST_JIT} % (compare with earlier anchors; a steady drift means the motor is warming up)"
      sleep "$PAUSE"
    fi
  done
done
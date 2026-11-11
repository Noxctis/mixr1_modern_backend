#!/usr/bin/env bash
# Reynolds / Power number sweep WITH TORQUE, run once per speed estimator, straight from the daemon (no dashboard).
# This is what ties the choice of estimator to your N_Re-N_Po curve: the same sweep with DCT and with CET.
#   bash ch5kit/ch5.sh npo           (about 40 minutes)      quick: REPS=1 VARIANTS="aligned dct20" bash ch5kit/scripts/run_npo.sh
set -u
KIT="$(cd "$(dirname "$0")/.." && pwd)"
if [ ! -f kinematics.hpp ] && [ ! -f include/kinematics.hpp ]; then cd "$KIT/.." || exit 1; fi
REPS="${REPS:-2}"; VARIANTS="${VARIANTS:-aligned dct20 newest}"; HOLD="${HOLD:-30}"
if [ -z "${DBIN:-}" ]; then for c in ./mixr1_daemon ./build/mixr1_daemon; do [ -x "$c" ] && { DBIN="$c"; break; }; done; fi
[ -x "${DBIN:-/nonexistent}" ] || { echo "ERROR: cannot find mixr1_daemon"; exit 1; }
DAEMON="${DAEMON:-sudo $DBIN}"

echo "=============================================================="
echo " N_Re - N_Po SWEEP WITH TORQUE"
echo " Fit the impeller, fill the vessel to the standard volume, torque amplifier ON and warmed up (30 min), shaft free."
echo "=============================================================="
read -r -p "Impeller diameter in mm (96 or 174): " IMP; IMP="${IMP:-96}"
case "$IMP" in 96) D=0.096; ABORT=1500; SP="100,200,300,400,500,600,700,800,900,1000,1100,1200,1300";;
               174) D=0.174; ABORT=400; SP="50,100,150,200,250,300,340";; *) echo "type 96 or 174"; exit 1;; esac
read -r -p "Fluid (for example tap water): " FLUID; FLUID="${FLUID:-tap water}"
read -r -p "Fluid temperature NOW in deg C: " T0
read -r -p "Baffles fitted? (yes/no): " BAF; BAF="${BAF:-no}"; case "$BAF" in y*|Y*) TAG=baf;; *) TAG=nobaf;; esac
OUT="${OUT:-logs/ch5_new/npo_${IMP}_${TAG}}"; mkdir -p "$OUT"
TQ=""; SJ="logs/ch5_new/guided_${IMP}_${TAG}/session.json"
[ -f "$SJ" ] && TQ=$(python3 -c "import json;print(json.load(open('$SJ')).get('torque_flags',''))" 2>/dev/null)
if [ -z "$TQ" ]; then echo "Torque flags: the ones you normally add to the daemon for the torque sensor (like --torque-port=/dev/ttyUSB0 --torque-baud=9600 ...)"; read -r -p "flags: " TQ; fi
[ -z "$TQ" ] && { echo "Without torque flags there is no torque, so no N_Po. Send me the command you normally use to read the torque."; exit 1; }
KP=0.2222; KI=7.1648
for g in logs/ch5_new/gains/common.env logs/ch5_new/estimators/gains.env; do [ -f "$g" ] && { . "$g"; echo "gains from $g: Kp=$KP Ki=$KI"; break; }; done
python3 - <<EOF
import json; json.dump(dict(impeller_mm=$IMP, D_m=$D, fluid="$FLUID", T_start=float("${T0:-nan}" if "${T0:-}" else "nan"), baffles="$BAF", torque_flags="$TQ", hold_s=$HOLD, setpoints="$SP", kp=$KP, ki=$KI), open("$OUT/meta.json","w"), indent=1)
EOF
flags_for() { case "$1" in dct5) echo "--estimator=dct --window=5000";; dct10) echo "--estimator=dct --window=10000";; dct20) echo "--estimator=dct --window=20000";; aligned) echo "--estimator=aligned";; newest) echo "--estimator=newest";; esac; }
read -r -p "Ready? Press Enter to start (Ctrl-C to cancel) " _
N=0; TOTAL=$(( $(echo $VARIANTS | wc -w) * REPS ))
for rep in $(seq 1 "$REPS"); do for v in $(shuf -e $VARIANTS); do
  N=$((N+1)); name="npo-${v}_r${rep}"
  if [ -s "$OUT/$name.csv" ] && [ "$(stat -c %s "$OUT/$name.csv")" -gt 100000 ]; then echo "[$N/$TOTAL] $name (done, skipping)"; continue; fi
  echo "[$N/$TOTAL] $name"
  $DAEMON --test --cpr=1024 --fifo --no-stream --wait-rest=30 --abort-rpm=$ABORT --log-age $TQ $(flags_for "$v") \
     --pi --schedule=$SP --hold=$HOLD --kp=$KP --ki=$KI --alpha=1.0 --csv="$OUT/$name.csv" > "$OUT/$name.stdout.txt" 2>&1 || echo "   WARNING: see $OUT/$name.stdout.txt"
  sleep "${GAP:-20}"
done; done
read -r -p "Fluid temperature NOW (end of the sweep), deg C: " T1
python3 - <<EOF
import json; m=json.load(open("$OUT/meta.json")); m["T_end"]=float("${T1:-nan}" if "${T1:-}" else "nan"); json.dump(m,open("$OUT/meta.json","w"),indent=1)
EOF
echo "Done. Next:   bash ch5kit/ch5.sh npo-analyze"

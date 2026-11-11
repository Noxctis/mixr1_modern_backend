#!/usr/bin/env bash
# THE ONLY COMMAND YOU NEED:    bash ~/ch5kit/ch5.sh
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ---- find the firmware folder by itself
find_repo() {
  for d in "$KIT/.." "$PWD" "$PWD/.." ; do
    for k in "$d/include/kinematics.hpp" "$d/kinematics.hpp"; do [ -f "$k" ] && { (cd "$(dirname "$k")"; [ "$(basename "$PWD")" = include ] && cd ..; pwd); return; }; done
  done
  local h; h=$(find "$HOME" -name kinematics.hpp -not -path "*/ch5kit/*" -not -path "*/.git/*" 2>/dev/null | head -1)
  [ -n "$h" ] && { (cd "$(dirname "$h")"; [ "$(basename "$PWD")" = include ] && cd ..; pwd); }
}
REPO="$(find_repo)"
[ -z "$REPO" ] && { echo "I cannot find your firmware folder (the one with kinematics.hpp). Tell me what 'ls' shows in it."; exit 1; }
cd "$REPO" || exit 1
echo "Firmware folder: $REPO"
detect_daemon() {
  DBIN=""
  for c in ./mixr1_daemon $DBIN; do [ -x "$c" ] && { DBIN="$c"; break; }; done
  [ -z "$DBIN" ] && DBIN=$(find . -maxdepth 3 -type f -name mixr1_daemon -perm -u+x 2>/dev/null | head -1)
  [ -z "$DBIN" ] && DBIN="./mixr1_daemon"
  export DBIN
}
detect_daemon
echo "Daemon:          $DBIN"
DATA="logs/ch5_new"; mkdir -p "$DATA"
pause() { read -r -p "$1  (press Enter) " _; }

build_it() {
  local rc=1
  if [ -f CMakeLists.txt ]; then
    mkdir -p build && cd build && { cmake .. > ../build.log 2>&1 && make -j2 >> ../build.log 2>&1 || { grep -q -i "atomic" ../build.log && { echo "   retrying with -latomic"; cmake -DCMAKE_EXE_LINKER_FLAGS=-latomic .. > ../build.log 2>&1 && make -j2 >> ../build.log 2>&1; }; }; }; rc=$?; cd ..
  elif [ -f Makefile ]; then
    make -j2 > build.log 2>&1; rc=$?
  elif [ -f main.cpp ]; then
    local srcs=""; for f in main encoder ec11 kinematics motor pi_controller process_monitor lcd network; do [ -f "$f.cpp" ] && srcs="$srcs $f.cpp"; done
    echo "   no Makefile found: compiling$srcs into ./mixr1_daemon"
    g++ -std=c++17 -O2 -Wall -o mixr1_daemon $srcs -lpigpiod_if2 -lrt -lpthread -latomic > build.log 2>&1; rc=$?
  else
    echo "I do not know how you build. Tell me the command you normally use."; return 1
  fi
  detect_daemon
  if [ $rc -ne 0 ] || [ ! -x "$DBIN" ]; then echo "BUILD FAILED. Send me the last 15 lines of build.log:"; tail -15 build.log; return 1; fi
  echo "   build OK ($DBIN)"
}

do_setup() {
  echo "== 1/6 backup: the installer saves a .bak copy of every file it changes"
  echo "== 2/6 adding the DCT estimator to your firmware"; python3 "$KIT/firmware/install_dct.py" || { echo "STOP: copy the red text above and send it to me."; return 1; }
  echo "== 3/6 python tools"; if ! pip3 install --break-system-packages pandas numpy scipy matplotlib tabulate > /dev/null 2>&1; then pip3 install pandas numpy scipy matplotlib tabulate 2>&1 | tail -1; fi
  sudo apt install -y sysstat 2>&1 | tail -1
  echo "== 4/6 platform"; bash "$KIT/scripts/prepare_platform.sh"
  echo "== 5/6 building (a few minutes)"
  build_it || return 1
  echo "== 6/6 checking the encoder"; echo "   Set the encoder DIP switch to 0011 (256 PPR). Nothing touching the shaft. The motor will spin 8 seconds."; pause "Ready?"
  sudo $DBIN --test --cpr=1024 --fifo --no-stream --no-torque --no-pi --pwm=614 --duration=8 --log-age --csv="$DATA/smoke.csv" > "$DATA/smoke.stdout.txt" 2>&1
  python3 "$KIT/scripts/check_preflight.py" "$DATA/smoke.stdout.txt" && echo "SETUP FINISHED. Now choose 2." || echo "Setup finished but the encoder check failed: fix what it says, then choose 1 again."
}

do_use() {   # bash ch5.sh use newest|aligned
  local want="$1"; local cfg; cfg=$(ls include/config.hpp config.hpp 2>/dev/null | head -1)
  [ -z "$cfg" ] && { echo "config.hpp not found"; return 1; }
  case "$want" in newest) v=true;; aligned) v=false;; *) echo "use:  bash ch5.sh use newest   or   bash ch5.sh use aligned"; return 1;; esac
  cp "$cfg" "$cfg.bak2"; sed -i "s/inline bool ESTIMATOR_NEWEST = [a-z]*;/inline bool ESTIMATOR_NEWEST = $v;/" "$cfg"
  grep -n "ESTIMATOR_NEWEST =" "$cfg" | head -1; build_it && echo "The default estimator is now: $want"
}
do_gains() {
  echo "GAINS FOR BOTH IMPELLERS. The firmware has ONE Kp and ONE Ki, so each impeller is measured and then ONE pair that suits both is computed."
  echo "   1 = measure the motor with the impeller that is fitted NOW (96 or 174)       about 3 minutes"
  echo "   2 = compute the ONE pair for both impellers and write it into the firmware   1 minute + rebuild"
  echo "   3 = check the pair with the impeller that is fitted NOW                      45 seconds"
  read -r -p "type 1, 2 or 3: " g
  mkdir -p "$DATA/gains"
  case "$g" in
    1) read -r -p "Which impeller is fitted now? type 96 or 174: " IMP
       case "$IMP" in 96) SCH="0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60"; AB=1500;; 174) SCH="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20"; AB=400;; *) echo "type 96 or 174"; return 1;; esac
       echo "Vessel filled to the standard volume, the $IMP mm impeller fitted, shaft free to turn."; pause "Ready?"
       sudo $DBIN --test --cpr=1024 --fifo --no-stream --no-torque --wait-rest=30 --no-pi --abort-rpm=$AB --schedule=$SCH --hold=10 --csv="$DATA/gains/plant_$IMP.csv" > "$DATA/gains/plant_$IMP.stdout.txt" 2>&1
       python3 "$KIT/scripts/analyze_sweep_fixed.py" "$DATA/gains/plant_$IMP.csv" --bw 5 --out "$DATA/gains/plant_$IMP" --emit-env "$DATA/gains/plant_$IMP.env" | tail -9
       echo "Saved for the $IMP mm impeller. Swap to the other impeller and choose 1 again, then choose 2.";;
    2) args=""; [ -f "$DATA/gains/plant_96.env" ] && args="$args --plant96 $DATA/gains/plant_96.env"; [ -f "$DATA/gains/plant_174.env" ] && args="$args --plant174 $DATA/gains/plant_174.env"
       [ -z "$args" ] && { echo "Do choice 1 first (for each impeller)."; return 1; }
       [ -f "$DATA/gains/plant_96.env" ] && [ -f "$DATA/gains/plant_174.env" ] || echo "NOTE: only one impeller measured so far; the pair will fit that one only."
       python3 "$KIT/scripts/combine_gains.py" $args --out "$DATA/gains/common.env" || return 1
       . "$DATA/gains/common.env"
  local cfg; cfg=$(ls include/config.hpp config.hpp 2>/dev/null | head -1); cp "$cfg" "$cfg.bak3"
  sed -i "s/\\(inline double GLOBAL_KP = \\)[0-9.]*;/\\1$KP;/; s/\\(inline double GLOBAL_KI = \\)[0-9.]*;/\\1$KI;/; s/\\(inline double FEEDBACK_ALPHA = \\)[0-9.]*;/\\11.0;/" "$cfg"
  grep -n "GLOBAL_KP =\\|GLOBAL_KI =\\|FEEDBACK_ALPHA =" "$cfg"
       build_it && echo "The firmware now uses Kp=$KP Ki=$KI for both impellers. Next choose 3 for each impeller.";;
    3) read -r -p "Which impeller is fitted now? type 96 or 174: " IMP
       case "$IMP" in 96) LO=300; HI=400; AB=1500;; 174) LO=150; HI=250; AB=400;; *) echo "type 96 or 174"; return 1;; esac
       pause "Impeller $IMP fitted, water in. The test goes $LO -> $HI -> $LO rpm (45 s)."
       sudo $DBIN --test --cpr=1024 --fifo --no-stream --no-torque --wait-rest=30 --abort-rpm=$AB --pi --target=$LO --ref-step-to=$HI --ref-step-at=15 --ref-step-to2=$LO --ref-step-at2=30 --duration=45 --csv="$DATA/gains/check_$IMP.csv" > "$DATA/gains/check_$IMP.stdout.txt" 2>&1
       python3 "$KIT/tools/ch5/analyze_ch5.py" steps --csv "$DATA/gains/check_$IMP.csv" --out "$DATA/gains/check_$IMP" | tail -12
       echo "GOOD if the overshoot is under about 10 % and the speed settles. Do this for BOTH impellers. If either is bad, send me $DATA/gains/ .";;
    *) echo "type 1, 2 or 3";;
  esac
}

do_estimators() {
  echo "How much time do you have?   (more runs = better proof that the differences are real, not luck)"
  echo "   1 = QUICK    about 15 min   DCT 20 ms, CET aligned, CET newest, once, short runs, no new-gain runs"
  echo "   2 = SHORT    about 35 min   the same three, twice each, short runs, with the new-gain runs"
  echo "   3 = MEDIUM   about 45 min   all five (DCT 5, 10, 20 ms and both CET), once, normal runs"
  echo "   4 = FULL     about 2 hours  all five, three repeats each"
  read -r -p "type 1, 2, 3 or 4: " m
  case "$m" in
    1) export VARIANTS="dct20 aligned newest" REPS=1 HOLD_A=9 HOLD_B=12 HOLD_P=8 SKIP_IMC=1 GAP="${GAP:-10}" SCHED_B="25,100,250,350,500,1000,1500";;
    2) export VARIANTS="dct20 aligned newest" REPS=2 HOLD_A=9 HOLD_B=12 HOLD_P=8 GAP="${GAP:-10}" SCHED_B="25,100,250,350,500,1000,1500";;
    3) export VARIANTS="dct5 dct10 dct20 aligned newest" REPS=1 GAP="${GAP:-10}";;
    *) export REPS=3;;
  esac
  bash "$KIT/scripts/run_estimators.sh"
}
do_analyze()    { python3 "$KIT/scripts/analyze_runs.py" --dir "$DATA/estimators"; }
do_ppr() {
  EST=aligned; grep -q "ESTIMATOR_NEWEST = true" include/config.hpp config.hpp 2>/dev/null && EST=newest
  echo "The PPR study will use the CET estimator you chose: $EST   (change it with:  bash ch5.sh use newest|aligned)"
  echo "Can you change the encoder DIP switch WITHOUT taking the encoder's top cover off (or are you doing this on a spare encoder)?"
  read -r -p "type y or n then Enter: " ans
  if [ "$ans" = "y" ]; then
    echo "   1 = SHORT  about 1.1 hours   PPR 48, 125, 256, 512, two repeats, 14 runs"
    echo "   2 = FULL   about 3.1 hours   PPR 48, 100, 125, 250, 256, 512, 1024, three repeats, 39 runs"
    read -r -p "type 1 or 2: " L
    if [ "$L" = "1" ]; then python3 "$KIT/tools/ppr/plan_runs.py" --out "$DATA/ppr" --current-ppr 256 --settings 48,125,256,512 --blocks 2 --b-settings 256
    else python3 "$KIT/tools/ppr/plan_runs.py" --out "$DATA/ppr" --current-ppr 256; fi
  else
    echo "The datasheet allows the top cover to come off only 3 times, so the plan uses exactly 3 (256 > 48 > 512 > 256)."
    echo "   1 = SHORT  about 0.6 hours   PPR 48, 256, 512, two repeats, 8 runs"
    echo "   2 = FULL   about 1.6 hours   PPR 48, 256, 512, three repeats, 18 runs"
    read -r -p "type 1 or 2: " L
    if [ "$L" = "1" ]; then python3 "$KIT/tools/ppr/plan_runs.py" --out "$DATA/ppr" --design blocked --settings 48,256,512 --blocks 2 --current-ppr 256 --ref-every 9 --b-settings 256
    else python3 "$KIT/tools/ppr/plan_runs.py" --out "$DATA/ppr" --design blocked --settings 48,256,512 --current-ppr 256 --ref-every 9; fi
  fi
  pause "Look at the table above. About to warm the motor for 5 minutes (shaft free, NO impeller)"
  python3 "$KIT/tools/ppr/ppr_campaign.py" warmup --dir "$DATA/ppr" --daemon "sudo $DBIN"
  echo "Now the campaign. When it says SET THE DIP SWITCH TO xxxx do that, switch the encoder power off and on, press Enter."
  python3 "$KIT/tools/ppr/ppr_campaign.py" run --dir "$DATA/ppr" --estimator "$EST" --daemon "sudo $DBIN" --repo .
  echo "Putting the rig back: SET THE DIP SWITCH TO 0011 and switch the encoder power off and on."; pause "Done?"
  python3 "$KIT/tools/ppr/ppr_campaign.py" preflight --ppr 256 --dir "$DATA/preflight" --daemon "sudo $DBIN"
}
do_ppr_analyze() {
  [ -f "$KIT/tools/ppr/analyze_ppr_study.py" ] && python3 "$KIT/tools/ppr/analyze_ppr_study.py" --dir "$DATA/ppr" --ref 256 --tach-res 1 --tach-acc-pct 0.05
  python3 "$KIT/scripts/analyze_ppr.py" --dir "$DATA/ppr"
}
do_npo()         { bash "$KIT/scripts/run_npo.sh"; }
do_npo_analyze() {
  read -r -p "Which impeller? type 96 or 174: " IMP
  read -r -p "Torque accuracy from the datasheet in N.m (or press Enter if you do not know): " ta
  if [ -n "$ta" ]; then python3 "$KIT/scripts/analyze_npo.py" --dir "$DATA/npo_$IMP" --tq-acc-nm "$ta"; else python3 "$KIT/scripts/analyze_npo.py" --dir "$DATA/npo_$IMP"; fi
}
do_guided() {
  read -r -p "Which impeller is fitted now? type 96 or 174 (the guided tests use different speed limits for each): " IMP
  echo "When it asks 'Impeller diameter in mm' type $IMP."
  python3 "$KIT/tools/ch5/ch5_guided.py" --daemon "$DBIN" --dir "$DATA/guided_$IMP"
}
do_backup() { f="$HOME/ch5_data_$(date +%Y%m%d_%H%M).tar.gz"; tar czf "$f" "$DATA" && echo "Saved $f. Copy it to your laptop:  scp pi@<PI-IP>:$f ."; }
do_status() {
  echo "daemon:          $DBIN"
  echo "estimator runs:  $(ls $DATA/estimators/est-*.csv 2>/dev/null | wc -l) files (full study = 45)"
  echo "N_Re-N_Po runs: 96 mm $(ls $DATA/npo_96/npo-*.csv 2>/dev/null | grep -v _index | wc -l) files, 174 mm $(ls $DATA/npo_174/npo-*.csv 2>/dev/null | grep -v _index | wc -l) files"
  echo "gains measured: $(ls $DATA/gains/plant_*.env 2>/dev/null | tr '\n' ' ')  common pair: $(cat $DATA/gains/common.env 2>/dev/null | tr '\n' ' ')"
  echo "PPR runs:        $(ls $DATA/ppr/mixr1_log_*test[AB].csv 2>/dev/null | wc -l) files"
  echo "guided runs:     96 mm $(cat $DATA/guided_96/index.csv 2>/dev/null | tail -n +2 | wc -l), 174 mm $(cat $DATA/guided_174/index.csv 2>/dev/null | tail -n +2 | wc -l) entries"
  ls "$DATA/estimators/results/RESULTS.txt" "$DATA/ppr/results_simple/RESULTS.txt" 2>/dev/null
}
menu() {
cat <<'EOF'

==================== CHAPTER 5: pick a number, then Enter ====================
 1  SETUP      (do first, once)              about 10 min   - you set the DIP switch
 2  ESTIMATORS DCT versus CET (automatic)    about 2 hours  - motor FREE (no impeller, no water)
 3  ANALYSE    estimators                    1 minute       - accuracy, precision, which CET to use
 4  GAINS      controller gains for the job  about 12 min   - impeller FITTED, vessel filled
 5  PPR STUDY  encoder resolution            about 3 hours  - you change the DIP switch when told
 6  ANALYSE    PPR study                     1 minute
 7  N_Re-N_Po  sweep WITH TORQUE, per estimator  about 40 min - impeller fitted, water, torque on
 8  ANALYSE    N_Re-N_Po                     1 minute       - how DCT/CET change your Power number
 9  GUIDED     everything else (menu)        3-4 days       - tachometer, weights, water
10  BACKUP     pack all data into one file
11  STATUS     what is done
 0  quit
==============================================================================
EOF
}
case "${1:-}" in
  npo) do_npo;; npo-analyze) do_npo_analyze;; setup) do_setup;; estimators) do_estimators;; analyze) do_analyze;; use) do_use "${2:-}";; gains) do_gains;; ppr) do_ppr;; ppr-analyze) do_ppr_analyze;; guided) do_guided;; backup) do_backup;; status) do_status;;
  *) while true; do menu; read -r -p "number: " c; case "$c" in 1) do_setup;; 2) do_estimators;; 3) do_analyze;; 4) do_gains;; 5) do_ppr;; 6) do_ppr_analyze;; 7) do_npo;; 8) do_npo_analyze;; 9) do_guided;; 10) do_backup;; 11) do_status;; 0) exit 0;; *) echo "type one of the numbers";; esac; done;;
esac

# START HERE: copy and paste only (both impellers, short options, and why each step exists)

Folder: `/home/pi/mixr1_modern_backend`   Kit: `/home/pi/mixr1_modern_backend/ch5kit`
If anything says STOP, FAIL or BUILD FAILED, copy the red text and send it to me. Do not fix it yourself.
Never open the dashboard unless the encoder switch is at **0011**.

## STEP 0: put the newest kit on the Pi (once)
On the Pi: `hostname -I` (prints the address, like 192.168.1.23). On your laptop, where `ch5kit.zip` is:
```
scp ch5kit.zip pi@192.168.1.23:~/
```
On the Pi:
```
cd /home/pi/mixr1_modern_backend && unzip -o ~/ch5kit.zip
```

## STEP 1: the menu (every time)
```
bash /home/pi/mixr1_modern_backend/ch5kit/ch5.sh
```
You type a number and press Enter. The menu:
```
 1 SETUP        2 ESTIMATORS (DCT vs CET)   3 ANALYSE        4 GAINS (both impellers)
 5 PPR STUDY    6 ANALYSE PPR               7 N_Re-N_Po+torque   8 ANALYSE N_Re-N_Po
 9 GUIDED       10 BACKUP                   11 STATUS
```


## IF YOUR SSH CONNECTION DROPS
- The menu now runs inside **tmux** automatically (it installs it the first time). Your run keeps going on the Pi even if your laptop sleeps or the Wi-Fi drops.
- To get back to it, log in again and type:
```
tmux attach -t ch5
```
- Do not press Ctrl-C to "leave"; to leave the screen without stopping anything press **Ctrl-B, then D**.
- If you start a script by hand (not through the menu), type `tmux new -s ch5` first, then run it inside.
- Safety: if a run is killed anyway, the daemon stops the motor by itself (it handles the hang-up signal). That run's file is incomplete; menus 2, 5 and 7 skip finished runs and redo the rest when you start them again.

## HOW LONG? You choose. More runs = stronger proof that a difference is real and not luck.
| Menu | Short | Full |
|---|---|---|
| 2 DCT vs CET | **QUICK 15 min** (3 estimators once) or **SHORT 35 min** (twice each) | MEDIUM 45 min (all 5 once), FULL 2 h (all 5, three repeats) |
| 5 PPR | **SHORT 0.6 to 1.1 h** | FULL 1.6 to 3.1 h |
Menu 2 and 5 ask you which one. Minimum honest set: SHORT in both. You can always run a longer one later; it only adds what is missing.

---

# DAY 1: motor FREE. Impeller OFF, no water.

**1 SETUP.** Type `1`. When asked, set the DIP switch to 0 0 1 1 (switches 1 2 3 4 = off off on on), switch the encoder power off and on, Enter. Expect `PASS`, `SETUP FINISHED`.

**2 DCT vs CET.** Type `2`, then choose 1, 2, 3 or 4 (time table above). Press Enter once and leave it.
*Why:* this answers "is CET accurate, and how does it differ from the frequency-based DCT?". Every run is compared with the **index pulses** (one per revolution, timed to 5 microseconds, independent of the quadrature, the PPR and every estimator), which gives **accuracy** (bias), not only scatter. It also gives: precision (SD), closed-loop behaviour with your old gains (does DCT oscillate?), DCT at 5, 10, 20 ms (is the failure delay or noise?), and the age of the CET estimate (aligned vs newest).

**3 ANALYSE.** Type `3`. The last lines say `RECOMMENDED ESTIMATOR: newest` or `aligned`.
If **newest**, paste: `bash /home/pi/mixr1_modern_backend/ch5kit/ch5.sh use newest` (it rebuilds).
The same analysis is your **FFT / shaft** evidence (below). Send me `logs/ch5_new/estimators/results/RESULTS.txt` and the `.pdf` files in that folder.

---

# DAY 2: motor FREE again. Impeller OFF.

**5 PPR STUDY (under the CET you chose).** Type `5`. It prints which CET it will use. Questions:
- "Can you change the DIP switch without removing the encoder top cover (or are you on a spare encoder)?" `y` or `n`. (The datasheet allows the cover off only 3 times; `n` uses exactly 3.)
- Then `1` = SHORT or `2` = FULL.
When it says **SET THE DIP SWITCH TO xxxx**: set switches 1 2 3 4 in that order (1 = on), switch the encoder power off and on, Enter. It checks the setting itself. In test B it prints `>>> READ TACH NOW`: put the handheld tachometer on the shaft, type the number, Enter (3 times per speed). At the end put the switch back to **0011**.
*Why:* it shows, for each PPR, edges per revolution, lost edges, scatter, the shaft line, the **bias against the index speed** (what resolution does to accuracy under CET), the update fraction, CPU load, and the tachometer check. This is the backing for choosing 256 PPR.

**6 ANALYSE PPR.** Type `6`. Send me `logs/ch5_new/ppr/results_simple/RESULTS.txt`.

---

# DAY 3 AND 4: every condition the rig is used in. Fill the vessel to the standard volume. Torque amplifier ON 30 minutes before.

The firmware has ONE Kp and ONE Ki, so the kit measures each condition (impeller and baffles in or out) and then finds **one pair that works for all of them**.

**Which conditions?** The lightest (96 mm, baffles OUT) and the heaviest (174 mm, baffles IN) are the two that matter most, because the pair has to be stable on the light one and acceptably fast on the heavy one. Measuring all four is better (about 4 minutes each). If you measure only two, check the other two with choice 3.

**A. In each condition (set it up first):**
1. `4`, then `1`. Type the impeller (`96` or `174`), then `y` if the baffles are IN, `n` if OUT, then Enter.
   The duty goes from 0 to 100 % in steps of 10 %, 10 s each, and the test **stops by itself** when the speed passes 1400 rpm (96 mm) or 400 rpm (174 mm). From your dashboard sweeps that happens at about 90 % duty for the 96 mm impeller and about 80 % for the 174 mm impeller, so the range covers 1350 rpm and 350 rpm.
   It prints the plant at low, mid and high speed. (The gain K falls as the speed rises, because the drag of the water grows; your 96 mm data: 0.48 rpm per count at 290 rpm, 0.34 at 800 rpm.)

**B. When all conditions are measured:**
2. `4`, then `2`. It computes ONE pair for all of them, prints for each condition the crossover, the phase margin and the gain margin, and what the compromise costs the slowest condition, writes the pair into the firmware and rebuilds. **Send me what it prints.**
3. In each condition: `4`, then `3` (45 second check). You want overshoot under about 10 % and a speed that settles.
4. `4`, then `4` lists what has been measured.

**C. Then the N_Re-N_Po sweep with torque, once per condition you want in the thesis:**
5. `7` (40 minutes): type the impeller, fluid `tap water`, temperature, baffles `yes`/`no`, torque flags. At the end type the temperature again.
6. `8`: type the impeller and `y`/`n` for baffles, then the torque accuracy from the datasheet (or Enter). Send me `logs/ch5_new/npo_96_nobaf/results/`, `npo_96_baf/`, `npo_174_nobaf/`, `npo_174_baf/` (whichever you ran).
7. `9` (guided, see below): it also asks the impeller and the baffles.

*Why 7 and 8:* your Power number is `N_Po = 2 pi tau / (rho n^2 D^5)`, so a speed error of x % gives about **2x %** in N_Po. The sweep runs once per estimator and also uses the index speed, so you see in your own numbers what DCT and CET do to the curve, and whether speed or torque dominates the uncertainty.

---

# THE GUIDED MENU (`9`), once per condition you want in the thesis (the main one first)
It asks the session questions. Type:
| It asks | Type |
|---|---|
| Impeller diameter | `96` or `174` (the one fitted) |
| Fluid | `tap water` |
| Fluid temperature, room temperature, supply voltage | the numbers |
| Liquid level | what you filled, like `5 L` |
| Baffles | `no` or `yes` |
| Torque flags | the flags you normally add to the daemon to read the torque sensor (no flags = no torque tests) |

Then type these in order: `2` (8 s spin), `3` (duty sweeps), `4` (step response), `5` (small steps), `6` (power limiter: look for `[SAFETY] ... power limit reached`), `7` (load disturbance: ONE routine, same every run, Enter at each disturbance), `13` (analyse), `14` (what is missing), `0`.
Do these **only once (with either impeller)**, they do not depend on the impeller: `8` (old CEK-A knob: label, nominal speed, 3 tachometer numbers), `9` (torque cold start: amplifier OFF for hours first, 90 min, do not touch the shaft), `10` (torque with weights: lever arm in metres, `up` then `down`, 3 times, start every series with 0 kg), `11` (endurance: 120 minutes, 350 rpm), `15` (Mode 1 checklist `p`/`f`/`n`).

**Dashboard (by hand, encoder at 0011):** run the automatic N_Re-N_Po sequence 3 times per impeller, same settings; copy the `..._sequence_summary.csv`, `..._sequence_log.csv` and `.mat` into `/home/pi/mixr1_modern_backend/logs/ch5_new/guided_96/` (or `guided_174`), then guided menu `12`. Integrity run (30 minutes, dashboard connected, then export `.csv` and `.mat`):
```
cd /home/pi/mixr1_modern_backend && sudo ./mixr1_daemon --test --cpr=1024 --fifo --pi --target=350 --duration=1800 --stream --ext --wait-client --csv=logs/ch5_new/guided_96/integrity_daemon.csv
```
Screenshots of the dashboard.

**End of every day:** `10` (backup) and copy the file it names to your laptop: `scp pi@192.168.1.23:~/ch5_data_YYYYMMDD_HHMM.tar.gz .`

---

# YOUR SHAFT QUESTION (FFT)
You think the shaft, which rides on the encoder, affects the readings. Menu 3 tests this three ways, from the same free-motor data as step 2:
1. **FFT**: lines at 1x, 2x, 3x the rotation frequency. Table `SHAFT / ECCENTRICITY`: if it is an **angle error of the encoder** (shaft not concentric), the relative amplitude A1/n stays the same at every speed and the equivalent angle stays under 0.25 degrees; if it is a **true speed ripple** of the shaft, A1 in rpm falls as the speed rises.
2. **Folded against shaft angle**: every sample is placed at its angle in the revolution using the index pulses. A shaft effect has the **same phase against the index** at every speed. Table `RIPPLE VERSUS SHAFT ANGLE`. Its phase slope also measures each estimator's **delay**.
3. **Dial indicator** (you, on paper): shaft runout at the encoder in mm. The predicted angle error is runout divided by the sensing radius.
The ripple is the same in DCT and CET, so it is not an estimator effect; the index speed averages whole revolutions, so it cancels the ripple exactly. That is the argument.

---

# ON PAPER (no computer)
1. Nameplates: motor (V, W, rpm, gear ratio, torque), driver (A), torque sensor (range, accuracy in N m), tachometer (resolution, accuracy).
2. Level converter pull-up value; which 5 V rail feeds the encoder.
3. **Both impellers: type, diameter, number and width of blades, clearance from the floor; vessel diameter and liquid height; baffles.** (Your thesis chapters do not mention the 174 mm impeller yet.)
4. Photos: encoder mounting, torque sensor installed, front panel.
5. Oscilloscope, 20 minutes: encoder A, B, index on the Pi side of the level converter, motor at two speeds: rise time, fall time, duty cycle, phase shift; PWM at the driver input.
6. Dial indicator: shaft runout at the encoder (mm).
7. Turn the shaft slowly by hand the way you call positive: which way, seen from the front? And `grep ENCODER_DIRECTION /home/pi/mixr1_modern_backend/config.hpp`: your preflight shows counts going down with positive speed, but the config you uploaded says 1.
8. Old files: the feedback alpha and window of `cet_1500rpm.csv` and `dct_pwm_sweep.csv`; the PPR of `cet_pwm_sweep.csv`; what the stress test did; whether the encoder was re-seated between the two runs of 4 September.

---

# SEND ME
| After | Send |
|---|---|
| 3 | `logs/ch5_new/estimators/results/RESULTS.txt` + the `.pdf` figures there |
| 6 | `logs/ch5_new/ppr/results_simple/RESULTS.txt` |
| 4 (choice 2) | what it printed |
| 8 (each impeller) | `logs/ch5_new/npo_96/results/` and `npo_174/results/` (`RESULTS.txt`, `fig_npo.pdf`) |
| guided (each impeller) | `logs/ch5_new/guided_96/analysis/`, `guided_174/analysis/` and their `index.csv` |
| paper | your answers |
Copy a file to your laptop: `scp pi@192.168.1.23:/home/pi/mixr1_modern_backend/logs/ch5_new/estimators/results/RESULTS.txt .`

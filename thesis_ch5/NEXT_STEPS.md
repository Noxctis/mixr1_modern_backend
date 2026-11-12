# Chapter 5: what you still have to do

Every item is also a `% TODO` in `chapter5.tex` (search for `TODO`). They are grouped here by what they need. Do group A first: those change numbers or claims in the chapter.

## A. Experiments and measurements (they change the results)

1. **Test the present gains against disturbances.** All disturbance runs (8, 9, 14, 15, 18 Sep) used the earlier gains, so the chapter cannot yet show that the identified-plant gains (Kp 0.233, Ki 7.51) or the firmware default (2.629, 12) reject a pour or the baffles. Repeat the pour test (open loop vs PI at 460 rpm on the 174 or 96 mm impeller, not the 202 mm) and the baffle test with the present gains, CET estimator, and the slew limit on. This is the main open comparison.
2. **Repeat the pour test cleanly.** In the 9 Sep run the speed fluctuated for 60 s (SD 13.6 rpm) after the first pour. Pour the same volume each time, one pour per 30 s, record the volume, and repeat with the present gains to see whether the fluctuation was the pouring or the fast loop.
3. **Measure the water temperature** at every run; put it in the run notes. Then either use the Tanaka (density) and Kestin/Vogel (viscosity) correlations in the daemon and dashboard or state the temperature that 996.3 kg/m3 and 0.854 mPa s are for (about 27 C; one note says 24 C, which would lower N_Re by 6 %).
4. **Calibrate the torque sensor with a stated uncertainty.** Record the balance and calliper used for the masses and the disc radius (24.77 mm), how the cord is fixed, and extend the load to 320 mN m (the 174 mm impeller reaches it; your test stopped at 243). Keep the calibration certificate that ships with the sensor. Also log the unloaded reading from cold start for an hour or more (the TC-U3 test) for the drift section.
5. **Run-to-run spread of N_Po (up to 20 %).** Find the cause (torque zero, liquid state, temperature), or state it as a limit. The two worst 96 mm runs and the slow-rising runs are listed in the plausibility section.
6. **Revolution count** is 9-39 % low in 7 of the 14 sequence logs: check the index wire and the rejected/missed counts the daemon prints at the end of a run.
7. **Oscilloscope measurements**: rise time of the converter outputs at the GPIO pins (replaces the assumed 20-100 pF), the duty cycle of A and B, the current ripple at 20 kHz, and the encoder runout if you want to separate encoder error from ripple.
8. **Endurance run** (TC-S2, TC-S3): continuous run at a representative setpoint; report uptime, loop-period distribution, temperature, torque zero drift.
9. **Mode 1 on the hardware**: run every row of the Mode 1 table and fill the Result column; install the run logger patch on the Pi and check the files of a short run and of a trip. Also record the longest display redraw (`[TIMING]` line) and the loop-period distribution with the TFT on.
10. **PPR with CET**: if you want the resolution choice defended with the final estimator, repeat the sweep at 125, 250 and 256 PPR with CET (data kit menus 5 and 6). Otherwise state that the choice rests on the analysis and the DCT data.
11. **Plants of the two impellers on the loaded rig** (data kit menu 4): K, A and dead time with the vessel filled, baffles in, for the 96 and 174 mm, and the closed-loop check of the common pair on both. The chapter has only the 96 mm plant and the 174 mm fits from file `baseline_gains.txt`.

## B. Information only you have (no experiment needed)

12. **Impeller geometry**: type, blade width, and what "96" and "174 mm" measure against the diameters used in N_Re (0.0665 and 0.1207 m); baffle width and wall clearance. (Already in the chapter: vessel 297 mm, 2 blades, 4 baffles, depth 200 mm, clearance 50 and 90 mm.)
13. **CFD set-up**: solver, mesh, turbulence model, boundary conditions, convergence, who ran it. Without it the CFD comparison cannot be defended.
14. **Armfield data source** (document, impeller geometries) and the **BibTeX entries** for Brown et al. (curve 10, and the geometry of that configuration) and for the Armfield report.
15. **Run notes**: the gains of the 0.5 rad/s run (the chapter assumes the earlier rule, Kp 0.78, Ki 20.6, computed, not logged), the volume and manner of the pours, the cause of the excluded runs.
16. **Motor and driver ratings** (DOGA motor, VNH5019): voltage, power, speed, gear ratio, torque, current. The proposal figure of 80-100 W / 0.5-1.0 N m must be reconciled with the as-built motor.
17. **Which 5 V supplies the encoder** and the converter HV pin; which board pins the two enables (GPIO 18, GPIO 14) go to; the converter is the SparkFun board sold as MakerLab SKU MLE00152 (cited from its schematic).
18. **Encoder direction** (`ENCODER_DIRECTION` -1 vs +1) and which way the shaft turns for positive speed; the 2500 rpm top speed from the open-loop sweep.
19. **Firmware fixes**: the 72 W in `mixr_dashboard.py` and in comments of `config.hpp` (values in force: 90 W limit, 81 W aim); `RPM_SAMPLE_WINDOW_US` is not read by the estimator (remove it); the comment saying 250 ms for the display refresh (the constant gives 100 ms).

## C. Figures and photographs

20. Photographs: encoder mounting (Fig 5.2), torque sensor in the shaft line with the disc and cord (Fig 5.7), the Mode 1 unit, and a dashboard screenshot. Block diagrams are in the chapter as TikZ.
21. Schematic (Appendix) and BOM entries: add the level converter (PHP 75).

## D. Writing that depends on other chapters

22. Appendix calculation example (one setpoint, unit conversion), the CEK-A knob-positioning comparison (headline result for the validation chapter), the cross-references to Chapters 3, 4 and 8 for the 174 mm impeller and the geometry.
23. Before submission: re-run every script in `analysis/` and compare the numbers in the chapter with their output; verify every new bib entry (volume, pages, revision, URL). Then delete or resolve every `% TODO`.

## What is already done and defended

Estimator comparison (CET vs DCT) with bias, scatter and age; plant identification; the gain rule with its textbook source (SIMC/IMC); margins and Ms; the 80-setting matrix on the 174 mm; the pour and baffle comparison (earlier gains); torque sensor specification and the bench test; the uncertainty of N_Po by the datasheet and by the bench test; the Mode 1 and Mode 2 descriptions from the code; the CFD, Armfield and literature comparisons with RMSE. `PROVENANCE.md` says where each number comes from.

## E. Added after reading the amplifier manual and Modbus sheet

24. **Amplifier settings and calibration**: write down FIL, Sd, Id, collecting speed (60 or 120 per s), power-on zero (SZE), baud and stop bit, and how it was calibrated (weight procedure or digital calibration from the sensor's mV/V). The manual wants a calibration load of at least half the full scale (ATO support says 10 %); your bench test reached 12 %. Redo or justify.
25. **RS485 wiring**: the protocol sheet advises shielded twisted pair, no star connections, and a terminating resistor; say what you used.
26. The manual and the protocol sheet disagree on baud codes and on address and filter ranges (the unit may follow either): state which one your amplifier matches.
27. The N_Po uncertainty now includes the amplifier (sensor 0.3 % FS and amplifier 0.2 % FS plus 1 count, in quadrature: 7.3 mN m), giving about 9.6 % (96 mm) and 4.3 % (174 mm) at the plateau. A 0.5 N m sensor would reduce it for the 96 mm impeller.

## F. Record integrity (patches in ../patches/)

28. Apply `patches/telemetry_integrity_daemon.patch` (in `mixr1_modern_backend`) and `patches/telemetry_integrity_dashboard.patch` (in `Research SeBoCa Test`) with `git apply`; rebuild the daemon. Both patches pass `git apply --check`; the dashboard compiles and its integrity report was tested on a log with injected gaps; the daemon change is not compiled or run.
29. Re-record the N_Re-N_Po sequences with the new version: the new logs carry the daemon clock and lost periods, and each export has an `_integrity.txt`. The logs of 5-7 Nov cannot prove they are complete.
30. Test with a deliberately slow dashboard to see the discarded-line count and the lost periods.
31. Find why the revolution count is low in 7 of the 14 logs (index wire, rejected/missed counts printed at the end of a run); the integrity report now warns when it differs from the speed by more than 2 %.

## G. How to apply the three patches, step by step

The patches are in `patches/` of the worktree (`mixr1_modern_backend/.claude/worktrees/seboca-research-analysis-d23e12/patches/`). They are text diffs that `git apply` applies to your real files. **None is applied yet.** Do them in this order, in a terminal (Git Bash or PowerShell) on your PC.

Set the folders once (Git Bash):
```bash
W="/c/Users/Chrys Sean Sevilla/Desktop/School Stuff/mixr1_modern_backend/.claude/worktrees/seboca-research-analysis-d23e12"
D="/c/Users/Chrys Sean Sevilla/Desktop/School Stuff/mixr1_modern_backend"
R="/c/Users/Chrys Sean Sevilla/Desktop/School Stuff/Research SeBoCa Test"
```

**1. Mode 1 run log (saves .csv and .mat when you press STOP).** In the daemon folder:
```bash
cd "$D"
cp "$W/include/session_log.hpp" include/session_log.hpp
git apply --check "$W/patches/mode1_session_log.patch" && git apply "$W/patches/mode1_session_log.patch"
```
It changes `src/main.cpp` and `include/config.hpp` and adds `include/session_log.hpp`. If `--check` complains about whitespace, add `--ignore-whitespace`.

**2. Telemetry integrity, daemon side (queues the lines).** Same folder:
```bash
git apply --check "$W/patches/telemetry_integrity_daemon.patch" && git apply "$W/patches/telemetry_integrity_daemon.patch"
```
It changes `src/network.cpp` and `include/network.hpp`.

**3. Telemetry integrity, dashboard side (daemon clock, lost periods, integrity report).** In the dashboard folder (its own git repository):
```bash
cd "$R"
git apply --check "$W/patches/telemetry_integrity_dashboard.patch" && git apply "$W/patches/telemetry_integrity_dashboard.patch"
python -m py_compile mixr_dashboard.py && echo OK
```
Commit nothing until you have tested.

**4. Build and test the daemon on the Raspberry Pi.** Copy `src/`, `include/` and the `Makefile` to the Pi (or `git pull` there), then:
```bash
make            # needs g++ 8 or newer; if it says std::filesystem is missing, add -lstdc++fs to LDFLAGS in the Makefile
sudo ./mixr1_daemon
```
Test Mode 1: choose the impeller, start with KO, turn the knob, stop with KO. The terminal prints `[LOG] Mode 1 run ended ... saving N rows` and `[LOG] Saved mode1_logs/mixr1_..._mode1.csv`. Copy the `mode1_logs/` folder to your PC and open the `.mat` in MATLAB (`load`) to check it.

**5. Test the integrity changes.** Run a sequence from the dashboard; the export now writes `*_integrity.txt` beside the CSV. For a fault test, drag the dashboard window during a run and look at the report (lost periods) and at the daemon's terminal (`lines were discarded`).

If a patch does not apply, do not force it: send me the error and I will rebase it on your current file.

## H. Found while reading the proposal manuscript

32. The proposal gives the MIXR-1 tank as 200 mm inner diameter (9 L, scaled from the 288 mm CEK-A); the chapter now says 297 mm (from you). Reconcile, and state which impeller/baffle sizes follow from it.
33. The proposal lists a DOGA 63 W, 24 V, 3000 rpm motor, a Same Sky AMT103-V encoder, an ATO-DYN-103 torque sensor with an ADS1115 ADC, a Cytron MD20A driver and a VL53L0X level sensor. The built rig uses an AMT102-V, the ATO-TQS-S03 with the RS485 amplifier ATO-LCTR-OAR, and a VNH5019 driver. List these differences in the chapter or in the validation chapter, and give the motor ratings (63 W, 24 V, 3000 rpm) with a source.
34. The proposal's agitation validation (TC-S5) asks for a plausible, trend-consistent curve. The chapter now answers that with the shape error (Table `tab:trend`); to test the viscosity explanation of a horizontal shift you need a point range that includes the bend of the curve (lower N_Re, or a more viscous liquid), and the water temperature.

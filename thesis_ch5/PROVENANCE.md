# Chapter 5: where every statement comes from, and what is not yet backed up

Status key: **M** measured (file and script given) · **D** derived by calculation from M or from the firmware · **C** cited from a source in `references.bib` · **A** assumption, labelled as such in the text · **U** unsourced: you must supply the source or the measurement.

Data roots: `R` = `Desktop/School Stuff/Research SeBoCa Test`, `B` = `Desktop/School Stuff/mixr1_modern_backend`. Scripts are in `thesis_ch5/analysis/`.

## 1. Measured results (M)

| Statement in the chapter | File(s) | Script |
|---|---|---|
| CET vs DCT scatter, bias, age, delay; closed-loop scatter | `B/logs/ch5_new/*` (daemon logs, 256 PPR, 10 ms) | `estimator_figs.py`, `tables_v2.py` |
| Shaft-synchronous ripple, orders | same logs + index pulses | `orders_figs.py` |
| Plant K, tau, theta of the free motor | `B/cet_pwm_sweep.csv`, `B/dct_pwm_sweep.csv`, new open-loop sweeps in `logs/ch5_new` | `ch5_control_figs.py` (`fig_plant_id`) |
| Measured and simulated closed-loop staircases (Table cl-model) | `B/logs/ch5_new` (two staircases) | `ch5_control_figs.py` (`fig_pi_step`) |
| 18-setting gain matrix | `B/matrix_ki_174mm_test/` | `ch5_control_figs.py` (`fig_gain_matrix`) |
| 80-setting matrix (Kp 0.66-4.6, alpha 0.15-1, Ki 12), baffled and unbaffled | `B/matrix81_174mm_baffled_ki12_20261015_2221/results_raw.txt`, `B/matrix81_174mm_unbaffled_ki12_20261015_2002/results_raw.txt` | summary computed ad hoc (grouped means by Kp and alpha) |
| Pours at 460 rpm, open loop vs PI | `R/mixr1_log_20260908_162112_WATEROPENLOOP...csv`, `R/mixr1_log_20260909_163617_...WATERTESTACTUALDATA.csv` | `fig_pour.py` |
| Baffles and drops at 350 rpm | `R/mixr1_log_20260918_121626...csv`, `..._20260915_120201...csv`, `..._20260914_185949...csv` | `ch5_control_figs.py` (`fig_disturbance`) |
| Duty vs speed, power, efficiency | `R/mixr1_*_sequence_*`, free-motor sweep | `ch5_control_figs.py` (`fig_duty_rpm`) |
| Load-cell calibration (slope 0.9978, RMSE 0.68 mN m) | `R/Torque_Sensor Test/loadcell_test_20261012_210846.csv` | `ch5_control_figs.py` (`fig_loadcell`) |
| N_Re-N_Po curves, best runs, RMSE against CFD, Armfield, literature | `R/mixr1_96mm_*`, `R/mixr1_174mm_*` (sequence logs and summaries) | `npo_rmse.py`, `npo_figs_v2.py`, `npo_figs_v3.py`, `tables_v2.py` |
| Revolution count vs integral of speed (integrity check) | the 14 sequence logs of 5-7 Nov | ad hoc script, numbers in section 5.7 |
| u(N_Po) from torque and speed | the two best-run summaries | ad hoc script (torque RMSE 0.68 mN m; speed scatter/sqrt(n); CET bias 0.02 %) |

## 2. Derived (D)

EMA time constant, cut-off, variance reduction; the loop L(s), crossover, phase and gain margins, Ms, delay margin (model of the free-motor plant, `ch5_control_figs.py` and the Ms script); the rule Kp = wc/(KA), Ki = wc/K; the quantization noise q/sqrt(12); the N_Re and N_Po formulas; the facts about the firmware (pins, constants, protocol, menu, limits) are read from `include/config.hpp`, `src/main.cpp`, `src/network.cpp`, `src/ec11.cpp`, `src/motor.cpp`, and `mixr_dashboard.py`.

## 3. Cited (C)

SIMC/IMC rule: Skogestad 2003, Rivera et al. 1986. Phase/gain margin minima: Seborg et al. Ms between 1.2 and 2: Astrom and Hagglund 2006. Damping ratio and phase margin (zeta ~ PM/100): Ogata 2010, Nise 2020. Estimator error analysis and notation: Anuchin et al. 2019. Encoder datasheet values: Same Sky AMT102-V. pigpio sample interval: pigpio documentation. Raspberry Pi GPIO not 5 V tolerant: Raspberry Pi documentation. Level shifter circuit: NXP AN10441, SparkFun schematic and MakerLab page.
**Check before you submit:** the two entries added this session (Skogestad, Rivera) and the two converter entries were written from memory of the real documents; verify volume, pages, revision, and URL.

## 4. Assumptions (A), stated as such in the text

- Level-converter pull-up 10 kOhm (reference design) and load 20-100 pF (not measured; 1.1 us at 50 pF used as the example). *Fix: read the pull-up from the board's schematic and measure the rise time on a scope.*
- Dead time of the plant includes 5 ms for the sample-and-hold of the controller and a few ms of driver latency (not separately measured).
- Phase and gain margins are model values for the free-motor plant, not measurements.

## 5. Unsourced or inconsistent (U): you must supply or fix these

**Resolved since the first version of this file** (from your messages): vessel 297 mm, 2 blades, 4 baffles, room-temperature water (item 6, partly); torque sensor datasheet (item 4: ATO-TQS-S03 / ATO-DYJN-103, 0.3 % FS = 6 mN m, transmitter ATO-LCTR-OAR, now cited and tabulated); converter pull-up 10 kOhm and BSS138 (SparkFun schematic via the MakerLab page, now cited); gains of the 8/9 and 14/15 Sep runs = earlier gains, no slew limit then (item 8); cause of the PI fluctuation = the pouring, per the operator (item 9, not proven). Still open in those items: see `NEXT_STEPS.md`.

1. **CFD set-up**: solver, mesh, turbulence model, baffle and impeller geometry, convergence, who ran it. The chapter compares against it but cannot describe it.
2. **Armfield student data**: the source document and the impeller geometries (transcribed from a table).
3. **Brown et al. curve 10**: the BibTeX entry and the geometry of the reference configuration (D/T, C/T, impeller type, baffles).
4. **Torque sensor**: datasheet, rated accuracy, model number; the 2 N m rating.
5. **Motor and driver ratings** (DOGA motor, VNH5019 limits): datasheet numbers.
6. **Impeller geometry**: blade count and width, vessel diameter, baffles; what the designations 96 and 174 mm measure against D = 0.0665 and 0.1207 m.
7. **Water properties**: rho = 996.3 kg/m3 and mu = 0.854 mPa s are not sourced in the code, and correspond to about 27 C, while one run note says 24 C (rho 997.3, mu 0.911: Reynolds number 6 % lower). Measure the temperature; cite Tanaka 1975 and Kestin 1978 (or the Vogel equation).
8. **Gains and baffle state of the 8/9 September pour runs and of the 14/15 September runs**, and whether the slew limit existed then.
9. **Cause of the PI fluctuation 34-95 s** in the 9 September run, and of the 7 sequence logs whose revolution count is 9-39 % low.
10. **Power limit constants**: 90 W / 81 W confirmed; the 72 W in `mixr_dashboard.py` and in comments of `config.hpp` are stale.
11. **The Mode 1 test table** has no results, and the Mode 1 run logger has not been run on the Pi.
12. **Top speed 2500 rpm**, encoder direction (`ENCODER_DIRECTION`), supply rail of the encoder, and the other `% TODO` comments left in `chapter5.tex` (53 at last count).

## 6. Limits of this audit

Sections written in earlier sessions were produced from the logs and scripts above, but I did not re-verify every number against its file in this pass; I audited the text for external claims without a citation and for assumptions presented as facts, and fixed those I found (converter, damping/phase-margin rule, design limits, PI "textbook form", GPIO tolerance, water properties). Before submission, run the scripts again and diff the numbers in the chapter against their output.

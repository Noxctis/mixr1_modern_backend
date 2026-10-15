# Auto-generated figure captions

Numbers come straight from the data; edit the wording, not the numbers.

**01_pwm_traces** - Motor PWM command at steady state for a 350 RPM step. Same vertical scale in all panels; repeat 1 shown. PWM jitter (RMS sample-to-sample change over the last 40 % of the run, mean of repeats): A 0.177 %; G18 0.041 %; G27 0.034 %; G35 0.037 %; G44 0.032 %.

**individual/01_pwm_trace_A** - PWM command at steady state, A: baseline (Kp 2.63, Ki 1.454): jitter 0.177 % duty. Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_G18** - PWM command at steady state, G18: Kp 3.81, α 0.15: jitter 0.041 % duty(−77 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_G27** - PWM command at steady state, G27: Kp 3.15, α 0.15: jitter 0.034 % duty(−81 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_G35** - PWM command at steady state, G35: α 0.2: jitter 0.037 % duty(−79 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_G44** - PWM command at steady state, G44: Kp 2.23, α 0.2: jitter 0.032 % duty(−82 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_overlay** - All shown settings overlaid on one axis (repeat 1).

**individual/01_pwm_overlay_gains_and_filter** - Baseline vs lower gains plus filter: PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**02_step_response** - Speed response to a step from rest to 350 RPM. (a) Startup; (b) steady state. Overshoot (mean of repeats): A 4.2 %; G18 4.0 %; G27 4.2 %; G35 4.3 %; G44 4.3 %.

**individual/02a_step_startup** - Startup response to the 350 RPM step; overshoot A 4.2 %; G18 4.0 %; G27 4.2 %; G35 4.3 %; G44 4.3 %.

**individual/02b_step_steady** - Steady-state speed, same shaft-order ripple for all settings.

**individual/02a_step_startup_gains_and_filter** - Baseline vs lower gains plus filter: startup response to the 350 RPM step (solid = run 1, dashed = run 2).

**03_tuning_summary** - Effect of the PI settings on PWM jitter and startup overshoot (bars: mean of repeats, dots: individual repeats). Lowest jitter with overshoot ≤ 6 %: G45 (0.024 % duty, 5.2 % overshoot).

**individual/03_effect_kp** - PWM jitter versus Kp (Ki = 1.454, no filter); bars = mean of repeats, dots = repeats.

**individual/03_effect_alpha** - PWM jitter versus filter α (Kp = 2.63, Ki = 1.454); bars = mean of repeats, dots = repeats.

**individual/03_tradeoff** - Every setting tested: jitter versus overshoot. Best trade-off (overshoot ≤ 6 %): G45.

**individual/03_ranking** - Settings ranked by PWM jitter, bar colour = startup overshoot. Large grid: baseline plus the 15 best acceptable settings.

**04_ripple_orders** - Amplitude spectra (zero-to-peak, Hann window, 15 s steady state) versus shaft order. (a) Speed; (b) PWM command. PWM ripple peaks: A: 1× 0.041, 2× 0.033 % duty; G18: 1× 0.023, 2× 0.011 % duty; G27: 1× 0.029, 2× 0.007 % duty; G35: 1× 0.017, 2× 0.013 % duty; G44: 1× 0.025, 2× 0.015 % duty.

**individual/04a_spectrum_speed** - Speed-ripple spectrum versus shaft order.

**individual/04b_spectrum_pwm** - PWM-command ripple versus shaft order. Peaks: A: 1× 0.041, 2× 0.033 % duty; G18: 1× 0.023, 2× 0.011 % duty; G27: 1× 0.029, 2× 0.007 % duty; G35: 1× 0.017, 2× 0.013 % duty; G44: 1× 0.025, 2× 0.015 % duty.

**04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.041, 2× 0.033 % duty; G18: 1× 0.023, 2× 0.011 % duty; G27: 1× 0.029, 2× 0.007 % duty; G35: 1× 0.017, 2× 0.013 % duty; G44: 1× 0.025, 2× 0.015 % duty.

**individual/04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.041, 2× 0.033 % duty; G18: 1× 0.023, 2× 0.011 % duty; G27: 1× 0.029, 2× 0.007 % duty; G35: 1× 0.017, 2× 0.013 % duty; G44: 1× 0.025, 2× 0.015 % duty.

**06_rpm_traces** - Raw motor speed at steady state for a 350 RPM step. Same vertical scale in all panels; repeat 1 shown. Speed ripple (std over the last 40 % of the run, mean of repeats): A 3.16 RPM; G18 3.38 RPM; G27 3.10 RPM; G35 3.22 RPM; G44 4.36 RPM.

**individual/06_rpm_trace_A** - Raw speed at steady state, A: baseline (Kp 2.63, Ki 1.454): std 3.16 RPM. Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_G18** - Raw speed at steady state, G18: Kp 3.81, α 0.15: std 3.38 RPM(+7 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_G27** - Raw speed at steady state, G27: Kp 3.15, α 0.15: std 3.10 RPM(−2 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_G35** - Raw speed at steady state, G35: α 0.2: std 3.22 RPM(+2 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_G44** - Raw speed at steady state, G44: Kp 2.23, α 0.2: std 4.36 RPM(+38 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_overlay** - All shown settings overlaid on one axis (repeat 1).

**individual/06_rpm_overlay_gains_and_filter** - Baseline vs lower gains plus filter: raw speed at steady state, repeat 1, same vertical scale as the other speed graphs.

**07_rpm_summary** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.27 RPM (G01) to 5.14 RPM (G77); the baseline is 3.16 RPM.

**individual/07a_rpm_std** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.27 RPM (G01) to 5.14 RPM (G77); the baseline is 3.16 RPM.

**individual/07b_rpm_ripple_peaks** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.27 RPM (G01) to 5.14 RPM (G77); the baseline is 3.16 RPM.

**individual/07c_pwm_vs_rpm** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.27 RPM (G01) to 5.14 RPM (G77); the baseline is 3.16 RPM.

**08_step_by_setting** - Step response from rest to 350 RPM for each setting on the same axes. Overshoot / settling time to ±5 % (mean of repeats): A 4.2 % / 16.44 s; G18 4.0 % / 7.95 s; G27 4.2 % / 7.07 s; G35 4.3 % / 6.47 s; G44 4.3 % / 6.28 s.

**individual/08_step_A** - Step response of A: baseline (Kp 2.63, Ki 1.454): speed (top) and PWM command (bottom), both repeats. Overshoot 4.2 %, settling 16.44 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_A** - Steady-state speed (top) and PWM command (bottom) for A: baseline (Kp 2.63, Ki 1.454); same axes as the other 09_rpm_pwm graphs.

**individual/08_step_G18** - Step response of G18: Kp 3.81, α 0.15: speed (top) and PWM command (bottom), both repeats. Overshoot 4.0 %, settling 7.95 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_G18** - Steady-state speed (top) and PWM command (bottom) for G18: Kp 3.81, α 0.15; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_G27** - Step response of G27: Kp 3.15, α 0.15: speed (top) and PWM command (bottom), both repeats. Overshoot 4.2 %, settling 7.07 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_G27** - Steady-state speed (top) and PWM command (bottom) for G27: Kp 3.15, α 0.15; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_G35** - Step response of G35: α 0.2: speed (top) and PWM command (bottom), both repeats. Overshoot 4.3 %, settling 6.47 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_G35** - Steady-state speed (top) and PWM command (bottom) for G35: α 0.2; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_G44** - Step response of G44: Kp 2.23, α 0.2: speed (top) and PWM command (bottom), both repeats. Overshoot 4.3 %, settling 6.28 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_G44** - Steady-state speed (top) and PWM command (bottom) for G44: Kp 2.23, α 0.2; same axes as the other 09_rpm_pwm graphs.

**individual/05_settings_table** - Settings tested and results (mean of repeats).

**05_settings_table** - Settings tested and results (mean of repeats), Ki values present: [1.5]. Large grid: baseline plus the best 20 acceptable settings of 81; the full list is in metrics_pi_configs.csv.

**10_grid_heatmaps** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10a_grid_jitter** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10b_grid_overshoot** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10c_grid_rise** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10d_grid_settle** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10e_grid_settle2** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10f_grid_rpm_std** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10g_grid_power** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10h_grid_current_ma** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10e_grid_best_region** - Kp × α grid of 81 settings (Ki fixed at 1.454): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 6 % and settling ≤ 10 s (31 of 81). Best acceptable setting: G44_kp2.235_a0.2 (Kp 2.23, α 0.2), jitter 0.032 % duty, overshoot 4.3 %.

**individual/10f_top_candidates** - Shortlist of the best acceptable settings.

**10_top_candidates** - Shortlist: the 8 best acceptable settings ranked by PWM jitter (mean of repeats), with the baseline for reference.

**individual/11a_power_trace_A** - Electrical power (W) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11a_power_trace_G18** - Electrical power (W) during the run for setting G18 (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11a_power_trace_G27** - Electrical power (W) during the run for setting G27 (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11a_power_trace_G35** - Electrical power (W) during the run for setting G35 (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11a_power_trace_G44** - Electrical power (W) during the run for setting G44 (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11a_power_overlay** - Electrical power (W) of the shown settings on the same axes as the single-setting graphs.

**individual/11a_power_bars** - Mean electrical power (w) over the last 40 % of the run, per setting (error bar = spread of repeats).

**individual/11b_current_trace_A** - Supply current (mA) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11b_current_trace_G18** - Supply current (mA) during the run for setting G18 (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11b_current_trace_G27** - Supply current (mA) during the run for setting G27 (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11b_current_trace_G35** - Supply current (mA) during the run for setting G35 (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11b_current_trace_G44** - Supply current (mA) during the run for setting G44 (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11b_current_overlay** - Supply current (mA) of the shown settings on the same axes as the single-setting graphs.

**individual/11b_current_bars** - Mean supply current (ma) over the last 40 % of the run, per setting (error bar = spread of repeats).

**individual/11c_voltage_trace_A** - Supply voltage (V) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**individual/11c_voltage_trace_G18** - Supply voltage (V) during the run for setting G18 (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**individual/11c_voltage_trace_G27** - Supply voltage (V) during the run for setting G27 (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**individual/11c_voltage_trace_G35** - Supply voltage (V) during the run for setting G35 (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**individual/11c_voltage_trace_G44** - Supply voltage (V) during the run for setting G44 (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**individual/11c_voltage_overlay** - Supply voltage (V) of the shown settings on the same axes as the single-setting graphs.

**00_summary_table** - Summary of all measured data (mean of repeats). See also metrics_*.csv for full per-run data.

**individual/00_summary_table** - Summary of all measured data (mean of repeats). See also metrics_*.csv for full per-run data.


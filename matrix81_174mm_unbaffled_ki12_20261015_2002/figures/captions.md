# Auto-generated figure captions

Numbers come straight from the data; edit the wording, not the numbers.

**01_pwm_traces** - Motor PWM command at steady state for a 300 RPM step. Same vertical scale in all panels; repeat 1 shown. PWM jitter (RMS sample-to-sample change over the last 40 % of the run, mean of repeats): A 0.194 %.

**individual/01_pwm_trace_A** - PWM command at steady state, A: baseline (Kp 2.63, Ki 12): jitter 0.194 % duty. Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_overlay** - All shown settings overlaid on one axis (repeat 1).

**02_step_response** - Speed response to a step from rest to 300 RPM. (a) Startup; (b) steady state. Overshoot (mean of repeats): A 17.3 %.

**individual/02a_step_startup** - Startup response to the 300 RPM step; overshoot A 17.3 %.

**individual/02b_step_steady** - Steady-state speed, same shaft-order ripple for all settings.

**03_tuning_summary** - Effect of the PI settings on PWM jitter and startup overshoot (bars: mean of repeats, dots: individual repeats). Lowest jitter with overshoot ≤ 5 %: A (0.194 % duty, 17.3 % overshoot).

**individual/03_effect_kp** - PWM jitter versus Kp (Ki = 12, no filter); bars = mean of repeats, dots = repeats.

**individual/03_effect_alpha** - PWM jitter versus filter α (Kp = 2.63, Ki = 12); bars = mean of repeats, dots = repeats.

**individual/03_tradeoff** - Every setting tested: jitter versus overshoot. Best trade-off (overshoot ≤ 5 %): A.

**individual/03_ranking** - Settings ranked by PWM jitter, bar colour = startup overshoot. Large grid: baseline plus the 15 best acceptable settings.

**04_ripple_orders** - Amplitude spectra (zero-to-peak, Hann window, 15 s steady state) versus shaft order. (a) Speed; (b) PWM command. PWM ripple peaks: A: 1× 0.034, 2× 0.025 % duty.

**individual/04a_spectrum_speed** - Speed-ripple spectrum versus shaft order.

**individual/04b_spectrum_pwm** - PWM-command ripple versus shaft order. Peaks: A: 1× 0.034, 2× 0.025 % duty.

**04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.034, 2× 0.025 % duty.

**individual/04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.034, 2× 0.025 % duty.

**06_rpm_traces** - Raw motor speed at steady state for a 300 RPM step. Same vertical scale in all panels; repeat 1 shown. Speed ripple (std over the last 40 % of the run, mean of repeats): A 12.58 RPM.

**individual/06_rpm_trace_A** - Raw speed at steady state, A: baseline (Kp 2.63, Ki 12): std 12.58 RPM. Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_overlay** - All shown settings overlaid on one axis (repeat 1).

**07_rpm_summary** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 11.97 RPM (G11) to 13.23 RPM (G47); the baseline is 12.58 RPM.

**individual/07a_rpm_std** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 11.97 RPM (G11) to 13.23 RPM (G47); the baseline is 12.58 RPM.

**individual/07b_rpm_ripple_peaks** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 11.97 RPM (G11) to 13.23 RPM (G47); the baseline is 12.58 RPM.

**individual/07c_pwm_vs_rpm** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 11.97 RPM (G11) to 13.23 RPM (G47); the baseline is 12.58 RPM.

**08_step_by_setting** - Step response from rest to 300 RPM for each setting on the same axes. Overshoot / settling time to ±5 % (mean of repeats): A 17.3 % / 14.58 s.

**individual/08_step_A** - Step response of A: baseline (Kp 2.63, Ki 12): speed (top) and PWM command (bottom), both repeats. Overshoot 17.3 %, settling 14.58 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_A** - Steady-state speed (top) and PWM command (bottom) for A: baseline (Kp 2.63, Ki 12); same axes as the other 09_rpm_pwm graphs.

**individual/05_settings_table** - Settings tested and results (mean of repeats).

**05_settings_table** - Settings tested and results (mean of repeats), Ki values present: [12.0]. Large grid: baseline plus the best 0 acceptable settings of 81; the full list is in metrics_pi_configs.csv.

**10_grid_heatmaps** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10a_grid_jitter** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10b_grid_overshoot** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10c_grid_rise** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10d_grid_settle** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10e_grid_settle2** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10f_grid_rpm_std** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10g_grid_power** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10h_grid_current_ma** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10e_grid_best_region** - Kp × α grid of 81 settings (Ki fixed at 12): PWM jitter, startup overshoot, settling time and speed ripple (mean of repeats). Outlined cells meet overshoot ≤ 5 % and settling ≤ 0.3 s (0 of 81). No setting met the limits.

**individual/10f_top_candidates** - Shortlist of the best acceptable settings.

**10_top_candidates** - Shortlist: the 0 best acceptable settings ranked by PWM jitter (mean of repeats), with the baseline for reference.

**individual/11a_power_trace_A** - Electrical power (W) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every power graph.

**individual/11b_current_trace_A** - Supply current (mA) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every current graph.

**individual/11c_voltage_trace_A** - Supply voltage (V) during the run for setting A (grey = last 40 % of the run, dashed = its mean). Axes identical in every voltage graph.

**00_summary_table** - Summary of all measured data (mean of repeats). See also metrics_*.csv for full per-run data.

**individual/00_summary_table** - Summary of all measured data (mean of repeats). See also metrics_*.csv for full per-run data.


# Auto-generated figure captions

Numbers come straight from the data; edit the wording, not the numbers.

**01_pwm_traces** - Motor PWM command at steady state for a 460 RPM step. Same vertical scale in all panels; repeat 1 shown. PWM jitter (RMS sample-to-sample change over the last 40 % of the run, mean of repeats): A 0.126 %; C1 0.096 %; C2 0.084 %; C3 0.073 %; B1 0.075 %; B2 0.047 %; B3 0.023 %; D1 0.059 %; D2 0.038 %; D3 0.050 %; D4 0.031 %.

**individual/01_pwm_trace_A** - PWM command at steady state, A: baseline (Kp 1.56, Ki 41.2): jitter 0.126 % duty. Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_C1** - PWM command at steady state, C1: Kp 1.2: jitter 0.096 % duty(−24 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_C2** - PWM command at steady state, C2: Kp 1.0: jitter 0.084 % duty(−33 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_C3** - PWM command at steady state, C3: Kp 0.8: jitter 0.073 % duty(−42 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_B1** - PWM command at steady state, B1: α 0.5: jitter 0.075 % duty(−41 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_B2** - PWM command at steady state, B2: α 0.3: jitter 0.047 % duty(−63 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_B3** - PWM command at steady state, B3: α 0.15: jitter 0.023 % duty(−82 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_D1** - PWM command at steady state, D1: Kp 1.2, α 0.5: jitter 0.059 % duty(−53 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_D2** - PWM command at steady state, D2: Kp 1.2, α 0.3: jitter 0.038 % duty(−70 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_D3** - PWM command at steady state, D3: Kp 1.0, α 0.5: jitter 0.050 % duty(−61 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_D4** - PWM command at steady state, D4: Kp 1.0, α 0.3: jitter 0.031 % duty(−75 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_overlay** - All shown settings overlaid on one axis (repeat 1).

**individual/01_pwm_overlay_no_filter** - Baseline vs lower Kp / Ki (no filter): PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**individual/01_pwm_overlay_filter_only** - Baseline vs feedback filter alone (baseline Kp, Ki): PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**individual/01_pwm_overlay_gains_and_filter** - Baseline vs lower gains plus filter: PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**02_step_response** - Speed response to a step from rest to 460 RPM. (a) Startup; (b) steady state. Overshoot (mean of repeats): A 1.3 %; C1 1.2 %; C2 1.2 %; C3 1.6 %; B1 15.3 %; B2 30.2 %; B3 65.4 %; D1 16.3 %; D2 30.1 %; D3 12.5 %; D4 34.3 %.

**individual/02a_step_startup** - Startup response to the 460 RPM step; overshoot A 1.3 %; C1 1.2 %; C2 1.2 %; C3 1.6 %; B1 15.3 %; B2 30.2 %; B3 65.4 %; D1 16.3 %; D2 30.1 %; D3 12.5 %; D4 34.3 %.

**individual/02b_step_steady** - Steady-state speed, same shaft-order ripple for all settings.

**individual/02a_step_startup_no_filter** - Baseline vs lower Kp / Ki (no filter): startup response to the 460 RPM step (solid = run 1, dashed = run 2).

**individual/02a_step_startup_filter_only** - Baseline vs feedback filter alone (baseline Kp, Ki): startup response to the 460 RPM step (solid = run 1, dashed = run 2).

**individual/02a_step_startup_gains_and_filter** - Baseline vs lower gains plus filter: startup response to the 460 RPM step (solid = run 1, dashed = run 2).

**03_tuning_summary** - Effect of the PI settings on PWM jitter and startup overshoot (bars: mean of repeats, dots: individual repeats). Lowest jitter with overshoot ≤ 15 %: D3 (0.050 % duty, 12.5 % overshoot).

**individual/03_effect_kp** - PWM jitter versus Kp (Ki = 41.2, no filter); bars = mean of repeats, dots = repeats.

**individual/03_effect_alpha** - PWM jitter versus filter α (Kp = 1.56, Ki = 41.2); bars = mean of repeats, dots = repeats.

**individual/03_tradeoff** - Every setting tested: jitter versus overshoot. Best trade-off (overshoot ≤ 15 %): D3.

**individual/03_ranking** - All settings ranked by PWM jitter, bar colour = startup overshoot.

**04_ripple_orders** - Amplitude spectra (zero-to-peak, Hann window, 15 s steady state) versus shaft order. (a) Speed; (b) PWM command. PWM ripple peaks: A: 1× 0.142, 2× 0.127 % duty; C1: 1× 0.110, 2× 0.092 % duty; C2: 1× 0.106, 2× 0.077 % duty; C3: 1× 0.104, 2× 0.065 % duty; B1: 1× 0.151, 2× 0.067 % duty; B2: 1× 0.110, 2× 0.034 % duty; B3: 1× 0.048, 2× 0.015 % duty; D1: 1× 0.122, 2× 0.049 % duty; D2: 1× 0.089, 2× 0.027 % duty; D3: 1× 0.107, 2× 0.041 % duty; D4: 1× 0.069, 2× 0.023 % duty.

**individual/04a_spectrum_speed** - Speed-ripple spectrum versus shaft order.

**individual/04b_spectrum_pwm** - PWM-command ripple versus shaft order. Peaks: A: 1× 0.142, 2× 0.127 % duty; C1: 1× 0.110, 2× 0.092 % duty; C2: 1× 0.106, 2× 0.077 % duty; C3: 1× 0.104, 2× 0.065 % duty; B1: 1× 0.151, 2× 0.067 % duty; B2: 1× 0.110, 2× 0.034 % duty; B3: 1× 0.048, 2× 0.015 % duty; D1: 1× 0.122, 2× 0.049 % duty; D2: 1× 0.089, 2× 0.027 % duty; D3: 1× 0.107, 2× 0.041 % duty; D4: 1× 0.069, 2× 0.023 % duty.

**04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.142, 2× 0.127 % duty; C1: 1× 0.110, 2× 0.092 % duty; C2: 1× 0.106, 2× 0.077 % duty; C3: 1× 0.104, 2× 0.065 % duty; B1: 1× 0.151, 2× 0.067 % duty; B2: 1× 0.110, 2× 0.034 % duty; B3: 1× 0.048, 2× 0.015 % duty; D1: 1× 0.122, 2× 0.049 % duty; D2: 1× 0.089, 2× 0.027 % duty; D3: 1× 0.107, 2× 0.041 % duty; D4: 1× 0.069, 2× 0.023 % duty.

**individual/04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.142, 2× 0.127 % duty; C1: 1× 0.110, 2× 0.092 % duty; C2: 1× 0.106, 2× 0.077 % duty; C3: 1× 0.104, 2× 0.065 % duty; B1: 1× 0.151, 2× 0.067 % duty; B2: 1× 0.110, 2× 0.034 % duty; B3: 1× 0.048, 2× 0.015 % duty; D1: 1× 0.122, 2× 0.049 % duty; D2: 1× 0.089, 2× 0.027 % duty; D3: 1× 0.107, 2× 0.041 % duty; D4: 1× 0.069, 2× 0.023 % duty.

**06_rpm_traces** - Raw motor speed at steady state for a 460 RPM step. Same vertical scale in all panels; repeat 1 shown. Speed ripple (std over the last 40 % of the run, mean of repeats): A 3.11 RPM; C1 2.86 RPM; C2 2.94 RPM; C3 3.05 RPM; B1 3.43 RPM; B2 3.27 RPM; B3 2.79 RPM; D1 3.30 RPM; D2 3.12 RPM; D3 3.13 RPM; D4 2.90 RPM.

**individual/06_rpm_trace_A** - Raw speed at steady state, A: baseline (Kp 1.56, Ki 41.2): std 3.11 RPM. Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_C1** - Raw speed at steady state, C1: Kp 1.2: std 2.86 RPM(−8 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_C2** - Raw speed at steady state, C2: Kp 1.0: std 2.94 RPM(−6 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_C3** - Raw speed at steady state, C3: Kp 0.8: std 3.05 RPM(−2 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_B1** - Raw speed at steady state, B1: α 0.5: std 3.43 RPM(+10 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_B2** - Raw speed at steady state, B2: α 0.3: std 3.27 RPM(+5 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_B3** - Raw speed at steady state, B3: α 0.15: std 2.79 RPM(−10 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_D1** - Raw speed at steady state, D1: Kp 1.2, α 0.5: std 3.30 RPM(+6 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_D2** - Raw speed at steady state, D2: Kp 1.2, α 0.3: std 3.12 RPM(≈ 0 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_D3** - Raw speed at steady state, D3: Kp 1.0, α 0.5: std 3.13 RPM(+1 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_trace_D4** - Raw speed at steady state, D4: Kp 1.0, α 0.3: std 2.90 RPM(−7 %). Same vertical scale as the other 06_rpm_trace graphs.

**individual/06_rpm_overlay** - All shown settings overlaid on one axis (repeat 1).

**individual/06_rpm_overlay_no_filter** - Baseline vs lower Kp / Ki (no filter): raw speed at steady state, repeat 1, same vertical scale as the other speed graphs.

**individual/06_rpm_overlay_filter_only** - Baseline vs feedback filter alone (baseline Kp, Ki): raw speed at steady state, repeat 1, same vertical scale as the other speed graphs.

**individual/06_rpm_overlay_gains_and_filter** - Baseline vs lower gains plus filter: raw speed at steady state, repeat 1, same vertical scale as the other speed graphs.

**07_rpm_summary** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.79 RPM (B3) to 3.43 RPM (B1); the baseline is 3.11 RPM.

**individual/07a_rpm_std** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.79 RPM (B3) to 3.43 RPM (B1); the baseline is 3.11 RPM.

**individual/07b_rpm_ripple_peaks** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.79 RPM (B3) to 3.43 RPM (B1); the baseline is 3.11 RPM.

**individual/07c_pwm_vs_rpm** - Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. Speed ripple ranges from 2.79 RPM (B3) to 3.43 RPM (B1); the baseline is 3.11 RPM.

**08_step_by_setting** - Step response from rest to 460 RPM for each setting on the same axes. Overshoot / settling time to ±5 % (mean of repeats): A 1.3 % / 0.14 s; C1 1.2 % / 0.11 s; C2 1.2 % / 0.08 s; C3 1.6 % / 0.08 s; B1 15.3 % / 0.18 s; B2 30.2 % / 0.27 s; B3 65.4 % / 0.62 s; D1 16.3 % / 0.14 s; D2 30.1 % / 0.28 s; D3 12.5 % / 0.13 s; D4 34.3 % / 0.32 s.

**individual/08_step_A** - Step response of A: baseline (Kp 1.56, Ki 41.2): speed (top) and PWM command (bottom), both repeats. Overshoot 1.3 %, settling 0.14 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_A** - Steady-state speed (top) and PWM command (bottom) for A: baseline (Kp 1.56, Ki 41.2); same axes as the other 09_rpm_pwm graphs.

**individual/08_step_C1** - Step response of C1: Kp 1.2: speed (top) and PWM command (bottom), both repeats. Overshoot 1.2 %, settling 0.11 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_C1** - Steady-state speed (top) and PWM command (bottom) for C1: Kp 1.2; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_C2** - Step response of C2: Kp 1.0: speed (top) and PWM command (bottom), both repeats. Overshoot 1.2 %, settling 0.08 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_C2** - Steady-state speed (top) and PWM command (bottom) for C2: Kp 1.0; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_C3** - Step response of C3: Kp 0.8: speed (top) and PWM command (bottom), both repeats. Overshoot 1.6 %, settling 0.08 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_C3** - Steady-state speed (top) and PWM command (bottom) for C3: Kp 0.8; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_B1** - Step response of B1: α 0.5: speed (top) and PWM command (bottom), both repeats. Overshoot 15.3 %, settling 0.18 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_B1** - Steady-state speed (top) and PWM command (bottom) for B1: α 0.5; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_B2** - Step response of B2: α 0.3: speed (top) and PWM command (bottom), both repeats. Overshoot 30.2 %, settling 0.27 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_B2** - Steady-state speed (top) and PWM command (bottom) for B2: α 0.3; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_B3** - Step response of B3: α 0.15: speed (top) and PWM command (bottom), both repeats. Overshoot 65.4 %, settling 0.62 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_B3** - Steady-state speed (top) and PWM command (bottom) for B3: α 0.15; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_D1** - Step response of D1: Kp 1.2, α 0.5: speed (top) and PWM command (bottom), both repeats. Overshoot 16.3 %, settling 0.14 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_D1** - Steady-state speed (top) and PWM command (bottom) for D1: Kp 1.2, α 0.5; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_D2** - Step response of D2: Kp 1.2, α 0.3: speed (top) and PWM command (bottom), both repeats. Overshoot 30.1 %, settling 0.28 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_D2** - Steady-state speed (top) and PWM command (bottom) for D2: Kp 1.2, α 0.3; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_D3** - Step response of D3: Kp 1.0, α 0.5: speed (top) and PWM command (bottom), both repeats. Overshoot 12.5 %, settling 0.13 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_D3** - Steady-state speed (top) and PWM command (bottom) for D3: Kp 1.0, α 0.5; same axes as the other 09_rpm_pwm graphs.

**individual/08_step_D4** - Step response of D4: Kp 1.0, α 0.3: speed (top) and PWM command (bottom), both repeats. Overshoot 34.3 %, settling 0.32 s. Same axes as the other 08_step graphs.

**individual/09_rpm_pwm_D4** - Steady-state speed (top) and PWM command (bottom) for D4: Kp 1.0, α 0.3; same axes as the other 09_rpm_pwm graphs.

**individual/05_settings_table** - Settings tested and results (mean of repeats).

**05_settings_table** - Settings tested and results (mean of repeats), Ki values present: [41.2].


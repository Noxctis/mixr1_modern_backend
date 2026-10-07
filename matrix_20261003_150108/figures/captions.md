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

**individual/05_settings_table** - Settings tested and results (mean of repeats).

**05_settings_table** - Settings tested and results (mean of repeats), Ki values present: [41.2].


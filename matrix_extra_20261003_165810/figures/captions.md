# Auto-generated figure captions

Numbers come straight from the data; edit the wording, not the numbers.

**01_pwm_traces** - Motor PWM command at steady state for a 460 RPM step. Same vertical scale in all panels; repeat 1 shown. PWM jitter (RMS sample-to-sample change over the last 40 % of the run, mean of repeats): A 0.109 %; C3 0.060 %; F2 0.054 %; F4 0.051 %; D3 0.034 %; F1 0.030 %; F3 0.027 %.

**individual/01_pwm_trace_A** - PWM command at steady state, A: baseline (Kp 1.56, Ki 41.2): jitter 0.109 % duty. Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_C3** - PWM command at steady state, C3: Kp 0.8: jitter 0.060 % duty(−45 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_F2** - PWM command at steady state, F2: Kp 0.8, Ki 25.0: jitter 0.054 % duty(−51 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_F4** - PWM command at steady state, F4: Kp 0.8, Ki 15.0: jitter 0.051 % duty(−54 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_D3** - PWM command at steady state, D3: Kp 1.0, α 0.5: jitter 0.034 % duty(−69 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_F1** - PWM command at steady state, F1: Kp 0.8, α 0.5: jitter 0.030 % duty(−73 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_trace_F3** - PWM command at steady state, F3: Kp 0.8, Ki 25.0, α 0.5: jitter 0.027 % duty(−76 %). Same vertical scale as the other 01_pwm_trace graphs.

**individual/01_pwm_overlay** - All shown settings overlaid on one axis (repeat 1).

**individual/01_pwm_overlay_no_filter** - Baseline vs lower Kp / Ki (no filter): PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**individual/01_pwm_overlay_gains_and_filter** - Baseline vs lower gains plus filter: PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.

**02_step_response** - Speed response to a step from rest to 460 RPM. (a) Startup; (b) steady state. Overshoot (mean of repeats): A 1.6 %; C3 1.8 %; F2 1.3 %; F4 1.2 %; D3 10.6 %; F1 13.4 %; F3 1.2 %.

**individual/02a_step_startup** - Startup response to the 460 RPM step; overshoot A 1.6 %; C3 1.8 %; F2 1.3 %; F4 1.2 %; D3 10.6 %; F1 13.4 %; F3 1.2 %.

**individual/02b_step_steady** - Steady-state speed, same shaft-order ripple for all settings.

**individual/02a_step_startup_no_filter** - Baseline vs lower Kp / Ki (no filter): startup response to the 460 RPM step (solid = run 1, dashed = run 2).

**individual/02a_step_startup_gains_and_filter** - Baseline vs lower gains plus filter: startup response to the 460 RPM step (solid = run 1, dashed = run 2).

**03_tuning_summary** - Effect of the PI settings on PWM jitter and startup overshoot (bars: mean of repeats, dots: individual repeats). Lowest jitter with overshoot ≤ 15 %: F3 (0.027 % duty, 1.2 % overshoot).

**individual/03_effect_kp** - PWM jitter versus Kp (Ki = 41.2, no filter); bars = mean of repeats, dots = repeats.

**individual/03_effect_ki** - PWM jitter versus Ki (Kp = 0.8, no filter); bars = mean of repeats, dots = repeats.

**individual/03_effect_filter** - PWM jitter versus the feedback filter (same Kp and Ki, with vs without filter); bars = mean of repeats, dots = repeats.

**individual/03_tradeoff** - Every setting tested: jitter versus overshoot. Best trade-off (overshoot ≤ 15 %): F3.

**individual/03_ranking** - All settings ranked by PWM jitter, bar colour = startup overshoot.

**04_ripple_orders** - Amplitude spectra (zero-to-peak, Hann window, 15 s steady state) versus shaft order. (a) Speed; (b) PWM command. PWM ripple peaks: A: 1× 0.046, 2× 0.110 % duty; C3: 1× 0.034, 2× 0.055 % duty; F2: 1× 0.021, 2× 0.047 % duty; F4: 1× 0.018, 2× 0.045 % duty; D3: 1× 0.030, 2× 0.037 % duty; F1: 1× 0.030, 2× 0.030 % duty; F3: 1× 0.020, 2× 0.027 % duty.

**individual/04a_spectrum_speed** - Speed-ripple spectrum versus shaft order.

**individual/04b_spectrum_pwm** - PWM-command ripple versus shaft order. Peaks: A: 1× 0.046, 2× 0.110 % duty; C3: 1× 0.034, 2× 0.055 % duty; F2: 1× 0.021, 2× 0.047 % duty; F4: 1× 0.018, 2× 0.045 % duty; D3: 1× 0.030, 2× 0.037 % duty; F1: 1× 0.030, 2× 0.030 % duty; F3: 1× 0.020, 2× 0.027 % duty.

**04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.046, 2× 0.110 % duty; C3: 1× 0.034, 2× 0.055 % duty; F2: 1× 0.021, 2× 0.047 % duty; F4: 1× 0.018, 2× 0.045 % duty; D3: 1× 0.030, 2× 0.037 % duty; F1: 1× 0.030, 2× 0.030 % duty; F3: 1× 0.020, 2× 0.027 % duty.

**individual/04c_ripple_peaks** - PWM-command ripple amplitude at 1× and 2× the shaft frequency for each setting (mean of repeats). A: 1× 0.046, 2× 0.110 % duty; C3: 1× 0.034, 2× 0.055 % duty; F2: 1× 0.021, 2× 0.047 % duty; F4: 1× 0.018, 2× 0.045 % duty; D3: 1× 0.030, 2× 0.037 % duty; F1: 1× 0.030, 2× 0.030 % duty; F3: 1× 0.020, 2× 0.027 % duty.

**individual/05_settings_table** - Settings tested and results (mean of repeats).

**05_settings_table** - Settings tested and results (mean of repeats), Ki values present: [15.0, 25.0, 41.2].


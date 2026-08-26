# Thesis Update: Pairwise Comparison Summary (26/08/2026)

## Scope
Three pairwise comparisons were generated from the latest datasets:
1. PI FIFO vs PI no FIFO
2. PI FIFO vs water raw FIFO
3. Water raw FIFO vs raw no FIFO

## 1) PI FIFO vs PI no FIFO
Source folder: thesis_pair_pi_fifo_vs_pi_nofifo

### Key quantitative findings
- Mean RMS loop jitter:
  - PI FIFO: 4.71 us
  - PI no FIFO: 7.77 us
  - Reduction with FIFO: about 39.4%
- P95 absolute jitter:
  - PI FIFO: 10.90 us
  - PI no FIFO: 18.32 us
  - Reduction with FIFO: about 40.5%
- P99 absolute jitter:
  - PI FIFO: 15.48 us
  - PI no FIFO: 33.26 us
  - Reduction with FIFO: about 53.5%
- Max late_us trend by step:
  - PI FIFO up to about 46 us
  - PI no FIFO up to about 156 us
- Mean PI steady-state error across valid targets:
  - PI FIFO: 5.87 RPM
  - PI no FIFO: 6.08 RPM

### Interpretation
- Scheduling with FIFO substantially tightens timing dispersion and tail jitter.
- Closed-loop speed tracking (steady-state RPM error) is similar in both runs, but FIFO makes timing behavior much more deterministic.
- For thesis argumentation: FIFO primarily improves real-time timing quality and high-percentile latency risk, while control accuracy remains near-equivalent.

### Recommended figures for slides
- jitter_comparison.png
- jitter_vs_pwm.png
- rpm_vs_pwm.png
- pi_performance.png

## 2) PI FIFO vs Water Raw FIFO
Source folder: thesis_pair_pi_fifo_vs_water_raw_fifo

### Key quantitative findings
- Mean RMS jitter:
  - PI FIFO: 4.71 us
  - Open-loop FIFO (water): 1.94 us
- PI improvement over open-loop baseline (interpolated) by target:
  - 100 RPM: 79.8%
  - 200 RPM: 88.1%
  - 300 RPM: 91.7%
  - 400 RPM: 94.6%
  - 500 RPM: 95.9%
  - 600 RPM: 96.0%
  - 700 RPM: 96.3%
  - 800 RPM: 95.2%
  - 900 RPM: 94.9%
  - 1000 RPM: 94.7%

### Interpretation
- Under water load, the open-loop plant curve differs strongly from PI setpoint behavior.
- PI control reduces steady-state error by roughly 80% to 96% across operating points.
- Open-loop FIFO can have lower jitter than PI FIFO because PI includes feedback computation and actuation updates; however, PI is the only mode that achieves target tracking.
- Thesis framing: control benefit dominates for speed accuracy, while small extra timing overhead remains bounded and deterministic.

### Recommended figures for slides
- pi_performance.png
- rpm_vs_pwm.png
- jitter_comparison.png
- jitter_vs_pwm.png

## 3) Water Raw FIFO vs Raw no FIFO
Source folder: thesis_pair_water_raw_fifo_vs_raw_nofifo

### Key quantitative findings
- Mean RMS jitter:
  - Open-loop FIFO: 1.94 us
  - Open-loop no FIFO: 262.07 us
  - FIFO reduction: about 99.3%
- P95 absolute jitter:
  - Open-loop FIFO: 3.39 us
  - Open-loop no FIFO: 41.29 us
  - FIFO reduction: about 91.8%
- P99 absolute jitter:
  - Open-loop FIFO: 8.05 us
  - Open-loop no FIFO: 754.15 us
  - FIFO reduction: about 98.9%
- Max late_us by step:
  - Open-loop FIFO: typically below about 76 us
  - Open-loop no FIFO: up to about 9107 us
- Timing outliers >500 us:
  - Open-loop FIFO: 0 outliers at all steps
  - Open-loop no FIFO: frequent outliers (for example up to 14 at 90% step)

### Interpretation
- This is the clearest scheduling result in the full dataset.
- Without FIFO, the tail-latency behavior (worst-case and P99) degrades by orders of magnitude.
- With FIFO, loop timing stays near-ideal around 10 ms and preserves stable RPM variance structure.
- Thesis framing: FIFO is essential for predictable real-time behavior under load and should be treated as a requirement, not a tuning option.

### Recommended figures for slides
- jitter_comparison.png
- jitter_vs_pwm.png
- rpm_hist_070.png
- rpm_hist_090.png
- rpm_hist_100.png

## Graph locations
- thesis_pair_pi_fifo_vs_pi_nofifo/
- thesis_pair_pi_fifo_vs_water_raw_fifo/
- thesis_pair_water_raw_fifo_vs_raw_nofifo/

## Suggested 1-slide message
- FIFO improves real-time timing determinism dramatically (especially tail jitter and worst-case lateness).
- PI control improves setpoint tracking massively versus open-loop baseline in loaded operation.
- Combined result supports the final architecture choice: FIFO scheduling + PI control.

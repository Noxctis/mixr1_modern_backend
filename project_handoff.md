# MIXR-1 Project Handoff

## Purpose

This document summarizes the current work on the MIXR-1 motor-control testing and analysis so another group member can continue with the same context.

## Professor's Main Feedback

The presentation and thesis must show the complete engineering process, not only code or final graphs.

The important items are:

1. Show raw data before processed or filtered data.
2. Characterize the motor using several input levels.
3. Use histograms to show RPM and timing distributions.
4. Report mean, standard deviation, and jitter.
5. Explain the first-order model used for curve fitting.
6. Define what the model parameters mean.
7. Explain how PI gains were selected.
8. Compare open-loop and closed-loop performance.
9. Show performance before and after applying the controller.
10. Test both no-load and water-loaded conditions.
11. Report rise time, settling time, overshoot, and steady-state error.
12. Explain whether the sampling rate is appropriate based on the measured system response.
13. Present the process with a block diagram, flowchart, or algorithm rather than showing source code directly.

The professor also suggested testing scheduling effects using higher priority, `SCHED_FIFO`, possible CPU isolation, and a short high-priority timing-critical function if necessary.

## Current C++ Architecture

The main program is [src/main.cpp](src/main.cpp).

The production daemon currently:

- Attempts to set the main thread to `SCHED_FIFO` priority 90.
- Runs the control loop every 10 ms.
- Uses `sleep_until()` with an absolute `next_wake` deadline.
- Reads the encoder count.
- Calculates RPM through `KinematicsEngine`.
- Runs the PI controller when a target RPM is present.
- Converts the 12-bit PWM command from `0..4095` to hardware PWM duty `0..1,000,000`.
- Sends telemetry to the dashboard.
- Updates the LCD periodically.

The encoder A/B signals are processed asynchronously by pigpio callbacks. The encoder count and revolution count are atomic values read by the main loop.

Important source files:

- [src/main.cpp](src/main.cpp): main daemon and test mode
- [src/kinematics.cpp](src/kinematics.cpp): encoder-to-RPM calculation and filtering
- [src/pi_controller.cpp](src/pi_controller.cpp): PI control and gain scheduling
- [include/config.hpp](include/config.hpp): CPR, timing, pins, and PI gain schedule
- [src/motor.cpp](src/motor.cpp): hardware PWM output
- [src/encoder.cpp](src/encoder.cpp): quadrature encoder callbacks

## Test Mode Implemented

The executable supports a hardware test mode:

```bash
./mixr1_daemon --test
```

Options:

```text
--sweep              Run the 0% to 100% sweep
--fixed              Run one fixed PWM or target test
--fifo               Request SCHED_FIFO priority 90
--no-fifo            Use normal Linux scheduling
--pi                 Enable PI control
--no-pi              Disable PI control
--target=RPM         Maximum target for a PI sweep
--pwm=0..4095        Fixed PWM for a fixed open-loop test
--duration=SECONDS   Duration of each sweep step
--csv=FILE           CSV output path
```

The default sweep has 11 steps:

```text
0%, 10%, 20%, ..., 100%
```

Each step lasts for the value supplied to `--duration`. For example, `--duration=10` means 10 seconds per step and approximately 110 seconds total.

The test CSV includes:

```text
elapsed_s
step_index
pwm_percent
loop_period_us
late_us
raw_rpm
filtered_rpm
target_rpm
pwm
error_rpm
```

## Correct Experiments

### 1. Open-Loop Motor Characterization

This test applies fixed PWM values and records motor RPM. No PI feedback is used.

The expected approximate relationship is:

| PWM | Expected target/reference RPM |
|---:|---:|
| 0% | 0 RPM |
| 10% | 250 RPM |
| 20% | 500 RPM |
| 40% | 1000 RPM |
| 60% | 1500 RPM |
| 80% | 2000 RPM |
| 100% | 2500 RPM |

Run without FIFO:

```bash
sudo ./mixr1_daemon --test --sweep --no-fifo --no-pi \
  --duration=10 --target=0 --csv=raw_no_fifo.csv
```

Run with FIFO:

```bash
sudo ./mixr1_daemon --test --sweep --fifo --no-pi \
  --duration=10 --target=0 --csv=raw_fifo.csv
```

`--target=0` is intentional here because this is open-loop. The target and controller metrics are not applicable to this experiment.

### 2. PI Closed-Loop Setpoint Test

The PI sweep uses target RPM steps from 0 to the chosen maximum target. With `--target=2500`, the steps are:

```text
0, 250, 500, ..., 2500 RPM
```

Run without FIFO:

```bash
sudo ./mixr1_daemon --test --sweep --no-fifo --pi \
  --target=2500 --duration=10 --csv=pi_no_fifo.csv
```

Run with FIFO:

```bash
sudo ./mixr1_daemon --test --sweep --fifo --pi \
  --target=2500 --duration=10 --csv=pi_fifo.csv
```

At every target change, the PI integral is reset. This avoids carrying accumulated error from one setpoint into the next.

Confirm that FIFO was actually enabled by checking the final console output for:

```text
fifo=active
```

If it says `fifo=off`, the FIFO request failed and the run should not be labelled as a FIFO result.

### 3. Water-Loaded Test

Repeat the same open-loop and PI experiments with the motor operating in water. Use different filenames:

```bash
sudo ./mixr1_daemon --test --sweep --fifo --no-pi \
  --duration=10 --target=0 --csv=water_raw_fifo.csv

sudo ./mixr1_daemon --test --sweep --fifo --pi \
  --target=2500 --duration=10 --csv=water_pi_fifo.csv
```

The no-load and water-loaded results must be reported separately because water introduces disturbance, drag, and turbulence.

## Python Analysis

The analysis script is [analyze_pwm_sweep.py](analyze_pwm_sweep.py).

Run it with open-loop and PI files:

```bash
python3 analyze_pwm_sweep.py \
  raw_fifo.csv pi_fifo.csv \
  --settle=2 \
  --settling-band=0.05 \
  --output=fifo_analysis
```

For a water-loaded comparison:

```bash
python3 analyze_pwm_sweep.py \
  water_raw_fifo.csv water_pi_fifo.csv \
  --settle=2 \
  --settling-band=0.05 \
  --output=water_analysis
```

The script discards the first 2 seconds of each step for steady-state statistics. This removes most of the acceleration transient. It still uses the complete step response for rise time, settling time, and overshoot.

The default timing-outlier limit is 500 microseconds. Samples above this limit are excluded from clean RPM statistics but are still reported as timing outliers.

Generated outputs include:

- `*_summary.csv`: per-step metrics
- `*_comparison.csv`: PI error versus the open-loop baseline
- `rpm_vs_pwm.png`: RPM characterization and tracking graph
- `jitter_vs_pwm.png`: loop-period jitter graph
- `pi_performance.png`: PI rise, settling, overshoot, and error graphs
- `rpm_hist_*.png`: raw RPM histograms
- `controller_analysis_report.md`: thesis/presentation explanation

## Metrics

### Mean RPM

The average raw RPM during the settled part of a step.

### Standard Deviation

The spread of raw RPM values, including timing effects.

### Clean Standard Deviation

The raw RPM spread after timing outliers above the configured lateness threshold are excluded.

### Loop Jitter

The standard deviation of the measured loop period:

$$
Jitter = std(T_{loop})
$$

The nominal loop period is 10,000 microseconds.

### Maximum Lateness

The largest delay between the intended absolute wake deadline and the actual wake time.

### Timing Outliers

Samples whose `late_us` exceeds the configured limit, normally 500 microseconds.

### Rise Time

The time for filtered RPM to move from 10% to 90% of the target.

### Settling Time

The time after which filtered RPM remains within the selected target band. The current default is plus or minus 5%.

A result of `nan` means the response did not remain inside the selected band before the step ended. This is a valid result, not a missing value.

### Overshoot

The maximum amount above the target:

$$
Overshoot(\%) = 100\frac{RPM_{peak}-RPM_{target}}{RPM_{target}}
$$

Negative overshoot is clipped to zero.

### Steady-State Error

The mean absolute error during the final fifth of a target step:

$$
 e_{ss} = mean\left(\left|RPM_{target}-RPM\right|\right)
$$

## First-Order Motor Model

The dynamic model should be written as:

$$
G(s)=\frac{K}{s+a}
$$

An equivalent standard form is:

$$
G(s)=\frac{K_m}{\tau s+1},\qquad \tau=\frac{1}{a}
$$

For a unit step, the response is:

$$
 y(t)=\frac{K}{a}\left(1-e^{-at}\right)
$$

or equivalently:

$$
 y(t)=K_m\left(1-e^{-t/\tau}\right)
$$

Definitions:

- $K$ or $K_m$: motor input-to-speed gain
- $a$: response rate
- $\tau$: time constant
- $\theta$: transport or measurement delay, if a delay is included

The open-loop PWM sweep provides the motor's steady-state PWM-to-RPM relationship. A dynamic first-order fit should use the raw time response of each step, not only the final RPM values.

The current analyzer also writes a simple static linear fit of RPM versus PWM. That fit is useful for characterization, but it is not by itself the complete dynamic first-order model.

## PI Gain Selection and IMC

The current C++ implementation uses gain scheduling in [include/config.hpp](include/config.hpp):

```cpp
constexpr std::array<GainTier, 5> PI_SCHEDULE = {{
    {0.00,    1.9646, 48.5934},
    {342.81,  1.5387, 40.3388},
    {861.56,  1.5195, 41.8610},
    {1378.44, 1.4901, 37.8287},
    {1766.56, 1.4740, 40.1298}
}};
```

The program linearly interpolates between these gain points based on target RPM.

The important accuracy point is that the current source does not calculate IMC gains at runtime. Therefore:

- If the gain values were calculated externally using IMC equations, describe them as IMC-derived scheduled gains.
- If they were selected through experiments, describe them as experimentally tuned gain-scheduling values.
- Do not call them IMC gains unless the model parameters and IMC tuning rule used to calculate them are documented.

For a first-order-plus-delay model:

$$
G(s)=\frac{K_m e^{-\theta s}}{\tau s+1}
$$

A common IMC PI rule is:

$$
K_p=\frac{\tau}{K_m(\lambda+\theta)}
$$

$$
K_i=\frac{K_p}{\tau}
$$

where $\lambda$ is the desired closed-loop time constant. A smaller $\lambda$ gives a faster response but generally reduces robustness. A larger $\lambda$ gives a slower but more robust response.

## Controller Improvement

The correct comparison is not simply PI RPM versus open-loop RPM because the two experiments have different inputs:

- Open loop input: PWM percentage
- PI input: target RPM

For each PI target, the analyzer estimates the corresponding open-loop PWM using the 0 to 2500 RPM range and uses the open-loop measured RPM at that PWM as the baseline.

The improvement is calculated as:

$$
Improvement(\%)=100\left(1-\frac{e_{PI}}{e_{open}}
ight)
$$

where:

- $e_{PI}$ is PI steady-state error
- $e_{open}$ is open-loop steady-state error at the equivalent PWM/target

Positive improvement means PI reduced error. Negative improvement means open loop happened to be closer to that target in that particular test point.

Do not hide negative values. Explain them and repeat the test if the result is caused by noise, an unsuitable baseline mapping, or insufficient settling time.

## Current Recorded Results

The available FIFO files showed the following general result:

- FIFO loop-period jitter was approximately single-digit to low double-digit microseconds.
- No-FIFO jitter was hundreds to over a thousand microseconds.
- FIFO maximum lateness was below approximately 160 microseconds in the recorded run.
- No-FIFO produced repeated multi-millisecond lateness values.
- No-FIFO also produced large raw RPM spikes at the same time as timing delays.
- This supports the professor's point that scheduler timing can distort RPM calculated from encoder counts over a nominal 10 ms interval.

The `raw_fifo.csv` open-loop RPM results were approximately:

| PWM | Mean RPM |
|---:|---:|
| 0% | 0 |
| 10% | 230 |
| 20% | 495 |
| 30% | 751 |
| 40% | 1015 |
| 50% | 1277 |
| 60% | 1537 |
| 70% | 1796 |
| 80% | 2053 |
| 90% | 2317 |
| 100% | 2492 |

These results support the approximate statement that 10% PWM corresponds to 250 RPM, but the measured data should be used in the final report instead of assuming perfect proportionality.

## Recommended Presentation Flow

```text
Problem and control objective
        |
        v
Open-loop PWM experiment
        |
        v
Raw RPM and timing data
        |
        v
First-order model identification
        |
        v
IMC or documented PI gain selection
        |
        v
Closed-loop PI experiment
        |
        v
No-load versus water-loaded comparison
        |
        v
Rise time, settling time, overshoot, steady-state error
        |
        v
Controller improvement and scheduling impact
```

## Remaining Work

1. Run the open-loop sweep with no FIFO and FIFO under the same physical condition.
2. Run the PI target sweep with no FIFO and FIFO under the same condition.
3. Repeat both tests under water load.
4. Confirm every FIFO run reports `fifo=active`.
5. Generate the analysis directories for each condition.
6. Fit and document the dynamic first-order parameters $K$, $a$, or $K_m$, $\tau$.
7. Document whether the stored PI schedule is IMC-derived or experimentally tuned.
8. Compare the controller using the same target speeds and physical load.
9. Add the generated plots and the flowchart from [scheduling_architecture.md](scheduling_architecture.md) to the presentation.
10. Include raw data plots before filtered plots.

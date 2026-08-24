# Motor and PI Controller Test Report

## Experiment interpretation

The open-loop test applies fixed PWM levels from 0% to 100%. The PI test applies matching RPM setpoints. Open-loop data is the plant baseline; PI data measures feedback tracking. FIFO and non-FIFO runs are scheduling comparisons.

## Metrics

Steady-state statistics discard the first configured settling interval and remove timing outliers above 500 us. Transient settling uses filtered RPM and a +/-5% target band.

- Mean RPM: average measured raw RPM.
- Standard deviation: spread of raw RPM, including timing effects.
- Clean standard deviation: raw RPM spread after timing outliers are excluded.
- Loop jitter: standard deviation of the measured loop period.
- Maximum lateness: largest delay after the absolute scheduled wake time.
- Timing outliers: samples whose lateness exceeds the configured threshold.
- Rise time: time for filtered RPM to move from 10% to 90% of target.
- Settling time: time after which filtered RPM remains within the settling band.
- Overshoot: maximum amount above target, expressed as a percentage.
- Steady-state error: mean absolute target error in the final fifth of a step.

## First-order model

Use the identified motor model:

$$G(s)=\frac{K}{s+a}=\frac{K_m}{\tau s+1},\qquad \tau=\frac{1}{a}$$

Here, K (or K_m) is the input-to-speed gain, a is the response rate, and tau is the time constant. Estimate these from the raw open-loop step responses, not from the PI response. The PWM sweep supplies the steady-state input/output relationship; the time trace supplies the dynamic time constant.

## IMC and gain selection

For a first-order-plus-delay model, an IMC PI rule is:

$$G(s)=\frac{K_m e^{-\theta s}}{\tau s+1},\qquad K_p=\frac{\tau}{K_m(\lambda+\theta)},\qquad K_i=\frac{K_p}{\tau}$$

lambda sets the desired closed-loop speed: smaller lambda is faster but less robust, while larger lambda is slower but more robust. The current C++ implementation uses the measured gain schedule in config.hpp and linearly interpolates Kp and Ki by target RPM. Describe these as IMC-derived only if the scheduled values were calculated with the IMC equations above; otherwise describe them as experimentally tuned gain-schedule values.

## Controller improvement

PI improvement is evaluated against the nearest measured open-loop operating point:

$$Improvement=100\left(1-\frac{e_{PI}}{e_{open}}\right)\%$$

where e is mean absolute steady-state RPM error. Use the generated comparison CSV and PI performance plot. Report water-loaded and no-load tests separately because water turbulence changes the plant.

The current open-loop settled data has an approximate linear steady-state fit of RPM = 25.54 * PWM(%) + -7.73. This is a static characterization, not the dynamic first-order model by itself.

PI FIFO: mean PI steady-state error across valid targets = 17.25 RPM.

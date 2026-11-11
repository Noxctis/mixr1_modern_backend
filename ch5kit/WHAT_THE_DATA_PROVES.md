# What each dataset is for, mapped to Chapter 5

| Claim in Chapter 5 | Data that supports it | Menu | Output you read |
|---|---|---|---|
| The CET estimator is **accurate** (no bias) | Mean speed of each estimator minus the speed from the **index pulses** (an independent hardware reference: one pulse per revolution, timed to 5 us, independent of the quadrature decoding, the PPR and every estimator) | 2, 3 | `ACCURACY` table, `fig_accuracy_bias.pdf` |
| CET is **more precise** than the frequency-based DCT | SD of the raw speed, open loop, 5 estimators x 10 speeds x 3 repeats | 2, 3 | `OPEN LOOP` table, `fig_openloop_sd.pdf` |
| The jitter is **inherent** (shaft-synchronous), not estimator noise | Spectrum of the raw speed: lines at 1x, 2x, 3x the rotation frequency, same in every estimator | 2, 3 | `fig_openloop_spectra.pdf`, `LINE AT 1x` table |
| DCT fails **in closed loop** and CET does not | Closed-loop staircase 10 to 1500 rpm with the old gains, all estimators | 2, 3 | `CLOSED LOOP, old gains` table, `fig_closedloop.pdf` |
| The failure is **delay**, not noise | DCT at 5, 10 and 20 ms windows (5 ms refreshes every loop: if the oscillation disappears, it was the refresh delay) | 2, 3 | the three DCT columns |
| **Which** CET (aligned or newest) | Age of the estimate (`age_us`) against the predicted `Ta` and `Ta/4`, SD, lost edges | 2, 3 | `AGE` table, `RECOMMENDED ESTIMATOR` |
| The **gains** are right | Plant identified from your own motor (K, tau, dead time), IMC rule, closed-loop check | 2 (automatic), 4 | `plant/identification.csv`, check table |
| **256 PPR** is a sound choice | Each PPR: edges per revolution, lost edges, SD, update fraction, CPU load, tachometer comparison | 5, 6 | `results_simple/RESULTS.txt` |
| The **Power number** is trustworthy, and what the estimator does to it | The same N_Re--N_Po sweep with torque, once per estimator, speed also from the index pulses | 7, 8 | `EFFECT OF THE SPEED ESTIMATOR ON N_Po`, `DOMINANT UNCERTAINTY` |
| **Torque** chain: drift, tare, calibration against masses | Cold-start log (90 min), masses on a lever arm up and down 3 times | 9 (guided 9, 10) | `torque_drift`, `torque_validation` |
| Duty-to-speed, breakaway, hysteresis (TC-U6) | Duty sweeps up and down | 9 (guided 3) | `duty_coarse`, `duty_fine` |
| Step response, power limiter, load disturbance (TC-S1) | Guided 4, 5, 6, 7 | 9 | `steps`, `disturbance_*` |
| Endurance, Mode 1, dashboard, logging integrity, plausibility vs Rushton | Guided 11, 15, dashboard, `integrity`, `plausibility` | 9 | tables in `analysis/` |

| The shaft rides on the encoder: is the jitter **inherent**? | FFT of the raw speed (equivalent encoder angle against 0.25 deg, amplitude against speed), the ripple **folded against shaft angle** with the index pulses (same phase at every speed), dial-indicator runout | 2, 3 (+ paper) | `SHAFT / ECCENTRICITY`, `RIPPLE VERSUS SHAFT ANGLE`, `fig_shaft_*.pdf` |
| One **Kp/Ki** works for **both impellers (96 and 174 mm), baffles in and out** | Plant of each condition at low, mid and high speed (K falls with speed as the drag grows), a search for the pair that keeps PM >= 60 deg and GM >= 8 dB on all of them, closed-loop check in each | 4 (choices 1, 2, 3) | `COMMON PAIR` table, `WHAT THE COMPROMISE COSTS`, check tables |
| Resolution (PPR) under CET: effect on accuracy | bias against the index speed, by PPR | 5, 6 | `ACCURACY BY PPR` |

## The argument for the thesis, in five sentences
1. The speed must be measured accurately because the Power number depends on it as n^-2 and the controller uses it as feedback.
2. A frequency-based estimator (DCT) counts edges over a window: one count of quantization per window, timing taken from the loop clock, and a refresh pattern that depends on the loop jitter.
3. CET uses edge time stamps from the sampler, whole encoder periods, and is formed at the controller's instant; two forms are implemented, one ending at the last aligned edge and one at the newest edge.
4. Measured against the index pulses (independent of every estimator) CET has no bias, DCT does not behave as well in closed loop, and the part of the scatter that remains is a ripple locked to the shaft angle that no estimator can remove.
5. The resolution of 256 PPR and the gains for both impellers follow from the measured plant, the edge rate the Pi can decode, and the stability margins.

Two things that are easy to get wrong in the thesis:
1. **N_Po depends on speed as n^-2, not n^-3.** The shaft power is measured as torque times speed, so `N_Po = P/(rho n^3 D^5) = 2 pi tau / (rho n^2 D^5)`. A speed error of x % gives about 2x % in N_Po. N_Re depends on speed as n^1.
2. **Precision is not accuracy.** The standard deviation says how much the estimate scatters; only a comparison with an independent reference says whether its mean is right. That is why the index-pulse reference is in every run.

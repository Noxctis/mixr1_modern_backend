# PI FIFO and PI No-FIFO Thesis Graphs

The experiment applies the same target-RPM steps to the PI-controlled motor under FIFO and No-FIFO scheduling. Each graph uses per-target summary values calculated from the raw measurements.

## Motor speed and RPM variation

The mean speed graph shows measured RPM against the requested target RPM. The error bars show plus or minus one RPM standard deviation. The distance between the measured speed and the target represents tracking error; the error bars represent speed repeatability.

The RPM standard-deviation graph measures variation in the motor output around its mean speed. It is calculated from raw RPM samples and is separate from timing jitter. Lower values indicate more repeatable PI-controlled speed.

## Initial IMC-based controller

The PI gains were selected from the initial motor-model curve fit using an IMC design. The fitted model estimates the motor gain and time constant; these determine the proportional and integral gains. The same gain schedule is used for FIFO and No-FIFO, so the scheduling comparison changes execution timing rather than the controller tuning. The gains are interpolated according to the target RPM.

The mean PI steady-state absolute tracking error is approximately 5.87 RPM with FIFO and 6.08 RPM without FIFO. The similar errors indicate that both conditions can track the target, while the timing graphs show the difference in execution determinism.

## Control-loop timing

The mean control-loop period is the average time between consecutive loop executions. The intended period is 10,000 microseconds, or 10 milliseconds. Both conditions have a mean close to this value, so the average loop rate is maintained. The mean alone does not reveal occasional delays.

Timing jitter is calculated from the measured loop_period_us values. For each sample, the actual interval between loop executions is compared with the typical period. RMS jitter summarizes overall timing variation; P95 and P99 show the deviation below which 95% and 99% of samples fall. Lower values indicate more deterministic controller execution. Average RMS jitter is 4.71 us for FIFO and 7.77 us for No-FIFO. Average P99 jitter is 15.48 us for FIFO and 33.26 us for No-FIFO.

Scheduled-loop lateness is different from jitter. It measures how late the loop executes compared with its absolute scheduled wake-up time. P95 lateness represents typical delayed behavior, while maximum lateness shows the worst interruption. FIFO's maximum lateness is approximately 46 us; No-FIFO reaches approximately 156 us.

The timing-outlier-rate graph counts samples whose lateness exceeds 500 us. FIFO has 0.0 outliers out of 8798.0 samples (0.00%). No-FIFO has 0.0 outliers (0.00%). These outliers represent severe scheduling interruptions that can delay feedback processing and motor updates.

## Thesis conclusion

The IMC-based PI controller achieves similar average tracking accuracy in both tests, with mean steady-state errors of 5.87 RPM for FIFO and 6.08 RPM for No-FIFO. FIFO provides the more deterministic execution schedule, with lower timing variation and fewer severe delays. This supports using FIFO to improve the repeatability of PI control without changing the controller gains.

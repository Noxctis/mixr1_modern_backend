# FIFO and No-FIFO Thesis Graphs

The experiment applies the same PWM sweep to the motor under FIFO and No-FIFO scheduling. Each graph uses the per-PWM summary values calculated from the raw measurements.

## Motor speed and RPM variation

The mean speed graph shows the average motor RPM produced at each PWM level. The error bars show plus or minus one RPM standard deviation. The mean response is the motor's average output; the error bars show repeatability. Larger error bars mean the motor speed changes more during the same test condition.

The RPM standard-deviation graph measures motor-output variation directly. It is calculated from the individual raw RPM samples within each PWM step. It is not the same as timing jitter. In this experiment, No-FIFO reaches approximately 154 RPM standard deviation at 100% PWM, compared with approximately 75 RPM for FIFO, showing less repeatable motor speed without FIFO.

## Control-loop timing

The mean control-loop period is the average time between consecutive loop executions. The intended period is 10,000 microseconds, or 10 milliseconds. Both conditions have a mean close to this value, so the average loop rate is maintained. The mean alone does not reveal occasional delays.

Timing jitter is calculated from the measured loop_period_us values. For each sample, the actual interval between loop executions is compared with the typical period. RMS jitter summarizes overall timing variation; P95 and P99 show the deviation below which 95% and 99% of samples fall. Lower values indicate more deterministic controller execution. Average RMS jitter is 1.94 us for FIFO and 262.07 us for No-FIFO. Average P99 jitter is 8.05 us for FIFO and 754.15 us for No-FIFO.

Scheduled-loop lateness is different from jitter. It measures how late the loop executes compared with its absolute scheduled wake-up time. P95 lateness represents typical delayed behavior, while maximum lateness shows the worst interruption. FIFO's maximum lateness is approximately 76 us; No-FIFO reaches approximately 9107 us.

The timing-outlier-rate graph counts samples whose lateness exceeds 500 us. FIFO has 0.0 outliers out of 8796.0 samples (0.00%). No-FIFO has 55.0 outliers (0.63%). These outliers represent severe scheduling interruptions that can delay feedback processing and motor updates.

## Thesis conclusion

FIFO does not substantially change the average 10 ms loop rate, but it greatly reduces timing variation and worst-case delays. The lower timing variation is accompanied by lower RPM variation, indicating more repeatable motor behavior. No-FIFO maintains a similar average loop period while allowing large timing interruptions, so average period alone would hide the real-time performance difference.

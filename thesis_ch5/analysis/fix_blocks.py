import sys
p = sys.argv[1]; s = open(p, encoding="utf-8").read()
R = [
(r"with the 96~mm impeller and the vessel filled, and the load was disturbed", r"in the filled vessel, and the load was disturbed"),
(r"Speed of the 96~mm impeller in the filled vessel for a constant duty", r"Speed in the filled vessel for a constant duty"),
(r"Each load was applied five times, the reading was the mean of 10 samples after the zero (baseline) of the trial was subtracted, and the order of the loads was not monotonic.", r"Each load was applied five times, and the reading is the mean of 10 samples after the zero (baseline) of the trial is subtracted."),
(r"The error is negative at every load but one, between $-0.05$ and $-1.2$~\%, with the largest relative errors at the two lowest loads (0.024 and 0.049~N\,m), where the sensor resolution of one~mN\,m is 2 to 4~\% of the load.", r"The mean error is negative at every load, between $-0.05$ and $-1.2$~\%, with the largest relative errors at the two lowest loads (0.024 and 0.049~N\,m), where an absolute error of 0.3~mN\,m is 1.2~\% of the load."),
(r"and the sensor reads about 0.2 to 0.5~\% low at the loads of the experiment.", r"and above 0.07~N\,m the sensor reads 0.05 to 0.6~\% low."),
(r"in which the motor delivers a large torque at a low speed and its losses in the windings dominate; the difference is not an error of the torque sensor.", r"in which the motor delivers a large torque at a low speed, so that losses in the windings probably dominate; this is not tested here."),
(r"16 to 21~\% for the 174~mm impeller", r"16 to 21~\% for the 174~mm impeller above 150~rpm"),
(r"because the speed hunts for a few seconds after the disturbance (the burst of oscillation in the middle panel of Figure~\ref{fig:disturbance}), which a faster loop damps.", r"because the speed oscillates for several seconds after the disturbance (the burst in the middle panel of Figure~\ref{fig:disturbance}), which the faster loop damps sooner."),
(r"With $a = 1$ the duty changes by 0.12 to 0.44~\% from one loop to the next at steady state, because it follows the quantization of the speed estimate; with $a = 0.2$ or 0.15 it does not change at the one-count resolution of the log, at the same rise time, settling time, and overshoot.", r"With $a = 1$ the duty changes by 0.12 to 0.44~\% from one loop to the next at steady state, because it follows the quantization of the speed estimate. With $a = 0.2$ or 0.15 and $K_i \geq 6$ it does not change at the one-count resolution of the log, and at $K_i = 1.454$ it changes by 0.16 to 0.24~\%, with the same rise time, settling time, and overshoot as without the filter."),
(r"Changing $K_p$ from 2.629 to 3.155 moves the metrics by less than one setting-to-setting difference.", r"Changing $K_p$ from 2.629 to 3.155 changes the rise and settling times by less than 0.2~s and the overshoot by about one percentage point."),
(r"are the setting of this matrix with the fastest response, the smallest speed scatter, and a quiet duty, and the overshoot of 11.6~\% as its cost.", r"are among the fastest settings of this matrix, with a speed scatter of 0.6 to 0.7~rpm and a quiet duty, and the overshoot of 11.6~\% is the cost."),
(r"The refit of the steps from 410 to 3276 counts", r"The refit of the steps from 410 to 2867 counts"),
(r"% TODO: confirm with the run notes that these are the gains of config.hpp", r"% TODO: state the impeller of the disturbance runs (the duty of 15 % for 350 rpm is that of the 96 mm impeller) and confirm with the run notes that these are the gains of config.hpp"),
]
for o, n in R:
    assert o in s, o[:70]
    s = s.replace(o, n)
open(p, "w", encoding="utf-8", newline="\n").write(s); print("ok")

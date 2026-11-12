import sys
p = sys.argv[1]; s = open(p, encoding="utf-8").read()
i = s.index("A slower loop is calmer. The noise of the estimate reaches")
j = s.index("is at 25~Hz (157~rad/s)", i)
new = (r"A slower loop is calmer. The noise of the estimate reaches the duty through $K_p = \omega_c/(K A)$: at 29~rad/s it turns 3.8~rpm of CET noise into 0.13~\% of the full-scale duty and 21~rpm of DCT noise into 0.7~\%, and at 5~rad/s 0.02 and 0.12~\% (Figure~\ref{fig:design-tradeoff}c). The shaft-synchronous ripple of Section~\ref{subsec:agit-fft} adds to this: it ")
s = s[:i] + new + s[j:]
old = r"(c) Time to reach 90~\% of the step and the overshoot against $\omega_c$."
assert old in s
s = s.replace(old, r"(c) Time to reach 90~\% of the step, and the standard deviation of the duty that the noise of the speed estimate produces through $K_p$ (3.8~rpm for CET and 21~rpm for DCT at 20~ms, from Table~\ref{tab:ol-sd}), against $\omega_c$.")
open(p, "w", encoding="utf-8", newline="\n").write(s); print("ok")

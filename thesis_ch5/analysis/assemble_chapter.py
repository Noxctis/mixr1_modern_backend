"""Assemble the revised chapter 5: python assemble_chapter.py <thesis_ch5 dir>  (reads chapter5.tex and parts/, writes chapter5_new.tex)"""
import sys, os
root = sys.argv[1]
s = open(os.path.join(root, "chapter5.tex"), encoding="utf-8").read()
blocks = open(os.path.join(root, "parts", "misc_blocks.tex"), encoding="utf-8").read().split("%%%")
B = {}
for b in blocks[1:]:
    k, _, body = b.partition("\n"); B[k.strip()] = body.strip("\n") + "\n"
ctl = open(os.path.join(root, "parts", "ctl_law.tex"), encoding="utf-8").read()
theory = open(os.path.join(root, "parts", "pi_theory.tex"), encoding="utf-8").read()


def span(s, start, end, end_in_line=True):
    i = s.index(start); i = s.rfind("\n", 0, i) + 1
    j = s.index(end, i); j = s.index("\n", j) + 1 if end_in_line else j
    return i, j


def replace_span(s, start, end, new, end_in_line=True):
    i, j = span(s, start, end, end_in_line); return s[:i] + new + ("\n" if not new.endswith("\n") else "") + s[j:]


def rep(s, o, n):
    assert o in s, "missing: " + o[:70]; return s.replace(o, n, 1)


# ---- plausibility section: literature belongs to the 96 mm configuration
s = rep(s, r"and the literature curve tabulated by Brown et al.\ (curve~10 of the reference, here the ``literature'' curve).", r"and the literature curve tabulated by Brown et al.\ (curve~10 of the reference, here the ``literature'' curve), which is the curve of the 96~mm impeller configuration and exists for no other.")
s = rep(s, r"(a) Power number against Reynolds number, with the literature value (dashed). (b) Torque against speed. (c) Ratio of the CFD torque to the measured torque; open markers: $N_{Re} < 4\times10^{4}$; the shaded band is $\pm10$~\%.",
        r"(a) Power number against Reynolds number over the whole range of the CFD (1 to $10^{6}$) and of the literature curve (squares, dashed), so that every point of the three is shown; the shaded band is the range of panel~b. (b) The turbulent range on a linear axis. (c) Torque against speed. (d) Ratio of the CFD torque to the measured torque; open markers: $N_{Re} < 4\times10^{4}$; the shaded band is $\pm10$~\%.")
s = rep(s, r"As Figure~\ref{fig:cfd-96} for the 174~mm impeller.", r"As Figure~\ref{fig:cfd-96} for the 174~mm impeller, with the CFD and the measurement only: there is no literature curve for this impeller.")
s = rep(s, r"(panel~c of both figures)", r"(panel~d of both figures)")
i, j = span(s, r"\paragraph{Comparison with the literature curve.}", r"\paragraph{Limits of the comparison.}", end_in_line=False)
lit = r"""\paragraph{Comparison with the literature curve.}
The curve of Brown et al.\ (curve~10) is that of the 96~mm impeller configuration, so it is compared with the 96~mm impeller only; the 174~mm impeller is compared with its CFD alone (Figure~\ref{fig:cfd-174}). Panel~(a) of Figure~\ref{fig:cfd-96} shows the three on one set of axes. The literature curve falls from 12 at $N_{Re} = 3$ to a flat level of 1.5 to 1.8 above $N_{Re} \approx 100$ and stays there to $10^{6}$. The CFD follows it at the high end, with 1.8 at $N_{Re} = 10^{4}$ and 1.5 at $10^{5}$ against 1.7 and 1.8, but it rises far above it below $5\times10^{3}$: 2.9 at 5000, 11.5 at 2000, and 36 at 1000, against 1.7, 1.6, and 1.6. The curve of the measurement lies at $N_{Po} \approx 2$ over $N_{Re} = 4\times10^{4}$ to $1.2\times10^{5}$, on the same level as the other two. Table~\ref{tab:rmse-lit} gives the error over the Reynolds numbers where each series exists. The best 96~mm run is 14~\% above the literature curve (RMSE 0.31) and the CFD is 18~\% below it (RMSE 0.32), so the measurement and the CFD lie on opposite sides of the literature curve by about the same amount, and the measurement is closer to the literature than the CFD is. The Armfield curves are far from the literature curve: the stock paddle by a factor of 5.4 (RMSE 8.0), the custom paddle 59~\% below it (RMSE 1.1) and the custom blades 19~\% below it on average (RMSE 2.4; the mean bias understates the error because the large positive errors at low $N_{Re}$ cancel the negative errors at high $N_{Re}$). The 96~mm data and the CFD therefore agree with the literature to within 14 to 18~\%, and the student curves do not agree with any of them. The geometry of the 96~mm impeller is not exactly that of the configuration of the literature curve, so the offset is not attributed to the measurement. % TODO: state the geometry of the reference configuration (impeller type, D/T, C/T, number of baffles) and in which ratios the 96 mm impeller departs from it.

\begin{table}[htbp]
    \centering
    \caption[The 96 mm data, the CFD and the Armfield data against the literature curve]{Error of the best 96~mm run, the 96~mm CFD, and the Armfield curves against the literature curve (Brown et al., curve~10), interpolated in the logarithms to the Reynolds number of each point. 96~mm run and CFD: $N_{Re}\geq4\times10^{4}$ and, for the CFD, up to the highest experimental value; Armfield: $N_{Re}\geq4000$. Bias is the mean of $(N_{Po} - N_{Po,\mathrm{lit}})/N_{Po,\mathrm{lit}}$.}
    \label{tab:rmse-lit}
    \small
    \resizebox{\textwidth}{!}{\input{Chapter5/Figs/best_vs_literature_96}}
\end{table}

"""
s = s[:i] + lit + s[j:]
# remove the literature figure block, if any
if r"\label{fig:literature}" in s:
    i, j = span(s, r"\includegraphics[width=0.7\textwidth]{fig_literature}", r"\end{figure}")
    i = s.rfind(r"\begin{figure}", 0, i); s = s[:i] + s[j:]

# ---- controller law
s = replace_span(s, r"% TODO: Equation 5.2 -- the discrete control law", r"Figure 5.5 -- controller block diagram showing the dynamic ceiling path.", ctl)
# ---- theory
i, j = span(s, r"\paragraph{Derivation of the gain rule.}", r"\paragraph{Check against measured closed-loop step responses.}", end_in_line=False); s = s[:i] + theory + s[j:]
# ---- plant figure
s = rep(s, r"The 100~Hz log limits the accuracy of this identification.", B["PLANTFIG"] + "\n" + r"The 100~Hz log limits the accuracy of this identification.")
# ---- step figure
s = rep(s, r"The earlier harness runs differed in one respect", B["PISTEP"] + "\n" + r"The earlier harness runs differed in one respect")
# ---- gain matrix
s = replace_span(s, r"% TODO: state the gains of the final tuning and how they were obtained.", r"referenced in passing here.", B["GAINMATRIX"])
# ---- tracking
s = replace_span(s, r"% TODO: TC-S1. Step the setpoint across the range", r"this is the headline comparison in Chapter~\ref{ch:validation}.", B["TRACKING"] + "\n% TODO: compare the steady-state error with the open-loop knob positioning of the CEK-A (Section~\\ref{subsec:che3214l}); this is the headline comparison in Chapter~\\ref{ch:validation}.\n")
# ---- loadcell
s = replace_span(s, r"% TODO: TC-U4. No independent torque gauge is available", r"Figure 5.9 (measured vs theoretical with the 1:1 line).", B["LOADCELL"])
# ---- driver
s = replace_span(s, r"% TODO: TC-U6. Sweep duty from 0 to 100 percent", r"controller has to overcome.", B["DRIVER"])
open(os.path.join(root, "chapter5_new.tex"), "w", encoding="utf-8", newline="\n").write(s); print("assembled", len(s))

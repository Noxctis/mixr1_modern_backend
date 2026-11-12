import sys
p = sys.argv[1]; s = open(p, encoding="utf-8").read()
one = [
(r"The interval between consecutive pulses gives a speed, $60/\Delta t$, that is independent of the quadrature decoding and of the resolution setting, and",
 "The interval $\\Delta t$ between consecutive pulses gives a speed\n\\begin{equation}\n    n_{\\mathrm{idx}} = \\frac{60}{\\Delta t}\n    \\label{eq:idx-speed}\n\\end{equation}\nthat is independent of the quadrature decoding and of the resolution setting, and"),
(r"and the mean speed is $(N-1) \cdot 60 \times 10^{6} / (t_N - t_1)$ in rpm with the pulse times in microseconds.",
 "and the mean speed is\n\\begin{equation}\n    \\bar{n}_{\\mathrm{idx}} = \\frac{(N-1)\\cdot 60\\times10^{6}}{t_N - t_1}\n    \\label{eq:idx-mean}\n\\end{equation}\nin rpm, with the pulse times in microseconds."),
(r"The corresponding time constant is $\tau = -T_s / \ln(1 - \alpha) = 61.5$~ms, equivalent to a cut-off of $1/(2\pi\tau) = 2.6$~Hz, and the filter reaches 95~\% of a step in 18.4 samples, 184~ms. For uncorrelated noise on the input the output variance is reduced to $\alpha/(2 - \alpha) = 8.1$~\% of the input variance, a reduction of the standard deviation to 28~\%.",
 "The corresponding time constant, cut-off frequency, and variance reduction for uncorrelated input noise are\n\\begin{equation}\n    \\tau = -\\frac{T_s}{\\ln(1 - \\alpha)} = 61.5\\ \\text{ms}, \\qquad f_c = \\frac{1}{2\\pi\\tau} = 2.6\\ \\text{Hz}, \\qquad \\frac{\\sigma_{\\mathrm{out}}^{2}}{\\sigma_{\\mathrm{in}}^{2}} = \\frac{\\alpha}{2 - \\alpha} = 8.1\\ \\%\n    \\label{eq:ema-props}\n\\end{equation}\nso the filter reaches 95~\\% of a step in 18.4 samples, 184~ms, and the standard deviation of uncorrelated noise falls to 28~\\%."),
]
for o, n in one:
    assert o in s, o[:60]; s = s.replace(o, n)
allr = [
(r"60/(N S_{\min})", r"\frac{60}{N S_{\min}}"), (r"60/(K\, T_s)", r"\frac{60}{K\,T_s}"), (r"60/(K \times 0.1\,\text{s})", r"\frac{60}{K \times 0.1\,\text{s}}"),
(r"K \geq 60/(n_{\text{low}} T_s)", r"K \geq \frac{60}{n_{\text{low}} T_s}"), (r"60/(T_w N)/\sqrt{12}", r"\frac{60}{T_w N\sqrt{12}}"), (r"60/(T_s N)", r"\frac{60}{T_s N}"), (r"60/(T_w N)", r"\frac{60}{T_w N}"),
(r"T_a = 60/(nK)", r"T_a = \frac{60}{nK}"), (r"T_a = 60\times10^{6}/(256\,n)", r"T_a = \frac{60\times10^{6}}{256\,n}"),
(r"$h/T_s$", r"$\frac{h}{T_s}$"), (r"$h/T_a$", r"$\frac{h}{T_a}$"), (r"$4/\Delta c$", r"$\frac{4}{\Delta c}$"), (r"f_{rot} = n/60", r"f_{rot} = \frac{n}{60}"), (r"$A_1/n$", r"$\frac{A_1}{n}$"),
(r"$[e(\theta_k) - e(\theta_{k-1})]/\Delta t$", r"$\dfrac{e(\theta_k) - e(\theta_{k-1})}{\Delta t}$"), (r"$|\sin(\pi f T_s)/(\pi f T_s)|$", r"$\left|\dfrac{\sin(\pi f T_s)}{\pi f T_s}\right|$"),
(r"$s = -K_i/K_p$", r"$s = -\dfrac{K_i}{K_p}$"), (r"$\ln 9/\omega_c \approx 2.2/\omega_c$", r"$\dfrac{\ln 9}{\omega_c} \approx \dfrac{2.2}{\omega_c}$"), (r"$3/\omega_c$", r"$\dfrac{3}{\omega_c}$"), (r"$1/\omega_c$", r"$\dfrac{1}{\omega_c}$"),
(r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}} = 0.73$", r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}} = 0.73$"),
]
for o, n in allr:
    if o not in s: print("skip", o[:40]); continue
    s = s.replace(o, n)
open(p, "w", encoding="utf-8", newline="\n").write(s); print("ok")

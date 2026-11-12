#!/usr/bin/env python3
"""
npo_figs_v3.py - per-impeller CFD figures, zoomed out so that every CFD point, every MIXR-1 point and the whole literature curve are shown.
    96 mm : CFD + MIXR-1 + literature (Brown, curve 10 is the 96 mm configuration)
    174 mm: CFD + MIXR-1 only (no literature curve exists for it)
    python npo_figs_v3.py --data "<Research SeBoCa Test>" --out ../Figs [--best96 TS] [--best174 TS]
Also writes best_vs_literature_96.tex (literature table without the 174 mm rows).
"""
import argparse, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from npo_rmse import D, FLOOR, BROWN, ARM, load_cfd, n_re, n_po, loglog, rmse
from npo_figs_v2 import style, sci, load_all, rank_runs, series, NSTEPS, C, MK


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True); ap.add_argument("--best96"); ap.add_argument("--best174"); a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); style(); cfd = load_cfd(os.path.join(a.data, "mixr_dashboard.py")); S, BEST = {}, {}
    for imp in D:
        R = load_all(a.data, imp); K = rank_runs({k: v for k, v in R.items() if len(v) >= NSTEPS[imp] - 1}, imp)
        BEST[imp] = getattr(a, "best" + imp[:-2]) or K[K.eligible].run.iloc[0]; S[imp] = series(R[BEST[imp]], imp)
    for imp in D:
        s, (cr, ct) = S[imp], cfd[imp]; col = C[imp]; cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); lit = imp == "96mm"
        ctau = loglog(s["rpm"], cr, ct); hi = s["re"] >= 4e4; ratio = ctau / s["tau"]; st = dict(r=np.mean(ratio[hi]), rt=rmse(ctau[hi], s["tau"][hi]), rn=rmse(n_po(ctau, s["rpm"], D[imp])[hi], s["npo"][hi]))
        fig, ax = plt.subplots(2, 2, figsize=(7.4, 6.0)); A, B, Cx, Dx = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        # (a) everything, zoomed out
        if lit: A.plot(BROWN[:, 0], BROWN[:, 1], "--s", color=C["lit"], lw=1.1, ms=3.2, mfc="white", label="Literature (Brown, curve 10)", zorder=2)
        A.plot(cN, cP, "-o", color=C["cfd"], mfc="white", mew=0.9, ms=3.8, lw=0.8, label="CFD (all simulated speeds)", zorder=3)
        A.plot(s["re"], s["npo"], MK[imp], color=col, mec="white", mew=0.5, ms=5.5, label=f"MIXR-1 ({imp[:-2]} mm, best run)", zorder=4)
        A.set_xscale("log"); A.set_yscale("log"); A.set_xlim(0.5, 3e6); A.set_ylim(0.2, 3000); A.set_xlabel(r"$N_{Re}$"); A.set_ylabel(r"$N_{Po}$"); A.set_title("(a) Complete range"); A.legend(frameon=False, loc="upper right", fontsize=7.3); A.xaxis.set_major_formatter(FuncFormatter(sci))
        A.axvspan(4e3, 2e5, color=col, alpha=0.07, lw=0)
        # (b) zoom
        if lit: m = (BROWN[:, 0] >= 3e3); B.plot(BROWN[m, 0], BROWN[m, 1], "--s", color=C["lit"], lw=1.1, ms=3.2, mfc="white", label="Literature", zorder=2)
        m = (cN >= 3e3) & (cN <= 2e5); B.plot(cN[m], cP[m], "-o", color=C["cfd"], mfc="white", mew=0.9, ms=4.2, lw=0.8, label="CFD", zorder=3)
        B.errorbar(s["re"], s["npo"], yerr=n_po(s["sd"], s["rpm"], D[imp]), fmt=MK[imp], color=col, mec="white", mew=0.5, ms=5.5, elinewidth=0.7, capsize=1.5, label="MIXR-1", zorder=4)
        B.set_xscale("log"); B.set_xlim(3e3, 2e5); B.set_ylim(0, max(3.2, 1.15 * s["npo"].max())); B.set_xlabel(r"$N_{Re}$"); B.set_ylabel(r"$N_{Po}$"); B.set_title("(b) Turbulent range, linear $N_{Po}$"); B.legend(frameon=False, loc="lower right", fontsize=7.3); B.xaxis.set_major_formatter(FuncFormatter(sci))
        # (c) torque
        mm = (cr >= s["rpm"].min() * 0.5) & (cr <= s["rpm"].max() * 1.4)
        Cx.plot(cr[mm], ct[mm], "-o", color=C["cfd"], mfc="white", mew=0.9, ms=4.2, lw=0.8, label="CFD", zorder=3)
        Cx.errorbar(s["rpm"], s["tau"], yerr=s["sd"], fmt=MK[imp], color=col, mec="white", mew=0.5, ms=5.5, elinewidth=0.7, capsize=1.5, label="MIXR-1", zorder=4)
        t_ = [50, 100, 200, 500, 1000, 2000] if lit else [50, 100, 200, 300, 400]; Cx.set_xscale("log"); Cx.set_yscale("log"); Cx.set_xticks(t_); Cx.set_xticklabels([str(x) for x in t_]); Cx.minorticks_off()
        Cx.set_xlabel("Speed (rpm)"); Cx.set_ylabel(r"Torque (N$\cdot$m)"); Cx.set_title("(c) Torque"); Cx.legend(frameon=False, loc="upper left", fontsize=7.3)
        # (d) ratio
        Dx.axhspan(0.9, 1.1, color="#EEEEEE", lw=0); Dx.axhline(1, color="k", lw=0.8); Dx.plot(s["rpm"][~hi], ratio[~hi], MK[imp], color=col, mfc="white", ms=5.5, label=r"$N_{Re}<4\times10^{4}$"); Dx.plot(s["rpm"][hi], ratio[hi], MK[imp], color=col, mec="white", mew=0.5, ms=5.5, label=r"$N_{Re}\geq4\times10^{4}$")
        Dx.axhline(st["r"], color=col, ls=":", lw=1); Dx.text(0.97, 0.06, f"mean {st['r']:.2f}\nRMSE$_{{\\tau}}$ {st['rt']:.3f} N$\\cdot$m\nRMSE$_{{N_{{Po}}}}$ {st['rn']:.2f}", transform=Dx.transAxes, ha="right", va="bottom", fontsize=7.6)
        Dx.set_xscale("log"); Dx.set_xticks(t_); Dx.set_xticklabels([str(x) for x in t_]); Dx.minorticks_off(); Dx.set_ylim(0, 1.8); Dx.set_xlabel("Speed (rpm)"); Dx.set_ylabel(r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}}$"); Dx.set_title("(d) CFD / experiment"); Dx.legend(frameon=False, loc="upper right", fontsize=7.3)
        fig.tight_layout(); fig.savefig(os.path.join(a.out, f"fig_cfd_{imp}.pdf")); fig.savefig(os.path.join(a.out, f"fig_cfd_{imp}.png"), dpi=170); plt.close(fig)

    # literature table: the 96 mm configuration only
    lit = lambda x: loglog(x, BROWN[:, 0], BROWN[:, 1]); rows = []
    s = S["96mm"]; cr, ct = cfd["96mm"]; cN, cP = n_re(cr, D["96mm"]), n_po(ct, cr, D["96mm"]); m = s["re"] >= 4e4; rl = lit(s["re"][m])
    rows.append(["MIXR-1 96 mm", int(m.sum()), s["npo"][m].mean(), np.nanmean(rl), rmse(s["npo"][m], rl), 100 * rmse(s["npo"][m], rl) / np.nanmean(rl), 100 * np.nanmean((s["npo"][m] - rl) / rl)])
    mm = (cN >= 4e4) & (cN <= s["re"].max() * 1.1); rl = lit(cN[mm]); rows.append(["CFD 96 mm", int(mm.sum()), cP[mm].mean(), np.nanmean(rl), rmse(cP[mm], rl), 100 * rmse(cP[mm], rl) / np.nanmean(rl), 100 * np.nanmean((cP[mm] - rl) / rl)])
    for name, arr in ARM.items():
        mk = arr[:, 0] >= 4000; x, y = arr[mk, 0], arr[mk, 1]; rl = lit(x); rows.append([f"Armfield: {name.lower()}", len(x), y.mean(), np.nanmean(rl), rmse(y, rl), 100 * rmse(y, rl) / np.nanmean(rl), 100 * np.nanmean((y - rl) / rl)])
    head = ["Series", "$n$", r"$\overline{N_{Po}}$", r"$\overline{N_{Po}}$ literature", r"RMSE $N_{Po}$", r"NRMSE (\%)", r"Bias (\%)"]; fmt = ["{}", "{:d}", "{:.2f}", "{:.2f}", "{:.2f}", "{:.0f}", "{:+.0f}"]
    L = [r"\begin{tabular}{@{}l" + "r" * 6 + "@{}}", r"\toprule", " & ".join(head) + r" \\", r"\midrule"] + [" & ".join(c if isinstance(c, str) else f.format(c) for c, f in zip(r, fmt)) + r" \\" for r in rows] + [r"\bottomrule", r"\end{tabular}"]
    open(os.path.join(a.out, "best_vs_literature_96.tex"), "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n"); print(BEST); [print(r) for r in rows]


if __name__ == "__main__":
    main()

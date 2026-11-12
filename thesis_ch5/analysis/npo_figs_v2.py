#!/usr/bin/env python3
"""
npo_figs_v2.py - publication-style figures: best run per impeller against the CFD, the literature curve and the Armfield data.

    python npo_figs_v2.py --data "<Research SeBoCa Test folder>" --out ../figures_v2 [--best174 20261106_211730] [--best96 ...]

Drop new mixr1_<imp>_<time>_sequence_summary.csv files in the data folder and run again: every complete run is ranked, the best
one is used (or the one you force with --best96 / --best174). Needs numpy, pandas, matplotlib.

Ranking rule (objective, printed in run_ranking.csv):
  1. the run is complete (all steps of the dashboard sequence reached, torque never negative);
  2. the torque never falls while the speed rises by more than 3 % (no collapse / power limiting);
  3. among those, the lowest coefficient of variation of N_Po over the plateau (N_Re >= 5e4) wins.
"""
import argparse, glob, os, re
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from npo_rmse import D, FLOOR, BROWN, ARM, load_cfd, n_re, n_po, loglog, rmse

NSTEPS = {"96mm": 28, "174mm": 23}
C = {"96mm": "#0072B2", "174mm": "#D55E00", "cfd": "#222222", "lit": "#7A7A7A",
     "Stock paddle": "#CC79A7", "Custom paddle": "#009E73", "Custom blades": "#E69F00"}
MK = {"96mm": "o", "174mm": "s", "Stock paddle": "^", "Custom paddle": "v", "Custom blades": "D"}


def style():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix",
                         "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5, "legend.fontsize": 7.8, "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
                         "axes.grid": True, "grid.color": "#DDDDDD", "grid.linewidth": 0.5, "axes.axisbelow": True, "axes.linewidth": 0.7,
                         "axes.spines.top": False, "axes.spines.right": False, "xtick.direction": "out", "ytick.direction": "out",
                         "savefig.dpi": 300, "pdf.fonttype": 42})


def sci(x, _):
    if x <= 0: return "0"
    e = int(np.floor(np.log10(x))); m = x / 10 ** e
    return rf"$10^{{{e}}}$" if abs(m - 1) < 1e-9 else rf"${m:g}\times10^{{{e}}}$"


def rpm_axis(ax, imp):
    t = [50, 100, 200, 500, 1000, 2000] if imp == "96mm" else [50, 100, 200, 300, 400]
    ax.set_xticks(t); ax.set_xticklabels([str(x) for x in t]); ax.minorticks_off()


# runs that are not used, by impeller and date (the 96 mm runs of 5 and 6 November are excluded by decision of the authors)
EXCLUDE_DATES = {"96mm": ("20261105", "20261106"), "174mm": ()}


def load_all(folder, imp):
    out = {}
    for f in sorted(glob.glob(os.path.join(folder, f"mixr1_{imp}_*_sequence_summary.csv"))):
        ts = re.search(r"_(\d{8}_\d{6})_sequence", f).group(1); d = pd.read_csv(f)
        if ts[:8] in EXCLUDE_DATES.get(imp, ()): continue
        out[ts] = d
    return out


def rank_runs(runs, imp):
    rows = []
    for ts, d in runs.items():
        d = d.copy()
        if d.torque_Nm_mean.iloc[-1] < 0: d = d.iloc[:-1]                      # step in which the motor was stopped
        complete = len(d) >= NSTEPS[imp] - (1 if ts in ("20261107_181500",) else 0)
        tau, rpm = d.torque_Nm_mean.to_numpy(), d.rpm_filtered_mean.to_numpy()
        hi = d.N_Re.to_numpy() >= 5e4
        collapse = int(sum(1 for i in range(1, len(d)) if hi[i] and tau[i] < 0.97 * tau[i - 1] and rpm[i] > 1.03 * rpm[i - 1]))
        npo = n_po(tau, rpm, D[imp])[hi]; cv = float(np.std(npo, ddof=1) / np.mean(npo)) if hi.sum() > 2 else np.nan
        rows.append(dict(run=ts, steps=len(d), complete=complete, torque_collapses=collapse, plateau_Npo=float(np.mean(npo)) if hi.any() else np.nan, plateau_CV_pct=100 * cv,
                         eligible=bool(complete and collapse == 0 and np.all(tau >= -1e-9))))
    R = pd.DataFrame(rows).sort_values(["eligible", "plateau_CV_pct"], ascending=[False, True]).reset_index(drop=True)
    return R


def series(d, imp):
    d = d.copy()
    if d.torque_Nm_mean.iloc[-1] < 0: d = d.iloc[:-1]
    rpm, tau, sd = d.rpm_filtered_mean.to_numpy(), d.torque_Nm_mean.to_numpy(), d.torque_Nm_std.to_numpy()
    ok = tau > FLOOR
    return dict(rpm=rpm[ok], tau=tau[ok], sd=sd[ok], re=n_re(rpm[ok], D[imp]), npo=n_po(tau[ok], rpm[ok], D[imp]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--best96"); ap.add_argument("--best174"); a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); style()
    cfd = load_cfd(os.path.join(a.data, "mixr_dashboard.py"))
    RUN, BEST, RANK = {}, {}, {}
    for imp in D:
        RUN[imp] = load_all(a.data, imp); RANK[imp] = rank_runs({k: v for k, v in RUN[imp].items() if len(v) >= NSTEPS[imp] - 1}, imp)
        RANK[imp].insert(0, "impeller", imp)
        forced = getattr(a, "best" + imp[:-2]); BEST[imp] = forced or RANK[imp][RANK[imp].eligible].run.iloc[0]
    pd.concat(RANK.values()).round(3).to_csv(os.path.join(a.out, "run_ranking.csv"), index=False)
    print(pd.concat(RANK.values()).round(3).to_string()); print("BEST:", BEST)

    S = {imp: series(RUN[imp][BEST[imp]], imp) for imp in D}
    stats = []
    for imp in D:
        s, (cr, ct) = S[imp], cfd[imp]; cf_t = loglog(s["rpm"], cr, ct); m = s["re"] >= 4e4
        npo_c = n_po(cf_t, s["rpm"], D[imp]); lit = loglog(s["re"], BROWN[:, 0], BROWN[:, 1])
        stats.append(dict(impeller=imp, run=BEST[imp], n=int(m.sum()), tau_rmse=rmse(cf_t[m], s["tau"][m]), tau_nrmse_pct=100 * rmse(cf_t[m], s["tau"][m]) / s["tau"][m].mean(),
                          ratio=float(np.mean(cf_t[m] / s["tau"][m])), npo_exp=s["npo"][m].mean(), npo_cfd=np.nanmean(npo_c[m]), npo_rmse=rmse(npo_c[m], s["npo"][m]),
                          npo_nrmse_pct=100 * rmse(npo_c[m], s["npo"][m]) / s["npo"][m].mean(), lit_rmse=rmse(s["npo"][m], lit[m]), lit_bias_pct=100 * np.nanmean((s["npo"][m] - lit[m]) / lit[m])))
    pd.DataFrame(stats).round(4).to_csv(os.path.join(a.out, "best_run_vs_cfd.csv"), index=False); print(pd.DataFrame(stats).round(3).to_string())

    # ---------------- Figure per impeller: N_Po, torque, ratio ----------------
    for imp in D:
        s, (cr, ct) = S[imp], cfd[imp]; col = C[imp]; st = [x for x in stats if x["impeller"] == imp][0]
        cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); rmax = s["rpm"].max() * 1.25
        fig, ax = plt.subplots(1, 3, figsize=(7.4, 2.75))
        # (a) N_Po vs N_Re
        A = ax[0]; mm = (cN >= 3e3) & (cN <= s["re"].max() * 1.6)
        A.plot(cN[mm], cP[mm], "-o", color=C["cfd"], mfc="white", mew=1.0, ms=4.2, lw=0.9, label="CFD", zorder=3)
        A.errorbar(s["re"], s["npo"], yerr=n_po(s["sd"], s["rpm"], D[imp]), fmt=MK[imp], color=col, mec="white", mew=0.5, ms=5, elinewidth=0.7, capsize=1.5, label=f"MIXR-1 ({imp[:-2]} mm)", zorder=4)
        A.axhline(np.mean(BROWN[BROWN[:, 0] > 1e3, 1]), color=C["lit"], ls="--", lw=0.9, label="Literature")
        A.set_xscale("log"); A.set_xlim(3e3, 2e5); A.set_ylim(0, max(3.2, 1.15 * s["npo"].max())); A.set_xlabel(r"$N_{Re}$"); A.set_ylabel(r"$N_{Po}$"); A.set_title("(a) Power number"); A.legend(loc="lower left", frameon=False)
        A.xaxis.set_major_formatter(FuncFormatter(sci))
        # (b) torque vs rpm
        B = ax[1]; mm = (cr >= s["rpm"].min() * 0.6) & (cr <= rmax)
        B.plot(cr[mm], ct[mm], "-o", color=C["cfd"], mfc="white", mew=1.0, ms=4.2, lw=0.9, label="CFD", zorder=3)
        B.errorbar(s["rpm"], s["tau"], yerr=s["sd"], fmt=MK[imp], color=col, mec="white", mew=0.5, ms=5, elinewidth=0.7, capsize=1.5, label="MIXR-1", zorder=4)
        B.set_xscale("log"); B.set_yscale("log"); rpm_axis(B, imp); B.set_xlabel("Speed (rpm)"); B.set_ylabel(r"Torque (N$\cdot$m)"); B.set_title("(b) Torque"); B.legend(loc="upper left", frameon=False)
        # (c) ratio
        Cx = ax[2]; r = loglog(s["rpm"], cr, ct) / s["tau"]; hi = s["re"] >= 4e4
        Cx.axhspan(0.9, 1.1, color="#EEEEEE", lw=0); Cx.axhline(1, color="k", lw=0.8)
        Cx.plot(s["rpm"][~hi], r[~hi], MK[imp], color=col, mfc="white", ms=5, label=r"$N_{Re}<4\times10^{4}$"); Cx.plot(s["rpm"][hi], r[hi], MK[imp], color=col, mec="white", mew=0.5, ms=5, label=r"$N_{Re}\geq4\times10^{4}$")
        Cx.axhline(st["ratio"], color=col, ls=":", lw=1); Cx.text(0.97, 0.05, f"mean {st['ratio']:.2f}\nRMSE$_{{\\tau}}$ {st['tau_rmse']:.3f} N$\\cdot$m\nRMSE$_{{N_{{Po}}}}$ {st['npo_rmse']:.2f}", transform=Cx.transAxes, ha="right", va="bottom", fontsize=7.6)
        Cx.set_xscale("log"); rpm_axis(Cx, imp); Cx.set_ylim(0, 1.8); Cx.set_xlabel("Speed (rpm)"); Cx.set_ylabel(r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}}$"); Cx.set_title("(c) CFD / experiment"); Cx.legend(loc="upper right", frameon=False, ncol=1)
        fig.tight_layout(w_pad=1.2); fig.savefig(os.path.join(a.out, f"fig_cfd_{imp}.pdf")); fig.savefig(os.path.join(a.out, f"fig_cfd_{imp}.png"), dpi=200); plt.close(fig)

    # ---------------- run-to-run figure ----------------
    fig, ax = plt.subplots(1, 2, figsize=(7.4, 2.9))
    for A, imp in zip(ax, D):
        for ts, d in RUN[imp].items():
            if len(d) < NSTEPS[imp] - 1: continue
            x = series(d, imp); A.plot(x["re"], x["npo"], "-", color="#BBBBBB", lw=0.8, zorder=1)
        s = S[imp]; A.plot(s["re"], s["npo"], "-" + MK[imp], color=C[imp], mec="white", mew=0.5, ms=5, lw=1.2, label=f"best run ({BEST[imp][4:6]}/{BEST[imp][6:8]}, {BEST[imp][9:11]}:{BEST[imp][11:13]})", zorder=3)
        A.plot([], [], color="#BBBBBB", label="other complete runs"); A.set_xscale("log"); A.set_xlim(8e3, 2e5); A.set_ylim(0, 3.2); A.set_xlabel(r"$N_{Re}$"); A.set_ylabel(r"$N_{Po}$"); A.set_title(f"{imp[:-2]} mm impeller"); A.legend(frameon=False, loc="lower right")
        A.xaxis.set_major_formatter(FuncFormatter(sci))
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_runs.pdf")); fig.savefig(os.path.join(a.out, "fig_runs.png"), dpi=200); plt.close(fig)

    # ---------------- literature comparison ----------------
    fig, ax = plt.subplots(figsize=(4.6, 3.5))
    ax.plot(BROWN[:, 0], BROWN[:, 1], "--", color=C["lit"], lw=1.3, label="Literature (Brown, curve 10)")
    for imp in D:
        cr, ct = cfd[imp]; N, P = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); m = (N >= 3e3) & (N <= 1.3e6)
        ax.plot(N[m], P[m], "-o", color=C[imp], mfc="white", mew=0.9, ms=3.6, lw=0.8, alpha=0.9, label=f"CFD {imp[:-2]} mm")
        ax.plot(S[imp]["re"], S[imp]["npo"], MK[imp], color=C[imp], mec="white", mew=0.5, ms=5.5, label=f"MIXR-1 {imp[:-2]} mm")
    ax.set_xscale("log"); ax.set_xlim(3e3, 1.5e6); ax.set_ylim(0, 4); ax.set_xlabel(r"$N_{Re}$"); ax.set_ylabel(r"$N_{Po}$"); ax.xaxis.set_major_formatter(FuncFormatter(sci)); ax.legend(frameon=False, loc="upper right")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_literature.pdf")); fig.savefig(os.path.join(a.out, "fig_literature.png"), dpi=200); plt.close(fig)

    # ---------------- Armfield comparison ----------------
    fig, ax = plt.subplots(1, 2, figsize=(7.4, 3.2), sharey=True)
    for A, imp in zip(ax, D):
        cr, ct = cfd[imp]; N, P = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); m = (N >= 3e3) & (N <= 3e5)
        A.plot(BROWN[:, 0], BROWN[:, 1], "--", color=C["lit"], lw=1.1, label="Literature")
        A.plot(N[m], P[m], "-o", color=C["cfd"], mfc="white", mew=0.9, ms=3.6, lw=0.8, label=f"CFD {imp[:-2]} mm")
        for name, arr in ARM.items(): A.plot(arr[:, 0], arr[:, 1], "-" + MK[name], color=C[name], mec="white", mew=0.4, ms=5, lw=1.0, label=f"Armfield: {name.lower()}")
        A.plot(S[imp]["re"], S[imp]["npo"], MK[imp], color=C[imp], mec="white", mew=0.6, ms=6, label=f"MIXR-1 {imp[:-2]} mm (best run)", zorder=6)
        A.set_xscale("log"); A.set_yscale("log"); A.set_xlim(3e3, 3e5); A.set_ylim(0.1, 150); A.set_xlabel(r"$N_{Re}$"); A.set_title(f"CFD {imp[:-2]} mm against Armfield data"); A.xaxis.set_major_formatter(FuncFormatter(sci))
    ax[0].set_ylabel(r"$N_{Po}$"); ax[0].legend(frameon=False, loc="upper right", fontsize=7); ax[1].legend(frameon=False, loc="upper right", fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_armfield.pdf")); fig.savefig(os.path.join(a.out, "fig_armfield.png"), dpi=200); plt.close(fig)


if __name__ == "__main__":
    main()

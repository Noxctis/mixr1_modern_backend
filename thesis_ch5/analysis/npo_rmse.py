#!/usr/bin/env python3
"""
npo_rmse.py - torque / power-number comparison for Chapter 5 (MIXR-1 vs CFD vs Armfield student data vs literature).

Produces, in --out:
    rmse_cfd_vs_mixr.csv / .tex        CFD against MIXR-1 (per impeller, torque and N_Po)
    rmse_cfd_vs_armfield.csv / .tex    CFD against the Armfield student data (stock paddle, custom paddle, custom blades)
    rmse_vs_literature.csv / .tex      every series against the literature curve (Brown, "Reference (curve 10)")
    runs_used.csv, per_run_rmse.csv, step_table_<imp>.csv
    fig_npo_all.pdf, fig_torque_cfd.pdf, fig_rmse_bars.pdf, fig_ratio.pdf

Run:  python npo_rmse.py --data "<folder with mixr1_*_sequence_summary.csv and mixr_dashboard.py>" --out ../figures
Needs numpy, pandas, matplotlib.
EDIT the RUNS dict below to change which runs are used.
"""
import argparse, ast, glob, os, re
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import LogFormatterMathtext

RHO, MU = 996.3, 8.54e-4
D = {"96mm": 0.0665, "174mm": 0.1207}          # the diameters of mixr_dashboard.py (used for every N_Re, N_Po below)
FLOOR = 0.002                                   # N.m, about two sensor counts: below it the torque is noise
RE_MIN = 4e4                                    # turbulent comparison range of the earlier analysis; CFD itself is trusted from N_Re 4000

# Runs used (timestamp inside the file name). Reason for each choice is in the chapter text / README.
RUNS = {
    "96mm":  ["20261107_180055", "20261107_181500"],
    "174mm": ["20261106_193106", "20261106_201732", "20261106_205359", "20261106_211730", "20261106_212551", "20261106_215412"],
}
DROP_LAST_STEP = {"20261107_181500"}            # step 28: motor stopped, torque negative

# ---- literature and Armfield student data, transcribed from the spreadsheet screenshot (check against the spreadsheet) ----
BROWN = np.array([
    (2.875822, 12.05009), (7.813283, 4.520778), (11.00723, 3.425236), (12.96641, 3.048011), (16.2104, 2.63477),
    (19.66839, 2.380064), (27.27232, 2.120899), (35.11131, 1.91684), (50.14572, 1.733952), (69.49935, 1.663481),
    (99.24903, 1.52714), (116.8255, 1.52927), (148.1218, 1.487801), (196.3416, 1.448006), (399.9524, 1.456838),
    (1002.22, 1.557622), (1981.722, 1.590022), (3976.283, 1.671932), (9966.799, 1.710172), (20000.08, 1.771926),
    (29406.66, 1.751726), (986535.7, 1.80512)])
ARM = {
    "Stock paddle": np.array([(3912.386, 105.1577), (15035.14, 8.646692), (23377.21, 12.06864), (33544.83, 7.81904)]),
    "Custom paddle": np.array([(3912.386, 70.1051), (18371.97, 1.591019), (27270.18, 1.304222), (37517.24, 0.679748),
                               (50317.24, 0.558591), (64441.38, 0.398095), (81213.79, 0.396585), (83862.07, 0.439557), (84303.45, 0.467199)]),
    "Custom blades": np.array([(7944.828, 7.953256), (16147.42, 2.143896), (24489.49, 0.836106), (36634.48, 0.356501),
                               (48551.72, 0.305383), (64882.76, 0.225177), (76800, 0.20274), (79889.66, 0.224167), (80331.03, 0.222314)]),
}
COL = {"96mm": "#0072B2", "174mm": "#D55E00", "Stock paddle": "#CC79A7", "Custom paddle": "#009E73", "Custom blades": "#E69F00", "lit": "#222222"}
MK = {"96mm": "o", "174mm": "s", "Stock paddle": "^", "Custom paddle": "v", "Custom blades": "D"}


def n_re(rpm, d): return RHO * (np.asarray(rpm, float) / 60) * d ** 2 / MU
def n_po(tau, rpm, d): return 2 * np.pi * np.asarray(tau, float) / (RHO * (np.asarray(rpm, float) / 60) ** 2 * d ** 5)
def loglog(x, xp, fp):
    """log-log interpolation of fp(xp) at x (nan outside the range of xp)."""
    xp, fp = np.asarray(xp, float), np.asarray(fp, float); o = np.argsort(xp); xp, fp = xp[o], fp[o]
    ok = (fp > 0) & (xp > 0); xp, fp = xp[ok], fp[ok]; x = np.asarray(x, float)
    out = np.exp(np.interp(np.log(x), np.log(xp), np.log(fp))); out[(x < xp[0]) | (x > xp[-1])] = np.nan
    return out
def rmse(a, b): a, b = np.asarray(a, float), np.asarray(b, float); m = np.isfinite(a) & np.isfinite(b); return float(np.sqrt(np.mean((a[m] - b[m]) ** 2))) if m.any() else np.nan


def load_cfd(dash):
    """CFD_REFERENCE of mixr_dashboard.py, read without importing it (it needs Qt)."""
    tree = ast.parse(open(dash, encoding="utf-8").read())
    for node in tree.body:
        tgt = node.target if isinstance(node, ast.AnnAssign) else (node.targets[0] if isinstance(node, ast.Assign) else None)
        if getattr(tgt, "id", None) == "CFD_REFERENCE":
            ref = ast.literal_eval(node.value); return {k: (np.array(v["rpm"], float), np.array(v["torque"], float)) for k, v in ref.items()}
    raise SystemExit("CFD_REFERENCE not found")


def load_runs(folder, imp):
    S = {}
    for ts in RUNS[imp]:
        f = glob.glob(os.path.join(folder, f"mixr1_{imp}_{ts}_sequence_summary.csv"))[0]
        d = pd.read_csv(f)
        if ts in DROP_LAST_STEP: d = d.iloc[:-1]
        S[ts] = d.set_index("target_rpm")[["rpm_filtered_mean", "torque_Nm_mean"]]
    return S


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True); a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cfd = load_cfd(os.path.join(a.data, "mixr_dashboard.py"))
    ex, per_run, step_rows, runs_used = {}, [], [], []
    for imp in D:
        R = load_runs(a.data, imp); cr, ct = cfd[imp]
        P = pd.concat({k: v.torque_Nm_mean for k, v in R.items()}, axis=1); Q = pd.concat({k: v.rpm_filtered_mean for k, v in R.items()}, axis=1)
        rpm, tau, tsd = Q.mean(axis=1).to_numpy(), P.mean(axis=1).to_numpy(), P.std(axis=1, ddof=1).to_numpy()
        ex[imp] = dict(rpm=rpm, tau=tau, sd=tsd, P=P, Q=Q, runs=R)
        for ts, v in R.items(): runs_used.append(dict(impeller=imp, run=ts, steps=len(v), rpm_max=v.rpm_filtered_mean.max()))
        cf_t = loglog(rpm, cr, ct)
        step_rows.append(pd.DataFrame(dict(impeller=imp, target_rpm=P.index, rpm=rpm, N_Re=n_re(rpm, D[imp]), torque_exp=tau, torque_sd=tsd, torque_cfd=cf_t,
                                           ratio_cfd_exp=cf_t / np.where(tau > FLOOR, tau, np.nan), Npo_exp=n_po(tau, rpm, D[imp]), Npo_cfd=n_po(cf_t, rpm, D[imp]))))
        step_rows[-1].round(6).to_csv(os.path.join(a.out, f"step_table_{imp}.csv"), index=False)
    pd.DataFrame(runs_used).to_csv(os.path.join(a.out, "runs_used.csv"), index=False)
    ST = pd.concat(step_rows)

    # ---------------------------------------------------------------- 1. CFD vs MIXR-1
    rows = []
    for imp in D:
        s = ST[ST.impeller == imp]
        for label, m in (("N_Re >= 4e4", (s.torque_exp > FLOOR) & (s.N_Re >= RE_MIN)), ("all above sensor floor", (s.torque_exp > FLOOR))):
            t = s[m]
            if t.empty: continue
            r = dict(impeller=imp, range=label, n=len(t), Re_lo=t.N_Re.min(), Re_hi=t.N_Re.max(),
                     tau_rmse=rmse(t.torque_cfd, t.torque_exp), tau_nrmse_pct=100 * rmse(t.torque_cfd, t.torque_exp) / t.torque_exp.mean(),
                     tau_ratio=(t.torque_cfd / t.torque_exp).mean(), Npo_exp=t.Npo_exp.mean(), Npo_cfd=t.Npo_cfd.mean(),
                     Npo_rmse=rmse(t.Npo_cfd, t.Npo_exp), Npo_nrmse_pct=100 * rmse(t.Npo_cfd, t.Npo_exp) / t.Npo_exp.mean())
            # run-to-run spread: RMSE of each individual run
            rr = []
            for ts, v in ex[imp]["runs"].items():
                vv = v[(v.torque_Nm_mean > FLOOR) & (n_re(v.rpm_filtered_mean, D[imp]) >= (RE_MIN if label.startswith("N_Re") else 0))]
                cft = loglog(vv.rpm_filtered_mean, *cfd[imp]); rr.append(rmse(n_po(cft, vv.rpm_filtered_mean, D[imp]), n_po(vv.torque_Nm_mean, vv.rpm_filtered_mean, D[imp])))
                per_run.append(dict(impeller=imp, range=label, run=ts, Npo_rmse=rr[-1], n=len(vv)))
            r["Npo_rmse_run_min"], r["Npo_rmse_run_max"] = min(rr), max(rr); rows.append(r)
    T1 = pd.DataFrame(rows); T1.round(4).to_csv(os.path.join(a.out, "rmse_cfd_vs_mixr.csv"), index=False)
    pd.DataFrame(per_run).round(4).to_csv(os.path.join(a.out, "per_run_rmse.csv"), index=False)

    # ---------------------------------------------------------------- 2. CFD vs Armfield
    rows = []
    for name, arr in ARM.items():
        for imp in D:
            cr, ct = cfd[imp]; cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); m = arr[:, 0] >= 4000
            x, y = arr[m, 0], arr[m, 1]; yc = loglog(x, cN, cP)
            rows.append(dict(series=name, cfd=imp, n=int(np.isfinite(yc).sum()), Re_lo=x.min(), Re_hi=x.max(), Npo_arm=np.nanmean(y), Npo_cfd=np.nanmean(yc),
                             Npo_rmse=rmse(yc, y), Npo_nrmse_pct=100 * rmse(yc, y) / np.nanmean(y), log10_rmse=rmse(np.log10(yc), np.log10(y)),
                             Npo_rmse_Re_ge_2e4=rmse(yc[x >= 2e4], y[x >= 2e4])))
    T2 = pd.DataFrame(rows); T2.round(4).to_csv(os.path.join(a.out, "rmse_cfd_vs_armfield.csv"), index=False)

    # ---------------------------------------------------------------- 3. everything against the literature curve
    def lit(x): return loglog(x, BROWN[:, 0], BROWN[:, 1])
    rows = []
    for imp in D:
        s = ST[(ST.impeller == imp) & (ST.torque_exp > FLOOR) & (ST.N_Re >= RE_MIN)]; cr, ct = cfd[imp]; cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp])
        mm = (cN >= RE_MIN) & (cN <= s.N_Re.max() * 1.1)
        rows.append(dict(series=f"MIXR-1 {imp}", n=len(s), Re_lo=s.N_Re.min(), Re_hi=s.N_Re.max(), Npo_mean=s.Npo_exp.mean(), lit_mean=np.nanmean(lit(s.N_Re)),
                         Npo_rmse=rmse(s.Npo_exp, lit(s.N_Re)), nrmse_pct=100 * rmse(s.Npo_exp, lit(s.N_Re)) / np.nanmean(lit(s.N_Re)), bias_pct=100 * np.nanmean((s.Npo_exp - lit(s.N_Re)) / lit(s.N_Re))))
        rows.append(dict(series=f"CFD {imp}", n=int(mm.sum()), Re_lo=cN[mm].min(), Re_hi=cN[mm].max(), Npo_mean=cP[mm].mean(), lit_mean=np.nanmean(lit(cN[mm])),
                         Npo_rmse=rmse(cP[mm], lit(cN[mm])), nrmse_pct=100 * rmse(cP[mm], lit(cN[mm])) / np.nanmean(lit(cN[mm])), bias_pct=100 * np.nanmean((cP[mm] - lit(cN[mm])) / lit(cN[mm]))))
    for name, arr in ARM.items():
        m = arr[:, 0] >= 4000; x, y = arr[m, 0], arr[m, 1]
        rows.append(dict(series=f"Armfield: {name}", n=len(x), Re_lo=x.min(), Re_hi=x.max(), Npo_mean=y.mean(), lit_mean=np.nanmean(lit(x)), Npo_rmse=rmse(y, lit(x)),
                         nrmse_pct=100 * rmse(y, lit(x)) / np.nanmean(lit(x)), bias_pct=100 * np.nanmean((y - lit(x)) / lit(x))))
    T3 = pd.DataFrame(rows); T3.round(4).to_csv(os.path.join(a.out, "rmse_vs_literature.csv"), index=False)

    # ---------------------------------------------------------------- figures
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix",
                         "font.size": 10, "axes.grid": True, "grid.alpha": 0.25, "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False})
    XL, YL = r"Reynolds number, $N_{Re}$", r"Power number, $N_{Po}$"
    def style_log(ax): ax.set_xscale("log"); ax.set_yscale("log"); ax.xaxis.set_major_formatter(LogFormatterMathtext()); ax.yaxis.set_major_formatter(LogFormatterMathtext())

    fig, (A, B) = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax in (A, B):
        ax.plot(BROWN[:, 0], BROWN[:, 1], "--", color=COL["lit"], lw=1.6, label="Literature (Brown, curve 10)", zorder=2)
        for name, arr in ARM.items(): ax.plot(arr[:, 0], arr[:, 1], "-" + MK[name], color=COL[name], lw=1.3, ms=4.5, label=f"Armfield: {name}", zorder=3)
        for imp in D:
            cr, ct = cfd[imp]; ax.plot(n_re(cr, D[imp]), n_po(ct, cr, D[imp]), "-", color=COL[imp], lw=1.2, alpha=0.55, label=f"CFD {imp}", zorder=2)
            s = ST[(ST.impeller == imp) & (ST.torque_exp > FLOOR)]
            ax.fill_between(s.N_Re, n_po(np.clip(s.torque_exp - s.torque_sd, 1e-6, None), s.rpm, D[imp]), n_po(s.torque_exp + s.torque_sd, s.rpm, D[imp]), color=COL[imp], alpha=0.15, lw=0)
            ax.plot(s.N_Re, s.Npo_exp, MK[imp] + "-", color=COL[imp], ms=4.5, lw=1.4, label=f"MIXR-1 {imp} (mean of {len(RUNS[imp])} runs, $\\pm$1 SD)", zorder=4)
    style_log(A); A.set_xlim(1, 2e6); A.set_ylim(0.1, 200); A.set_xlabel(XL); A.set_ylabel(YL); A.set_title("(a) Full range", fontsize=10)
    B.set_xscale("log"); B.set_xlim(4e3, 2e5); B.set_ylim(0, 4); B.set_xlabel(XL); B.set_ylabel(YL); B.set_title("(b) Turbulent range, linear $N_{Po}$", fontsize=10)
    h, l = B.get_legend_handles_labels(); fig.legend(h, l, loc="lower center", ncol=3, fontsize=8, frameon=False); fig.tight_layout(rect=[0, 0.17, 1, 1]); fig.savefig(os.path.join(a.out, "fig_npo_all.pdf")); fig.savefig(os.path.join(a.out, "fig_npo_all.png"), dpi=130); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(10, 4.1))
    for ax, imp in zip(axs, D):
        cr, ct = cfd[imp]; s = ST[ST.impeller == imp]; ok = s.torque_exp > FLOOR
        ax.plot(cr, ct, "-", color=COL[imp], alpha=0.55, lw=1.4, label="CFD")
        for ts, v in ex[imp]["runs"].items(): ax.plot(v.rpm_filtered_mean, v.torque_Nm_mean.where(v.torque_Nm_mean > FLOOR), ".", color="#888", ms=3, alpha=0.8, label="individual runs" if ts == RUNS[imp][0] else None)
        ax.errorbar(s.rpm[ok], s.torque_exp[ok], yerr=s.torque_sd[ok], fmt=MK[imp] + "-", color=COL[imp], ms=4, lw=1.2, capsize=2, label="MIXR-1 (mean $\\pm$1 SD)")
        style_log(ax); ax.set_xlim(40, 2000 if imp == "96mm" else 800); ax.set_ylim(1e-3, 1); ax.set_xlabel("Shaft speed (rpm)"); ax.set_ylabel(r"Torque (N$\cdot$m)"); ax.set_title(f"{imp} impeller", fontsize=10); ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_torque_cfd.pdf")); fig.savefig(os.path.join(a.out, "fig_torque_cfd.png"), dpi=130); plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.4, 3.8))
    for imp in D:
        s = ST[(ST.impeller == imp) & (ST.torque_exp > FLOOR)]; ax.plot(s.rpm, s.ratio_cfd_exp, MK[imp] + "-", color=COL[imp], ms=4, lw=1.2, label=imp)
    ax.axhline(1, color="k", ls=":", lw=1); ax.set_xscale("log"); ax.set_ylim(0, 1.6); ax.set_xlabel("Shaft speed (rpm)"); ax.set_ylabel("Torque ratio, CFD / experiment"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_ratio.pdf")); fig.savefig(os.path.join(a.out, "fig_ratio.png"), dpi=130); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(10, 3.9))
    t1 = T1[T1.range == "N_Re >= 4e4"]; x = np.arange(len(t1))
    axs[0].bar(x - 0.18, t1.Npo_exp, 0.36, color="#888", label="MIXR-1"); axs[0].bar(x + 0.18, t1.Npo_cfd, 0.36, color="#56B4E9", label="CFD")
    axs[0].set_xticks(x, t1.impeller); axs[0].set_ylabel(r"Mean $N_{Po}$, $N_{Re}\geq4\times10^{4}$"); axs[0].set_title(f"(a) CFD against MIXR-1 (RMSE {', '.join(f'{v:.2f}' for v in t1.Npo_rmse)})", fontsize=9); axs[0].legend(fontsize=8)
    lab = [f"{r.series}\n{r.cfd}" for r in T2.itertuples()]; axs[1].bar(range(len(T2)), T2.Npo_rmse, color=[COL[s] for s in T2.series]); axs[1].set_yscale("log")
    axs[1].set_xticks(range(len(T2)), [f"{s.split()[0][:5]}.{s.split()[1][:4]}\nCFD {c[:-2]}" for s, c in zip(T2.series, T2.cfd)], fontsize=7); axs[1].set_ylabel(r"RMSE of $N_{Po}$"); axs[1].set_title("(b) CFD against Armfield student data", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_rmse_bars.pdf")); fig.savefig(os.path.join(a.out, "fig_rmse_bars.png"), dpi=130); plt.close(fig)

    # ---------------------------------------------------------------- LaTeX tables
    def tex(df, cols, heads, fmt, name):
        L = [r"\begin{tabular}{@{}" + "l" * sum(isinstance(df[c].iloc[0], str) for c in cols) + "r" * sum(not isinstance(df[c].iloc[0], str) for c in cols) + r"@{}}", r"\toprule", " & ".join(heads) + r" \\", r"\midrule"]
        for _, r in df.iterrows(): L.append(" & ".join(r[c] if isinstance(r[c], str) else fmt.get(c, "{:.3g}").format(r[c]) for c in cols).replace("_", r"\_").replace("%", r"\%") + r" \\")
        L += [r"\bottomrule", r"\end{tabular}"]; open(os.path.join(a.out, name), "w").write("\n".join(L) + "\n")
    t = T1.copy(); t["impeller"] = t.impeller.str.replace("mm", " mm"); t["range"] = t["range"].str.replace("N_Re >= 4e4", r"Re $\geq 4\times10^{4}$")
    t["rng"] = t.apply(lambda r: f"{r.Re_lo:.2g}--{r.Re_hi:.2g}".replace("e+0", r"\times10^{").replace("e+", r"\times10^{") if False else f"{r.Re_lo/1e3:.1f}--{r.Re_hi/1e3:.0f}", axis=1)
    tex(t, ["impeller", "range", "n", "tau_rmse", "tau_nrmse_pct", "tau_ratio", "Npo_exp", "Npo_cfd", "Npo_rmse", "Npo_nrmse_pct"],
        ["Impeller", "Range", "$n$", r"RMSE $\tau$ (N\,m)", r"NRMSE $\tau$ (\%)", r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}}$", r"$\overline{N_{Po}}$ exp", r"$\overline{N_{Po}}$ CFD", r"RMSE $N_{Po}$", r"NRMSE $N_{Po}$ (\%)"],
        {"n": "{:d}", "tau_rmse": "{:.4f}", "tau_nrmse_pct": "{:.1f}", "tau_ratio": "{:.2f}", "Npo_exp": "{:.2f}", "Npo_cfd": "{:.2f}", "Npo_rmse": "{:.2f}", "Npo_nrmse_pct": "{:.1f}"}, "rmse_cfd_vs_mixr.tex")
    t = T2.copy(); t["cfd"] = t.cfd.str.replace("mm", " mm")
    tex(t, ["series", "cfd", "n", "Npo_arm", "Npo_cfd", "Npo_rmse", "Npo_nrmse_pct", "log10_rmse"], ["Armfield series", "CFD", "$n$", r"$\overline{N_{Po}}$ Armfield", r"$\overline{N_{Po}}$ CFD", r"RMSE $N_{Po}$", "NRMSE (\\%)", r"RMSE $\log_{10}N_{Po}$"],
        {"n": "{:d}", "Npo_arm": "{:.2f}", "Npo_cfd": "{:.2f}", "Npo_rmse": "{:.2f}", "Npo_nrmse_pct": "{:.0f}", "log10_rmse": "{:.2f}"}, "rmse_cfd_vs_armfield.tex")
    tex(T3, ["series", "n", "Npo_mean", "lit_mean", "Npo_rmse", "nrmse_pct", "bias_pct"], ["Series", "$n$", r"$\overline{N_{Po}}$", r"$\overline{N_{Po}}$ literature", r"RMSE $N_{Po}$", "NRMSE (\\%)", "Bias (\\%)"],
        {"n": "{:d}", "Npo_mean": "{:.2f}", "lit_mean": "{:.2f}", "Npo_rmse": "{:.2f}", "nrmse_pct": "{:.0f}", "bias_pct": "{:+.0f}"}, "rmse_vs_literature.tex")
    pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
    print(T1.round(3).to_string()); print(); print(T2.round(3).to_string()); print(); print(T3.round(3).to_string()); print(); print(pd.DataFrame(per_run).round(3).to_string())


if __name__ == "__main__":
    main()

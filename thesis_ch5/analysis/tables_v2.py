#!/usr/bin/env python3
"""LaTeX tables for the best run per impeller (same ranking as npo_figs_v2.py). python tables_v2.py --data DIR --out Figs [--best96 TS] [--best174 TS]"""
import argparse, os, numpy as np, pandas as pd
from npo_rmse import D, FLOOR, BROWN, ARM, load_cfd, n_re, n_po, loglog, rmse
from npo_figs_v2 import load_all, rank_runs, series, NSTEPS

ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True); ap.add_argument("--best96"); ap.add_argument("--best174"); a = ap.parse_args()
cfd = load_cfd(os.path.join(a.data, "mixr_dashboard.py")); rows1, rows3, runs = [], [], {}
lit = lambda x: loglog(x, BROWN[:, 0], BROWN[:, 1])
for imp in D:
    R = load_all(a.data, imp); K = rank_runs({k: v for k, v in R.items() if len(v) >= NSTEPS[imp] - 1}, imp)
    best = getattr(a, "best" + imp[:-2]) or K[K.eligible].run.iloc[0]; runs[imp] = best
    s = series(R[best], imp); cr, ct = cfd[imp]; cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp]); ctau = loglog(s["rpm"], cr, ct); cpo = n_po(ctau, s["rpm"], D[imp])
    for lab, m in ((r"Re $\geq 4\times10^{4}$", s["re"] >= 4e4), ("all above sensor floor", s["re"] > 0)):
        t = s["tau"][m]; rows1.append([f"{imp[:-2]} mm", lab, int(m.sum()), rmse(ctau[m], t), 100 * rmse(ctau[m], t) / t.mean(), np.mean(ctau[m] / t), s["npo"][m].mean(), np.nanmean(cpo[m]), rmse(cpo[m], s["npo"][m]), 100 * rmse(cpo[m], s["npo"][m]) / s["npo"][m].mean()])
    m = s["re"] >= 4e4; rl = lit(s["re"][m])
    rows3.append([f"MIXR-1 {imp[:-2]} mm", int(m.sum()), s["npo"][m].mean(), np.nanmean(rl), rmse(s["npo"][m], rl), 100 * rmse(s["npo"][m], rl) / np.nanmean(rl), 100 * np.nanmean((s["npo"][m] - rl) / rl)])
    mm = (cN >= 4e4) & (cN <= s["re"].max() * 1.1); rl = lit(cN[mm])
    rows3.append([f"CFD {imp[:-2]} mm", int(mm.sum()), cP[mm].mean(), np.nanmean(rl), rmse(cP[mm], rl), 100 * rmse(cP[mm], rl) / np.nanmean(rl), 100 * np.nanmean((cP[mm] - rl) / rl)])
for name, arr in ARM.items():
    mk = arr[:, 0] >= 4000; x, y = arr[mk, 0], arr[mk, 1]; rl = lit(x)
    rows3.append([f"Armfield: {name.lower()}", len(x), y.mean(), np.nanmean(rl), rmse(y, rl), 100 * rmse(y, rl) / np.nanmean(rl), 100 * np.nanmean((y - rl) / rl)])
def write(name, head, rows, fmt):
    L = [r"\begin{tabular}{@{}" + ("ll" + "r" * (len(head) - 2) if name.startswith("best_cfd") else "l" + "r" * (len(head) - 1)) + "@{}}", r"\toprule", " & ".join(head) + r" \\", r"\midrule"]
    for r in rows: L.append(" & ".join(c if isinstance(c, str) else f.format(c) for c, f in zip(r, fmt)) + " \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]; open(os.path.join(a.out, name), "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")
write("best_cfd_vs_mixr.tex", ["Impeller", "Range", "$n$", r"RMSE $\tau$ (N\,m)", r"NRMSE $\tau$ (\%)", r"$\tau_{\mathrm{CFD}}/\tau_{\mathrm{exp}}$", r"$\overline{N_{Po}}$ exp", r"$\overline{N_{Po}}$ CFD", r"RMSE $N_{Po}$", r"NRMSE $N_{Po}$ (\%)"], rows1, ["{}", "{}", "{:d}", "{:.4f}", "{:.1f}", "{:.2f}", "{:.2f}", "{:.2f}", "{:.2f}", "{:.1f}"])
write("best_vs_literature.tex", ["Series", "$n$", r"$\overline{N_{Po}}$", r"$\overline{N_{Po}}$ literature", r"RMSE $N_{Po}$", r"NRMSE (\%)", r"Bias (\%)"], rows3, ["{}", "{:d}", "{:.2f}", "{:.2f}", "{:.2f}", "{:.0f}", "{:+.0f}"])
pd.DataFrame(rows1).round(3).to_csv(os.path.join(a.out, "best_cfd_vs_mixr.csv"), index=False); pd.DataFrame(rows3).round(3).to_csv(os.path.join(a.out, "best_vs_literature.csv"), index=False)
print(runs); [print([round(x,3) if not isinstance(x,str) else x for x in r]) for r in rows1 + rows3]

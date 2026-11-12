#!/usr/bin/env python3
"""
trend_match.py - does the N_Re-N_Po curve of the apparatus have the SAME SHAPE as the CFD and the literature curve?
The absolute level (torque offset, geometry) and a horizontal shift (the viscosity, that is the temperature, of the water) are removed:
  level-free error : both curves divided by their own mean over the common N_Re range, then RMSE of N_Po/mean
  shift            : the factor s by which N_Re of the data must be multiplied for the best match of the level-free curves (s = mu_assumed/mu_true)
  slope            : d(N_Po)/d(ln N_Re) of each curve over the common range (per natural-log unit)
    python trend_match.py --data "<Research SeBoCa Test>" --out ../Figs
Writes trend_metrics.csv / .tex and fig_trend.pdf/.png. Uses the same runs as npo_figs_v2.py (96 mm: 7 Nov only).
"""
import argparse, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from npo_rmse import D, BROWN, load_cfd, n_re, n_po, loglog, rmse
from npo_figs_v2 import style, load_all, rank_runs, series, NSTEPS, C, MK


def curve_at(x, xs, ys): return loglog(x, xs, ys)


def slope(x, y):
    return float(np.polyfit(np.log(x), y, 1)[0])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", required=True); ap.add_argument("--out", required=True); a = ap.parse_args(); style()
    cfd = load_cfd(os.path.join(a.data, "mixr_dashboard.py")); rows = []; fig, ax = plt.subplots(1, 2, figsize=(7.4, 3.0)); LO, HI = 1.2e4, 1.2e5
    for A, imp in zip(ax, D):
        R = load_all(a.data, imp); K = rank_runs({k: v for k, v in R.items() if len(v) >= NSTEPS[imp] - 1}, imp); best = K[K.eligible].run.iloc[0]
        s = series(R[best], imp); cr, ct = cfd[imp]; cN, cP = n_re(cr, D[imp]), n_po(ct, cr, D[imp])
        m = (s["re"] >= LO) & (s["re"] <= HI); x, y = s["re"][m], s["npo"][m]
        refs = {"CFD": lambda q: curve_at(q, cN, cP)}
        if imp == "96mm": refs["literature"] = lambda q: curve_at(q, BROWN[:, 0], BROWN[:, 1])
        for lo2, (name, f) in [(l, kv) for l in (1.2e4, 4e4) for kv in refs.items()]:
            x2, y2 = x[x >= lo2], y[x >= lo2]; r = f(x2); ok = np.isfinite(r); xx, yy, rr = x2[ok], y2[ok], r[ok]
            lvl = rmse(yy, rr); yn, rn = yy / yy.mean(), rr / rr.mean(); lvl_free = rmse(yn, rn)
            best_s, best_e = 1.0, 1e9
            for sft in np.exp(np.linspace(np.log(0.7), np.log(1.4), 141)):
                q = xx * sft; r2 = f(q); k = np.isfinite(r2)
                if k.sum() < 0.8 * len(xx): continue
                e = rmse(yy[k] / yy[k].mean(), r2[k] / r2[k].mean())
                if e < best_e: best_s, best_e = sft, e
            rows.append(dict(impeller=imp, run=best, reference=name, from_Re=lo2, n=int(ok.sum()), RMSE_level=lvl, NRMSE_level_pct=100 * lvl / yy.mean(), RMSE_shape=lvl_free,
                             RMSE_shape_shifted=best_e, shift_s=best_s, slope_data=slope(xx, yn), slope_ref=slope(xx, rn)))
        # figure: curves normalised to their own mean over the common range
        for name, f, col, mk in [("CFD", refs["CFD"], C["cfd"], "-o")] + ([("Literature", refs["literature"], C["lit"], "--s")] if imp == "96mm" else []):
            q = np.geomspace(LO, HI, 40); r = f(q); A.plot(q, r / np.nanmean(r), mk, color=col, ms=3, mfc="white", lw=0.9, label=name)
        A.plot(x, y / y.mean(), MK[imp], color=C[imp], mec="white", mew=0.5, ms=5.5, label=f"MIXR-1 {imp[:-2]} mm"); A.set_xscale("log"); A.set_ylim(0.6, 1.5)
        A.set_xlabel(r"$N_{Re}$"); A.set_ylabel(r"$N_{Po}$ / mean $N_{Po}$ over the range"); A.set_title(f"{imp[:-2]} mm: shape of the curve"); A.legend(frameon=False, fontsize=7.3, loc="upper right")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_trend.pdf")); fig.savefig(os.path.join(a.out, "fig_trend.png"), dpi=200); plt.close(fig)
    T = pd.DataFrame(rows); T.round(3).to_csv(os.path.join(a.out, "trend_metrics.csv"), index=False); print(T.round(3).to_string())
    head = ["Impeller", "Reference", r"$N_{Re} \geq$", "$n$", "Level RMSE", "Shape RMSE", "Shape RMSE, shifted", "Shift $s$", r"Slope data", r"Slope ref."]
    L = [r"\begin{tabular}{@{}llrrrrrrr@{}}", r"\toprule", " & ".join(head) + r" \\", r"\midrule"]
    for r in rows:
        L.append(f"{r['impeller'][:-2]}~mm & {r['reference']} & ${r['from_Re'] / 1e4:g}\\times10^{{4}}$ & {r['n']} & {r['RMSE_level']:.2f} & {r['RMSE_shape']:.3f} & {r['RMSE_shape_shifted']:.3f} & {r['shift_s']:.2f} & {r['slope_data']:+.3f} & {r['slope_ref']:+.3f} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]; open(os.path.join(a.out, "trend_metrics.tex"), "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()

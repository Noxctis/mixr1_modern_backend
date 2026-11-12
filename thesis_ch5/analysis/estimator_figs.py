#!/usr/bin/env python3
"""CET vs DCT figures and numbers for Chapter 5 from logs/ch5_new/estimators/results (menu 3 of the data kit). Usage: python estimator_figs.py RESULTS_DIR OUT_DIR"""
import sys, os, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
R, O = sys.argv[1], sys.argv[2]; os.makedirs(O, exist_ok=True)
plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix", "font.size": 10,
                     "axes.grid": True, "grid.alpha": .25, "axes.spines.top": False, "axes.spines.right": False})
EST = ["dct5", "dct10", "dct20", "aligned", "newest"]
LAB = {"dct5": "DCT 5 ms", "dct10": "DCT 10 ms", "dct20": "DCT 20 ms", "aligned": "CET aligned", "newest": "CET newest edge"}
COL = {"dct5": "#E69F00", "dct10": "#D55E00", "dct20": "#CC79A7", "aligned": "#0072B2", "newest": "#009E73"}
MK = {"dct5": "^", "dct10": "v", "dct20": "D", "aligned": "o", "newest": "s"}
ol = pd.read_csv(f"{R}/open_loop_by_estimator.csv"); cb = pd.read_csv(f"{R}/closed_loop_Bimc.csv"); co = pd.read_csv(f"{R}/closed_loop_Bold.csv")
fig, ax = plt.subplots(1, 3, figsize=(12, 3.9))
for e in EST:
    d = ol[ol.est == e]; ax[0].plot(d.n, d.sd, MK[e] + "-", color=COL[e], ms=4, lw=1.2, label=LAB[e])
    d = cb[cb.est == e]; ax[1].plot(d.target, d.sd, MK[e] + "-", color=COL[e], ms=4, lw=1.2, label=LAB[e])
    d = co[co.est == e]; ax[2].plot(d.target, d.sd, MK[e] + "-", color=COL[e], ms=4, lw=1.2, label=LAB[e])
for a, t in zip(ax, ["(a) Open loop", "(b) Closed loop, gains from the identified plant", "(c) Closed loop, earlier gains"]):
    a.set_yscale("log"); a.set_xlabel("Speed (rpm)" if a is ax[0] else "Setpoint (rpm)"); a.set_ylabel("SD of the raw speed (rpm)"); a.set_title(t, fontsize=9)
ax[0].legend(fontsize=7.5); fig.tight_layout(); fig.savefig(f"{O}/fig_est_sd.pdf"); fig.savefig(f"{O}/fig_est_sd.png", dpi=130); plt.close(fig)
# numbers: mean SD over steps from 30 % PWM, with and without the 90 % step (the DCT 90 % step runs away)
s = ol[(ol.pwm_pct >= 30)]; rows = []
for e in EST:
    d = s[s.est == e]; rows.append(dict(est=e, mean_30_to_90=d.sd.mean(), mean_30_to_80=d[d.pwm_pct <= 80].sd.mean(), at_1300rpm=float(ol[(ol.est == e) & (ol.pwm_pct == 50)].sd.iloc[0]),
                                        cl_imc_ge250=cb[(cb.est == e) & (cb.target >= 250)].sd.mean(), cl_old_ge250=co[(co.est == e) & (co.target >= 250)].sd.mean(),
                                        cl_imc_1500=float(cb[(cb.est == e) & (cb.target == 1500)].sd.iloc[0])))
T = pd.DataFrame(rows); T.round(2).to_csv(f"{O}/estimator_summary.csv", index=False); print(T.round(2).to_string())

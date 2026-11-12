#!/usr/bin/env python3
"""
fig_pour.py - water-pour disturbance at 460 rpm, open loop against PI (Section 5.4, Figure fig_pour).

    python fig_pour.py --root "<School Stuff>/Research SeBoCa Test" --out ../Figs

Open loop:  mixr1_log_20260908_162112_WATEROPENLOOP_256ppr_20ms_20pwm.csv   (constant duty 20 %, three pours)
PI:         mixr1_log_20260909_163617_256ppr_20ms_WATERTESTACTUALDATA.csv    (460 rpm setpoint, four pours)
Both on the old 202 mm impeller (not used for the CFD comparison).  Writes fig_pour.pdf/.png and pour_summary.csv.
"""
import argparse, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OL = "mixr1_log_20260908_162112_WATEROPENLOOP_256ppr_20ms_20pwm.csv"
PI = "mixr1_log_20260909_163617_256ppr_20ms_WATERTESTACTUALDATA.csv"
PI_POURS = [34.0, 65.0, 95.0, 123.5]          # operator's pour times; the first is where the filtered speed starts to fall
BAND = 0.02                                   # recovery: the 0.5 s mean stays within 2 % of the speed before the pour
GREY, BLUE, TEAL = "#7A7A7A", "#0072B2", "#009E73"


def style():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix",
                         "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5, "legend.fontsize": 7.8, "axes.grid": True, "grid.color": "#DDDDDD",
                         "grid.linewidth": 0.5, "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 300, "pdf.fonttype": 42})


def find_pours(t, n, lo, hi, thr=0.04):
    """Onset of every fall of more than thr within 1 s (open loop), at least 15 s apart."""
    out, last = [], -99.0
    for i in range(len(t) - 100):
        if lo < t[i] < hi and t[i] - last > 15 and n[i] > 100 and n[i + 100] < n[i] * (1 - thr):
            out.append(float(t[i])); last = t[i]
    return out


def metrics(t, n, tp, win=12.0):
    pre = float(np.median(n[(t >= tp - 5) & (t < tp - 0.5)]))
    tt = np.arange(tp, tp + 25, 0.5); y = np.array([n[(t >= a) & (t < a + 0.5)].mean() for a in tt])
    m = tt < tp + win; imin = int(np.argmin(np.where(m, y, np.inf)))
    bad = np.abs(y - pre) > BAND * pre
    rec = float(tt[bad][-1] + 0.5 - tp) if bad.any() else 0.0
    fin = float(np.median(n[(t >= tp + 15) & (t < tp + 25)])) if (t >= tp + 25).any() else float("nan")
    sd = float(np.std(n[(t >= tp - 5) & (t < tp - 0.5)]))
    return dict(pour_s=tp, pre_rpm=pre, dip_pct=100 * (pre - y[imin]) / pre, t_dip_s=float(tt[imin] - tp), rec_s=rec, final_pct=100 * (fin - pre) / pre, pre_sd_pct=100 * sd / pre)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--root", required=True); ap.add_argument("--out", required=True); a = ap.parse_args()
    style(); rows = []; runs = {}
    for key, f in (("ol", OL), ("pi", PI)):
        d = pd.read_csv(os.path.join(a.root, f)); runs[key] = (d["t (s)"].to_numpy(), d["Raw RPM"].to_numpy(), d["Filtered RPM"].to_numpy())
    t_o, r_o, n_o = runs["ol"]; t_p, r_p, n_p = runs["pi"]
    ol_pours = find_pours(t_o, n_o, 5, 150)
    for tp in ol_pours: rows.append(dict(control="Open loop", **metrics(t_o, n_o, tp)))
    for tp in PI_POURS: rows.append(dict(control="PI", **metrics(t_p, n_p, tp)))
    T = pd.DataFrame(rows); T.round(2).to_csv(os.path.join(a.out, "pour_summary.csv"), index=False); print(T.round(2).to_string())

    for lo, hi in ((10, 30), (34, 65), (65, 95), (98, 150)):
        m = (t_p >= lo) & (t_p < hi); print("PI %d-%d s: mean filtered %.1f rpm, SD filtered %.1f, SD raw %.1f, min %.0f max %.0f" % (lo, hi, n_p[m].mean(), n_p[m].std(), r_p[m].std(), n_p[m].min(), n_p[m].max()))
    fig = plt.figure(figsize=(7.4, 4.6)); gs = fig.add_gridspec(2, 2, width_ratios=[1.55, 1], hspace=0.42, wspace=0.28)
    A = fig.add_subplot(gs[0, 0]); B = fig.add_subplot(gs[1, 0]); Cc = fig.add_subplot(gs[:, 1])
    for ax, (t, r, n, col, tit, pours) in ((A, (t_o, r_o, n_o, GREY, "(a) Open loop, constant duty of 20 %", ol_pours)),
                                           (B, (t_p, r_p, n_p, BLUE, "(b) PI, setpoint 460 rpm", PI_POURS))):
        m = (t >= 0) & (t <= 150)
        ax.plot(t[m], r[m], color="#CFCFCF", lw=0.5, label="raw"); ax.plot(t[m], n[m], color=col, lw=1.1, label="filtered")
        for x in pours: ax.axvline(x, color="#D55E00", ls=":", lw=1)
        ax.plot([], [], color="#D55E00", ls=":", label="water poured"); ax.set_xlim(0, 150); ax.set_ylim(200, 520); ax.set_ylabel("Speed (rpm)"); ax.set_title(tit, loc="left", fontsize=9)
    A.legend(frameon=False, ncol=3, loc="lower right", fontsize=7); B.set_xlabel("Time (s)")
    for t, n, pours, col, lab in ((t_o, n_o, ol_pours, GREY, "open loop"), (t_p, n_p, PI_POURS, BLUE, "PI")):
        for k, x in enumerate(pours):
            m = (t >= x - 3) & (t <= x + 20); pre = np.median(n[(t >= x - 5) & (t < x - 0.5)]); ns = pd.Series(n).rolling(50, min_periods=1, center=True).mean().to_numpy()
            Cc.plot(t[m] - x, 100 * (ns[m] / pre - 1), color=col, lw=1.0, alpha=0.9, label=lab if k == 0 else None)
    Cc.axvline(0, color="#D55E00", ls=":", lw=1); Cc.axhline(0, color="k", lw=0.5); Cc.axhspan(-100 * BAND, 100 * BAND, color=TEAL, alpha=0.12, lw=0)
    Cc.text(19.5, 100 * BAND + 0.8, r"$\pm$2 %", ha="right", fontsize=7.5, color=TEAL)
    Cc.set_xlim(-3, 20); Cc.set_ylim(-42, 10); Cc.set_xlabel("Time from the pour (s)"); Cc.set_ylabel("Speed change (%)"); Cc.set_title("(c) Each pour, 0.5 s mean", loc="left", fontsize=8)
    Cc.legend(frameon=False, loc="lower right")
    fig.savefig(os.path.join(a.out, "fig_pour.pdf"), bbox_inches="tight"); fig.savefig(os.path.join(a.out, "fig_pour.png"), dpi=170, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
    main()

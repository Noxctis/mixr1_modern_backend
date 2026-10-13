#!/usr/bin/env python3
"""
step_metrics.py - the standard step-response numbers, MEASURED straight from the data (no model, no fitting):
    rise time 10-90 %,  settling time (2 % and 5 %),  overshoot,  peak time,  final value,  steady-state error,  noise
for ANY of your step files:
    * open-loop from rest (constant PWM):          map_*.csv from  run_steps.sh map          (speed goes 0 -> final)
    * open-loop two-level step (base -> step):     <label>_r1_14_18.csv from  run_steps.sh steps   (step time found from the PWM column, or --t-step)
    * closed-loop (PI) start from rest to target:  the CSVs of  run_matrix.sh   (speed goes 0 -> target)
The file type is detected from the CSV (intended_mode, pwm, target_rpm columns).

USAGE
    python3 step_metrics.py map_*.csv                                  # open-loop steps from rest
    python3 step_metrics.py "steps_mid_unbaffled_mid_*/*.csv" --t-step 25
    python3 step_metrics.py "matrix_all_*/G*_r1.csv" "matrix_all_*/A_baseline_r*.csv"      # closed-loop starts from rest
    python3 step_metrics.py map_*.csv --plots                           # also one annotated plot per file
Options: --smooth 0.05 (s, averaging window before the crossings are measured; 0.01 = almost raw)  --out step_metrics (folder for the CSV / plots)

Definitions (y0 = value before the step, yf = final value; fraction = (y - y0)/(yf - y0)):
    t10, t90      first time the smoothed speed crosses 10 % / 90 % of the change;   RISE TIME = t90 - t10
    overshoot     (peak - yf) / (yf - y0) in %, from the smoothed speed;   peak time = when that peak occurs
    settling      the LAST time the smoothed speed is outside +-2 % (or +-5 %) of the FINAL VALUE (inf = it never stays inside)
    FAST STAGE    plants with a slow tail (water spin-up) get a second reading: 'fast plateau' = median speed 0.3-0.5 s after the step, and the 10-90 % rise of THAT.
Needs: numpy, pandas; matplotlib for --plots.
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd


def first_cross(t, frac, level):
    i = np.where(frac >= level)[0]
    return float(t[i[0]]) if len(i) else float("nan")


def settling(t, sm, yf, pct):
    out = np.abs(sm - yf) > pct / 100.0 * abs(yf)
    if not out.any():
        return 0.0
    j = np.where(out)[0][-1]
    return float("inf") if j >= len(t) - 1 else float(t[j + 1])


def measure(t, y, t0, y0, yf, smooth_s):
    """Metrics for the part of the record after t0 (the step instant). t, y: time and speed arrays."""
    dt = float(np.median(np.diff(t)))
    k = max(1, int(round(smooth_s / dt)))
    sm_all = pd.Series(y).rolling(k, center=True, min_periods=1).mean().to_numpy()
    sel = t >= t0
    ts, sm = t[sel] - t0, sm_all[sel]
    d = yf - y0
    if abs(d) < 1e-9:
        return None
    frac = (sm - y0) / d
    t10, t90 = first_cross(ts, frac, 0.1), first_cross(ts, frac, 0.9)
    ipk = int(np.argmax(frac))
    out = dict(y0=y0, final=yf, change=d, t10_s=t10, t90_s=t90, rise_10_90_s=t90 - t10, overshoot_pct=max(0.0, (frac[ipk] - 1.0) * 100.0), peak_time_s=float(ts[ipk]),
               settle_5pct_s=settling(ts, sm, yf, 5.0), settle_2pct_s=settling(ts, sm, yf, 2.0))
    # fast stage: the speed 0.3-0.5 s after the step is the plateau of the quick response; then its own 10-90 %
    win = (ts >= 0.3) & (ts < 0.5)
    if win.sum() >= 5:
        pl = float(np.median(y[sel][win]))
        if abs(pl - y0) > 0.05 * abs(d) and abs(pl - y0) < 0.97 * abs(d):          # a visible slow tail: the plateau is clearly short of the final value
            fr2 = (sm - y0) / (pl - y0)
            f10, f90 = first_cross(ts, fr2, 0.1), first_cross(ts, fr2, 0.9)
            out.update(fast_plateau=pl, fast_share_pct=(pl - y0) / d * 100.0, fast_rise_10_90_s=f90 - f10)
    return out


def classify(df, t_step_arg):
    """Returns (kind, t0, y0, text) for one daemon CSV."""
    low = {re.sub(r"[^a-z0-9]", "", c.lower()): c for c in df.columns}
    t = df[low.get("elapseds") or low.get("times") or low.get("time")].to_numpy(float)
    y = df[low.get("rawrpm") or low.get("rpm")].to_numpy(float)
    mode = str(df[low["intendedmode"]].iloc[0]) if "intendedmode" in low else ""
    pwm = df[low["pwm"]].to_numpy(float) if "pwm" in low else None
    if t_step_arg is not None:
        pre = (t >= t_step_arg - 1.0) & (t < t_step_arg - 0.05)
        return "step at --t-step", t_step_arg, float(np.median(y[pre])) if pre.sum() > 5 else 0.0, t, y
    if mode.upper().startswith("PI"):
        tg = float(df[low["targetrpm"]].median()) if "targetrpm" in low else float("nan")
        return f"closed loop from rest, target {tg:g} RPM", float(t[0]), 0.0, t, y
    if pwm is not None and np.ptp(pwm) > 0:                                        # two-level open-loop step: find the instant the PWM changes
        i = int(np.argmax(np.abs(pwm - pwm[0]) > 0.5 * np.ptp(pwm)))
        t0 = float(t[i])
        pre = (t >= t0 - 1.0) & (t < t0 - 0.05)
        return "open-loop step (two levels)", t0, float(np.median(y[pre])) if pre.sum() > 5 else float(y[:5].mean()), t, y
    return "open-loop from rest", float(t[0]), 0.0, t, y


def main():
    ap = argparse.ArgumentParser(description="Measured step-response metrics (rise 10-90 %, settling, overshoot) from your CSVs")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--t-step", type=float, default=None, help="time of the step in the file (s) if it cannot be found from the PWM column")
    ap.add_argument("--smooth", type=float, default=0.05, help="averaging window in s (default 0.05 = 5 samples, like the daemon's rise time; use 0.01 to reproduce the daemon's raw-sample overshoot/settling)")
    ap.add_argument("--out", default="step_metrics")
    ap.add_argument("--plots", action="store_true")
    args = ap.parse_args()
    files = []
    for p in args.files:
        files += sorted(glob.glob(p)) if any(ch in p for ch in "*?[") else [p]
    files = [f for f in files if os.path.isfile(f)]
    if not files:
        sys.exit("no files found")
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for f in files:
        try:
            df = pd.read_csv(f)
            kind, t0, y0, t, y = classify(df, args.t_step)
        except Exception as e:                                                       # noqa: BLE001
            print(f"  skip {os.path.basename(f)}: {e}")
            continue
        tail = (t >= t[-1] - max(1.0, 0.1 * (t[-1] - t0)))
        yf = float(np.mean(y[tail]))
        m = measure(t, y, t0, y0, yf, args.smooth)
        if m is None:
            print(f"  skip {os.path.basename(f)}: no change between the start and the end")
            continue
        m.update(file=os.path.basename(f), kind=kind, noise_rpm=float(np.std(y[tail])))
        if "targetrpm" in {re.sub(r'[^a-z0-9]', '', c.lower()) for c in df.columns}:
            tg = float(df[[c for c in df.columns if re.sub(r'[^a-z0-9]', '', c.lower()) == 'targetrpm'][0]].median())
            if tg > 0 and kind.startswith("closed"):
                m["sse_rpm"] = tg - yf
        rows.append(m)
        if args.plots:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.plot(t - t0, y, color="lightgray", lw=0.8, label="measured")
            ax.plot(t - t0, pd.Series(y).rolling(max(1, int(args.smooth / np.median(np.diff(t)))), center=True, min_periods=1).mean(), color="#2471A3", lw=1.8, label="smoothed")
            ax.axhline(yf, color="k", ls=":", lw=1)
            ax.axhspan(yf * 0.98, yf * 1.02, color="g", alpha=0.10, label="+-2 % of final")
            ax.axhspan(yf * 0.95, yf * 1.05, color="orange", alpha=0.08, label="+-5 % of final")
            for lv, nm, c in ((0.1, "10 %", "#E08A00"), (0.9, "90 %", "#C0392B")):
                tt = m["t10_s"] if lv == 0.1 else m["t90_s"]
                if tt == tt:
                    ax.axvline(tt, color=c, ls="--", lw=1.2)
                    ax.text(tt, y0 + (yf - y0) * (0.05 if lv == 0.1 else 0.55), f" {nm}\n {tt:.3g} s", color=c, fontsize=9)
            ax.set_xlim(-0.2, min(t[-1] - t0, max(2.0, (m["settle_5pct_s"] if np.isfinite(m["settle_5pct_s"]) else m["t90_s"] * 2) * 1.6)))
            ax.set_xlabel("Time since the step (s)"); ax.set_ylabel("Speed (RPM)"); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="lower right")
            ax.set_title(f"{os.path.basename(f)}: rise 10-90 % {m['rise_10_90_s']:.3g} s, settling 5 % {m['settle_5pct_s']:.3g} s, overshoot {m['overshoot_pct']:.1f} %", fontsize=10)
            fig.tight_layout(); fig.savefig(os.path.join(args.out, os.path.splitext(os.path.basename(f))[0] + "_metrics.png"), dpi=140); plt.close(fig)
    if not rows:
        sys.exit("no usable files")
    R = pd.DataFrame(rows)
    R.to_csv(os.path.join(args.out, "step_metrics.csv"), index=False)
    fmt = lambda v, p=2: ("inf" if v == float("inf") else ("-" if v != v else f"{v:.{p}f}"))        # noqa: E731
    print(f"\n{'file':>30} {'type':>34} {'start':>6} {'final':>6} | {'t10 s':>6} {'t90 s':>7} {'RISE s':>7} | {'settle 5%':>9} {'settle 2%':>9} {'overshoot %':>11} | fast stage (plateau / share / rise)")
    for r in R.itertuples():
        fast = f"{r.fast_plateau:.0f} RPM / {r.fast_share_pct:.0f} % / {r.fast_rise_10_90_s * 1000:.0f} ms" if hasattr(r, "fast_plateau") and r.fast_plateau == r.fast_plateau else "-"
        print(f"{r.file[:30]:>30} {r.kind[:34]:>34} {r.y0:6.0f} {r.final:6.0f} | {fmt(r.t10_s, 3):>6} {fmt(r.t90_s):>7} {fmt(r.rise_10_90_s):>7} | {fmt(r.settle_5pct_s):>9} {fmt(r.settle_2pct_s):>9} {r.overshoot_pct:11.1f} | {fast}")
    print(f"\n[done] {len(R)} file(s): {args.out}/step_metrics.csv" + (" and one plot per file" if args.plots else ""))


if __name__ == "__main__":
    main()
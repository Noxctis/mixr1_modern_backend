#!/usr/bin/env python3
"""
combine_gains.py -- ONE Kp/Ki pair for ALL the conditions the rig is used in (the firmware has one GLOBAL_KP / GLOBAL_KI).

    python3 combine_gains.py --plants gains/plant_96_nobaf.env gains/plant_96_baf.env gains/plant_174_nobaf.env gains/plant_174_baf.env --out gains/common.env

Each .env comes from analyze_sweep_fixed.py --emit-env for one condition (impeller, baffles, vessel filled). If <env>.ops.csv exists next to it, its three
operating points (low, mid, high speed, with the gain K that falls as the fluid drag grows) are used as three plants; otherwise the single plant is used.
For every candidate pair (Kp, Ki) the loop  L = (Kp + Ki/s) * K*A/(s+A) * exp(-(THETA + Ts/2) s)  is evaluated for every plant: gain crossover wc,
phase margin PM, gain margin GM (Ts/2 = 5 ms is the sample-and-hold of the 10 ms controller). The chosen pair maximises the crossover of the SLOWEST
plant while ALL plants keep PM >= --min-pm, GM >= --min-gm and wc <= --max-wc. It also prints, for each condition, the best pair for it alone,
so you see what the compromise costs.
"""
import argparse, os
import numpy as np, pandas as pd


def read_env(p):
    d = {}
    for ln in open(p):
        if "=" in ln: k, v = ln.strip().split("=", 1); d[k] = float(v)
    return d


def margins(kp, ki, K, A, th, tau_f=0.0, w=np.logspace(-1, 2.7, 2500)):
    s = 1j * w
    L = (kp + ki / s) * K * A / (s + A) * np.exp(-s * th) * (1 / (tau_f * s + 1) if tau_f > 0 else 1)
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    i = np.where(mag < 1)[0]
    if len(i) == 0: return np.nan, -999.0, -999.0
    i = i[0]; wc = w[i]; pm = 180 + np.degrees(ph[i])
    j = np.where(ph < -np.pi)[0]
    gm = -20 * np.log10(mag[j[0]]) if len(j) else 99.0
    return wc, pm, gm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plants", nargs="*", default=[]); ap.add_argument("--plant96"); ap.add_argument("--plant174"); ap.add_argument("--out", default=None)
    ap.add_argument("--min-pm", type=float, default=60.0); ap.add_argument("--min-gm", type=float, default=8.0)
    ap.add_argument("--max-wc", type=float, default=10.0, help="largest gain crossover allowed for any plant, rad/s (the loop runs at 100 Hz)")
    ap.add_argument("--alpha", type=float, default=1.0, help="FEEDBACK_ALPHA that will be used (1.0 = no filter)")
    ap.add_argument("--ts", type=float, default=0.010, help="controller period, s")
    a = ap.parse_args()
    files = list(a.plants) + [f for f in (a.plant96, a.plant174) if f]
    if not files: raise SystemExit("give at least one plant file with --plants")
    tau_f = 0.0 if a.alpha >= 1 else -a.ts / np.log(1 - a.alpha)
    P = []                                                                     # (condition, point, K, A, theta_total)
    for f in files:
        name = os.path.basename(f).replace(".env", "").replace("plant_", ""); e = read_env(f); ops = f + ".ops.csv"
        if os.path.exists(ops):
            for _, r in pd.read_csv(ops).iterrows(): P.append((name, r.point, r.K, r.A, r.THETA + a.ts / 2))
        else:
            P.append((name, "single", e["K"], e["A"], e["THETA"] + a.ts / 2))
    print("PLANTS (K rpm/count, pole A 1/s, dead time + half a loop period):")
    for c, pt, K, A, th in P: print(f"  {c:16s} {pt:7s} K = {K:.3f}   A = {A:5.1f} (tau {1000 / A:4.0f} ms)   loop delay {th * 1e3:4.1f} ms   K*A = {K * A:5.1f}")
    kps = np.logspace(-2.0, 0.8, 60); ratios = np.logspace(0.3, 2.0, 45)       # Ki/Kp from 2 to 100 rad/s
    feas, alone = [], {}
    for kp in kps:
        for r in ratios:
            ki = kp * r; res = [margins(kp, ki, K, A, th, tau_f) for (_, _, K, A, th) in P]
            for (c, pt, *_), (wc, pm, gm) in zip(P, res):                       # best pair for one condition alone (its own operating points)
                pass
            ok = all(pm >= a.min_pm and gm >= a.min_gm and wc <= a.max_wc for (wc, pm, gm) in res)
            for c in {x[0] for x in P}:
                rc = [r_ for (x, r_) in zip(P, res) if x[0] == c]
                if all(pm >= a.min_pm and gm >= a.min_gm and wc <= a.max_wc for (wc, pm, gm) in rc):
                    sc = min(wc for (wc, _, _) in rc)
                    if c not in alone or sc > alone[c][0]: alone[c] = (sc, kp, ki)
            if ok:
                feas.append((min(wc for (wc, _, _) in res), min(pm for (_, pm, _) in res), kp, ki, res))
    if not feas: raise SystemExit("No pair keeps the margins on ALL plants: lower --min-pm, or look at the plant that fails (the printout above).")
    top = max(f[0] for f in feas)
    near = [f for f in feas if f[0] >= 0.97 * top]                 # pairs within 3 % of the fastest slowest-loop: take the one with the most phase margin
    _, _, kp, ki, res = max(near, key=lambda f: f[1])
    print(f"\nCOMMON PAIR (every plant keeps PM >= {a.min_pm:g} deg, GM >= {a.min_gm:g} dB, crossover <= {a.max_wc:g} rad/s):   Kp = {kp:.4f}   Ki = {ki:.3f}   (Ki/Kp = {ki / kp:.1f} rad/s)")
    for (c, pt, *_), (wc, pm, gm) in zip(P, res): print(f"  {c:16s} {pt:7s} crossover {wc:5.2f} rad/s  PM {pm:5.1f} deg  GM {gm:5.1f} dB")
    print("\nWHAT THE COMPROMISE COSTS (slowest operating point of each condition, against the best pair for that condition alone):")
    for c in sorted({x[0] for x in P}):
        mine = min(wc for (x, (wc, _, _)) in zip(P, res) if x[0] == c)
        if c in alone: print(f"  {c:16s} crossover {mine:5.2f} rad/s with the common pair, {alone[c][0]:5.2f} rad/s possible alone  ->  {100 * mine / alone[c][0]:3.0f} %")
    if a.out:
        open(a.out, "w").write(f"KP={kp:.4f}\nKI={ki:.4f}\nGAINS_OK=1\n"); print(f"wrote {a.out}")


if __name__ == "__main__":
    main()

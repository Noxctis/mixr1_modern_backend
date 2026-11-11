#!/usr/bin/env python3
"""
combine_gains.py -- ONE Kp/Ki pair that works for BOTH impellers (the firmware has one GLOBAL_KP / GLOBAL_KI).

    python3 combine_gains.py --plant96 gains/plant_96.env --plant174 gains/plant_174.env --out gains/common.env

Each .env file comes from analyze_sweep_fixed.py --emit-env (K in rpm/count, A = 1/tau in 1/s, THETA = dead time in s) for that impeller fitted
and the vessel filled. For every candidate pair (Kp, Ki) the loop  L = (Kp + Ki/s) * K*A/(s+A) * exp(-THETA s) [* feedback filter]  is evaluated
for each plant: gain crossover wc, phase margin PM, gain margin GM. The chosen pair maximises the crossover of the SLOWER loop while BOTH loops
keep PM >= --min-pm and GM >= --min-gm. It also prints the best pair for each plant alone, so you can see what the compromise costs.
"""
import argparse, numpy as np


def read_env(p):
    d = {}
    for ln in open(p):
        if "=" in ln: k, v = ln.strip().split("=", 1); d[k] = float(v)
    return d


def margins(kp, ki, K, A, th, tau_f=0.0, w=np.logspace(-1, 3, 3000)):
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
    ap.add_argument("--plant96"); ap.add_argument("--plant174"); ap.add_argument("--out", default=None)
    ap.add_argument("--min-pm", type=float, default=60.0); ap.add_argument("--min-gm", type=float, default=8.0)
    ap.add_argument("--max-wc", type=float, default=10.0, help="largest gain crossover allowed for either impeller, rad/s (the loop runs at 100 Hz)")
    ap.add_argument("--alpha", type=float, default=1.0, help="FEEDBACK_ALPHA that will be used (1.0 = no filter)")
    a = ap.parse_args()
    P = {}
    for name, path in (("96 mm", a.plant96), ("174 mm", a.plant174)):
        if path: P[name] = read_env(path)
    if not P: raise SystemExit("give at least one plant file")
    tau_f = 0.0 if a.alpha >= 1 else -0.010 / np.log(1 - a.alpha)
    print("PLANTS:")
    for n, p in P.items(): print(f"  {n}: K = {p['K']:.4f} rpm/count, pole A = {p['A']:.2f} 1/s (tau {1000 / p['A']:.1f} ms), dead time {p['THETA'] * 1e3:.1f} ms")
    kps = np.logspace(-2.3, 0.7, 70); ratios = np.logspace(0, 2.0, 50)            # Ki/Kp from 1 to 100 rad/s
    best, best_alone = None, {n: None for n in P}
    for kp in kps:
        for r in ratios:
            ki = kp * r; res = {n: margins(kp, ki, p["K"], p["A"], p["THETA"], tau_f) for n, p in P.items()}
            for n, (wc, pm, gm) in res.items():                                   # best pair for one plant alone
                if pm >= a.min_pm and gm >= a.min_gm and wc <= a.max_wc and (best_alone[n] is None or wc > best_alone[n][0]): best_alone[n] = (wc, kp, ki, pm, gm)
            if all(pm >= a.min_pm and gm >= a.min_gm and wc <= a.max_wc for (wc, pm, gm) in res.values()):
                score = min(wc for (wc, _, _) in res.values())
                if best is None or score > best[0]: best = (score, kp, ki, res)
    if best is None: raise SystemExit("No pair meets the margins for both plants: lower --min-pm or tell me the plants.")
    _, kp, ki, res = best
    print(f"\nCOMMON PAIR (both impellers keep PM >= {a.min_pm:g} deg, GM >= {a.min_gm:g} dB and crossover <= {a.max_wc:g} rad/s):   Kp = {kp:.4f}   Ki = {ki:.3f}   (Ki/Kp = {ki / kp:.1f} rad/s)")
    for n, (wc, pm, gm) in res.items():
        alone = best_alone[n]
        print(f"  {n}: crossover {wc:5.2f} rad/s, PM {pm:5.1f} deg, GM {gm:5.1f} dB   | best possible for this impeller alone: crossover {alone[0]:5.2f} rad/s (Kp {alone[1]:.3f}, Ki {alone[2]:.2f})"
              f"  -> the compromise gives it {100 * wc / alone[0]:.0f} % of that speed")
    if a.out:
        open(a.out, "w").write(f"KP={kp:.4f}\nKI={ki:.4f}\nGAINS_OK=1\n"); print(f"wrote {a.out}")


if __name__ == "__main__":
    main()

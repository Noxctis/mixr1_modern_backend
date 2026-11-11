#!/usr/bin/env python3
"""
analyze_sweep_fixed.py -- step-response identification and IMC PI tuning for MIXR-1.

Corrections to analyze_sweep.py
  1. IMC formulas.  For a plant G(s) = Kp_plant * A/(s + A)  (K in rpm per PWM count, pole A in 1/s)
     the PI controller that cancels the pole and gives a closed-loop bandwidth w_c = 1/Tc is
         Kp = 1 / (K * A * Tc)          Ki = 1 / (K * Tc)          (Ki/Kp = A)
     The old script used Kp = 1/(K*Tc) and Ki = A/(K*Tc), i.e. BOTH GAINS TOO LARGE BY A, so the real
     bandwidth was A times the label (about 25 times for this rig).
  2. Dead time.  The speed response has about 8 ms of dead time (estimator delay + driver), a quarter of the time constant. A first-order fit without it is biased and depends on the time origin.
     The default model here is first order plus dead time (FOPDT); with dead time theta the IMC rule is
         Kp = 1 / (K * A * (Tc + theta))     Ki = 1 / (K * (Tc + theta))
  3. Time origin: t = 0 is the FIRST row of the new step, the row that already carries the new command. (Taking the last row of the previous step
     adds one loop period, 10 ms, to the dead time; that was an error of an earlier version, found by comparing with measured closed-loop steps.)
  4. Steps limited by the speed or power cap (K much lower than the median) are flagged and excluded.
  5. No prompts: python analyze_sweep_fixed.py FILE.csv --bw 5 --out results

Limits that remain: the log is at 100 Hz and the time constant is about 30-40 ms, so the pole and the dead
time are only resolved to about +/-20 %. For a better identification log the aligned snapshots at every
edge around one PWM step and compute the speed offline (see PPR_retest_protocol.md).
"""
import argparse, os
import numpy as np, pandas as pd
from scipy.optimize import curve_fit
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def first_order(t, base, K, A):
    return base + K * (1 - np.exp(-A * t))


def fopdt(t, base, K, A, th):
    return base + K * (1 - np.exp(-A * np.clip(t - th, 0, None)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv"); ap.add_argument("--bw", type=float, default=5.0, help="target closed-loop bandwidth, rad/s")
    ap.add_argument("--model", choices=["fopdt", "first"], default="fopdt"); ap.add_argument("--out", default="sweep_results")
    ap.add_argument("--emit-env", default=None, help="write KP= KI= K= A= THETA= GAINS_OK= lines to this file (for the shell scripts)")
    ap.add_argument("--min-seconds", type=float, default=6.0, help="ignore a step shorter than this (the run was aborted inside it)")
    ap.add_argument("--skip-first", type=int, default=1, help="skip this many steps after step 0 (from-rest steps include stiction)")
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    df = pd.read_csv(a.csv); Tc = 1.0 / a.bw; rows = []
    for i in [s for s in sorted(df.step_index.unique()) if s > a.skip_first]:
        prev, cur = df[df.step_index == i - 1], df[df.step_index == i]
        if prev.empty or cur.empty: continue
        if cur.elapsed_s.iloc[-1] - cur.elapsed_s.iloc[0] < a.min_seconds: continue
        base = prev[prev.elapsed_s >= prev.elapsed_s.max() - 1.5].raw_rpm.mean()
        dpwm = float(cur.pwm.iloc[0] - prev.pwm.iloc[-1])
        if dpwm == 0: continue
        t = cur.elapsed_s.to_numpy(float) - cur.elapsed_s.iloc[0]; y = cur.raw_rpm.to_numpy(float)   # t = 0 at the FIRST row of the step: that row already carries the new command
        k0 = float(np.median(y[-20:]) - base)                      # initial guess: where the response ends (negative for a step down)
        try:
            if a.model == "fopdt":
                p, _ = curve_fit(lambda t, K, A, th: fopdt(t, base, K, A, th), t, y, p0=[k0, 25.0, 0.015],
                                 bounds=([-5000, 1, 0], [5000, 300, 0.2])); K, A, th = p; mdl = lambda t: fopdt(t, base, K, A, th)
            else:
                p, _ = curve_fit(lambda t, K, A: first_order(t, base, K, A), t, y, p0=[k0, 20.0]); K, A = p; th = 0.0
                mdl = lambda t: first_order(t, base, K, A)
        except Exception as exc:
            print(f"step {i}: fit failed ({exc})"); continue
        rms = float(np.sqrt(np.mean((y - mdl(t)) ** 2)))
        Kpl = K / dpwm
        rows.append(dict(step=int(i), pwm_from=int(prev.pwm.iloc[-1]), pwm_to=int(cur.pwm.iloc[0]), direction=("up" if dpwm > 0 else "down"), base_rpm=base, K_plant=Kpl, A=A,
                         tau_ms=1e3 / A, theta_ms=th * 1e3, fit_rms_rpm=rms))
        fig, ax = plt.subplots(figsize=(5, 3)); ax.plot(t, y, ".", ms=3, alpha=.5); ax.plot(t, mdl(t), "r-")
        ax.set_xlim(0, 0.6); ax.set_xlabel("t since the command (s)"); ax.set_ylabel("rpm"); ax.set_title(f"step {i}: tau {1e3 / A:.0f} ms, dead time {th * 1e3:.0f} ms")
        fig.tight_layout(); fig.savefig(os.path.join(a.out, f"step_{i}.png"), dpi=150); plt.close(fig)
    R = pd.DataFrame(rows)
    # a step is unreliable if the speed was capped (gain falls by a quarter against the step before), if its pole is absurd against the median
    # (the response is too small or too fast for a 100 Hz log), or if its fit is much worse than the others
    R = R.sort_values("base_rpm").reset_index(drop=True)
    moving = R.base_rpm >= 50                                  # a step that starts from rest includes the breakaway (stiction) of the shaft
    Rm = R[moving]
    Amed, rmsmed = Rm.A.median(), Rm.fit_rms_rpm.median()
    capped = pd.Series(False, index=R.index); capped.loc[Rm.index] = (Rm.K_plant < 0.75 * Rm.K_plant.shift(1)).fillna(False)
    R["limited"] = (~moving) | capped | (R.A < Amed / 3) | (R.A > Amed * 3) | (R.fit_rms_rpm > 2.5 * rmsmed)
    good = R[~R.limited]
    K, A, th = good.K_plant.median(), good.A.median(), good.theta_ms.median() / 1e3
    Kp, Ki = 1 / (K * A * (Tc + th)), 1 / (K * (Tc + th))
    R.to_csv(os.path.join(a.out, "identification.csv"), index=False)
    for d_, g_ in good.groupby("direction"):
        print(f"  {d_:>4}: K = {g_.K_plant.median():.4f} rpm/count, tau = {1e3 / g_.A.median():.1f} ms, dead time = {g_.theta_ms.median():.1f} ms  ({len(g_)} steps)")
    pd.set_option("display.width", 200); print(R.round(3).to_string(index=False))
    print(f"\nmedian over unlimited steps: K = {K:.4f} rpm/count, A = {A:.2f} 1/s (tau {1e3 / A:.1f} ms), dead time {th * 1e3:.1f} ms")
    print(f"target bandwidth {a.bw:g} rad/s -> IMC gains  Kp = {Kp:.4f}  Ki = {Ki:.4f}   (Ki/Kp = {Ki / Kp:.2f} = pole)")
    print(f"gain crossover of the designed loop  Kp*K*A = {Kp * K * A:.2f} rad/s  (check against the target)")
    print("Check it in closed loop before using it: the feedback filter (FEEDBACK_ALPHA) and the loop delay add phase lag.")
    ops = None
    if len(good) >= 3:
        g_ = good.sort_values("base_rpm").reset_index(drop=True); parts = np.array_split(np.arange(len(g_)), 3)
        ops = pd.DataFrame([dict(point=lab, base_rpm=float(g_.iloc[ix].base_rpm.median()), K=float(g_.iloc[ix].K_plant.median()), A=float(good.A.median()), THETA=float(good.theta_ms.median() / 1e3))
                            for lab, ix in zip(("low", "mid", "high"), parts)])
        print("\noperating points (K falls with speed because the drag of the fluid grows; the pole and the dead time are the medians of all reliable steps):")
        print(ops.round(4).to_string(index=False))
    if a.emit_env:
        ok = bool(len(good) >= 4 and 0.2 < K < 1.5 and 5 < A < 120 and 0 <= th < 0.1)
        if not ok:
            print("WARNING: the identified plant looks wrong (K or the pole or the dead time is out of range); using safe default gains.")
            Kp, Ki = 0.2, 7.0
        with open(a.emit_env, "w") as f:
            f.write(f"KP={Kp:.4f}\nKI={Ki:.4f}\nK={K:.4f}\nA={A:.3f}\nTHETA={th:.4f}\nGAINS_OK={int(ok)}\n")
        print(f"wrote {a.emit_env}: KP={Kp:.4f} KI={Ki:.4f} (GAINS_OK={int(ok)})")
        if ops is not None: ops.to_csv(a.emit_env + ".ops.csv", index=False)


if __name__ == "__main__":
    main()

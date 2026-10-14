#!/usr/bin/env python3
"""
textbook_baseline.py - the textbook way to get a baseline Kp and Ki from OPEN-LOOP STEP RESPONSES (0 -> X % duty), then hand it to the 81-run matrix.

WHAT THE BOOKS DO (references at the bottom)
  1. Apply a step to the PWM with the controller out of the loop (open loop) and record the speed.
  2. Fit a FIRST-ORDER-PLUS-DEAD-TIME (FOPDT) model   G(s) = K e^(-theta s) / (tau s + 1)
        K     = (change in speed) / (change in PWM)                                   process gain, RPM per PWM count
        theta = time from the step until the speed STARTS to respond                  dead time (a pure delay: nothing happens yet)
        tau   = time from theta until the speed reaches 63.2 % of its final change    time constant
  3. Turn K, tau, theta into PI gains (all three rules are the same idea: pick how fast you want the closed loop, cancel the plant's pole):
        YOUR FORMULA (pole-zero cancellation)  Kp = 1/K,                Ki = Kp/tau          (closed loop ~ first order with time constant tau)
        IMC / lambda tuning (Rivera-Morari-Skogestad 1986)  Kp = tau / (K (lambda + theta)),  Ki = Kp/tau        (lambda = chosen closed-loop time constant)
        SIMC (Skogestad 2003)                  Kp = tau / (K (tau_c + theta)),  Ti = min(tau, 4 (tau_c + theta)), Ki = Kp/Ti    (tau_c = theta is the 'tight' choice)
     With theta = 0 and lambda = tau the IMC rule IS your formula.

USAGE
    python3 textbook_baseline.py map_*.csv --work-rpm 350                 # prints K, theta, tau for every step and the gains; writes baseline_gains.txt
    python3 textbook_baseline.py "steps_mid_unbaffled_*/map_*.csv" --work-rpm 350 --out baseline_unbaffled
    then:   sudo ./run_matrix.sh 350 40 1 --baseline-file baseline_unbaffled/baseline_gains.txt --out matrix81 --pause 30 ...
Options: --rule yourformula|imc|simc (which rule sets the baseline, default yourformula)   --lambda-factor 1.0 (IMC: lambda = factor * tau)   --n-steps 3 (steps nearest the working speed that are used)
Needs: numpy, pandas; matplotlib for the plots.

CHECK IT DOES FOR YOU: a textbook FOPDT assumes ONE smooth first-order response. If tau comes out very different from step to step (or the fit is poor) the response has more than one time scale
(your motor in water: a quick jump of the motor + impeller, then a slow creep as the water spins up). The tool warns when that happens, because then the baseline depends on WHICH step you read.

References: Rivera, Morari, Skogestad, "Internal model control: PID controller design", Ind. Eng. Chem. Process Des. Dev. 25 (1986) 252-265.
            Skogestad, "Simple analytic rules for model reduction and PID controller tuning", J. Process Control 13 (2003) 291-309.
            Seborg, Edgar, Mellichamp, Doyle, "Process Dynamics and Control" (FOPDT from step response, IMC tuning);  Sundaresan & Krishnaswamy (1978) two-point method.
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd


def first_time(t, y, level):
    i = np.where(y >= level)[0]
    return float(t[i[0]]) if len(i) else float("nan")


def analyse_file(path, smooth_n=5):
    d = pd.read_csv(path)
    low = {re.sub(r"[^a-z0-9]", "", c.lower()): c for c in d.columns}
    tcol = next((low[k] for k in ("elapseds", "times", "time", "t") if k in low), None)
    rcol = next((low[k] for k in ("rawrpm", "rpm") if k in low), None)
    if tcol is None or rcol is None:
        raise ValueError("needs a time column and a raw_rpm column")
    t, y = d[tcol].to_numpy(float), d[rcol].to_numpy(float)
    if "pwm" in low:
        u = float(np.median(d[low["pwm"]]))
    else:
        m = re.search(r"(\d+(?:\.\d+)?)\.csv$", os.path.basename(path))
        if not m:
            raise ValueError("no pwm column and no duty in the file name")
        u = float(m.group(1)) / 100.0 * 4095.0
    sm = pd.Series(y).rolling(smooth_n, center=True, min_periods=1).mean().to_numpy()
    tail = t >= t[-1] - max(3.0, 0.1 * t[-1])
    yf, noise = float(np.mean(y[tail])), float(np.std(y[tail]))
    if yf < 5 * noise:
        raise ValueError(f"final speed {yf:.1f} RPM is not clearly above the noise ({noise:.1f} RPM): duty below the friction dead zone?")
    K = yf / u
    theta = float(t[np.argmax(sm > max(3 * noise, 0.02 * yf))])                          # first CLEAR response
    t63 = first_time(t, sm, 0.632 * yf)
    tau = max(t63 - theta, 0.01)
    # Sundaresan-Krishnaswamy two-point method (35.3 % and 85.3 %): tau = 0.67 (t85 - t35), theta = 1.3 t35 - 0.29 t85
    t35, t85 = first_time(t, sm, 0.353 * yf), first_time(t, sm, 0.853 * yf)
    tau_sk, theta_sk = 0.67 * (t85 - t35), max(1.3 * t35 - 0.29 * t85, 0.0)
    win = (t >= 0.3) & (t < 0.5)
    fast_share = float(np.median(y[win]) / yf * 100.0) if win.sum() > 4 else float("nan")
    tau_quick = float("nan")                                                              # time constant of the QUICK stage alone: 63.2 % of its own plateau
    if win.sum() > 4 and float(np.median(y[win])) > 5 * noise:
        tau_quick = max(first_time(t, sm, 0.632 * float(np.median(y[win]))) - theta, 0.01)
    return dict(file=os.path.basename(path), duty_pct=u / 40.95, counts=u, final_rpm=yf, K=K, theta_s=theta, tau_s=tau, tau_sk_s=tau_sk, theta_sk_s=theta_sk, noise_rpm=noise,
                fast_share_pct=fast_share, tau_quick_s=tau_quick, t=t, y=y, sm=sm)


def gains(K, tau, theta, rule, lam_factor, lam_s=None):
    if rule == "yourformula":
        kp = 1.0 / K
        return kp, kp / tau, f"Kp = 1/K, Ki = Kp/tau   (IMC with lambda = tau and the dead time ignored)"
    if rule == "imc":
        lam = lam_s if lam_s else lam_factor * tau
        kp = tau / (K * (lam + theta))
        return kp, kp / tau, f"IMC: Kp = tau/(K (lambda + theta)), Ki = Kp/tau   (lambda = {lam:.3g} s: closed-loop time constant, i.e. closed-loop bandwidth about {1 / lam:.3g} rad/s)"
    tc = theta
    kp = tau / (K * (tc + theta))
    ti = min(tau, 4 * (tc + theta))
    return kp, kp / ti, "SIMC tight (tau_c = theta): Kp = tau/(K (tau_c + theta)), Ti = min(tau, 4 (tau_c + theta))"


def main():
    ap = argparse.ArgumentParser(description="Textbook FOPDT baseline Kp, Ki from open-loop 0 -> X % step responses")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--work-rpm", type=float, default=350.0, help="working speed: the steps whose final speed is nearest this one set the baseline")
    ap.add_argument("--n-steps", type=int, default=3, help="how many steps (nearest the working speed) define the baseline")
    ap.add_argument("--all-steps", action="store_true", help="use EVERY step (median) instead of the ones nearest the working speed")
    ap.add_argument("--duties", default=None, help='use exactly these steps, e.g. "14 18 22" (duty %%)')
    ap.add_argument("--rule", default="yourformula", choices=("yourformula", "imc", "simc"))
    ap.add_argument("--tau-method", default="total", choices=("total", "quick"),
                    help="total = tau from 63.2 %% of the TOTAL change (the textbook reading, default); quick = tau from 63.2 %% of the QUICK stage only (steadier when the response has a slow tail)")
    ap.add_argument("--lambda-factor", type=float, default=1.0)
    ap.add_argument("--lambda-s", type=float, default=None, help="IMC closed-loop time constant in SECONDS (bandwidth = 1/lambda rad/s): --rule imc --lambda-s 1 designs for a 1 rad/s closed loop")
    ap.add_argument("--smooth-samples", type=int, default=5, help="samples averaged before reading the 63.2 %% point (the speed is sampled every 10 ms)")
    ap.add_argument("--out", default="textbook_baseline")
    args = ap.parse_args()
    files = []
    for p in args.files:
        files += sorted(glob.glob(p)) if any(ch in p for ch in "*?[") else [p]
    rows = []
    for f in files:
        if not os.path.isfile(f):
            continue
        try:
            rows.append(analyse_file(f, args.smooth_samples))
        except Exception as e:                                                       # noqa: BLE001
            print(f"  skip {os.path.basename(f)}: {e}")
    if len(rows) < 2:
        sys.exit("need at least 2 usable open-loop step files from rest (more duties across your range is better)")
    rows_all = sorted(rows, key=lambda r: r["duty_pct"])
    # repeats of the same duty are combined (median) so that 3 repeats of one duty cannot crowd out the others
    groups = {}
    for r in rows_all:
        groups.setdefault(round(r["duty_pct"]), []).append(r)
    rows = []
    for dpct, rs in sorted(groups.items()):
        med = lambda k: float(np.median([x[k] for x in rs]))                       # noqa: E731
        taus_g = [x["tau_s"] for x in rs]
        rows.append(dict(file=f"{len(rs)} file(s)" if len(rs) > 1 else rs[0]["file"], duty_pct=float(dpct), counts=med("counts"), final_rpm=med("final_rpm"), K=med("K"), theta_s=med("theta_s"),
                         tau_s=med("tau_s"), tau_quick_s=med("tau_quick_s"), tau_sk_s=med("tau_sk_s"), theta_sk_s=med("theta_sk_s"), noise_rpm=med("noise_rpm"), fast_share_pct=med("fast_share_pct"),
                         n=len(rs), tau_repeat_spread=max(taus_g) / min(taus_g)))
    os.makedirs(args.out, exist_ok=True)
    print("\nFOPDT read from each open-loop step (K = final speed / duty step;  theta = time to the first clear response;  tau = (time to 63.2 % of the final speed) - theta)\n")
    tau_key = "tau_quick_s" if args.tau_method == "quick" else "tau_s"
    print(f"(time constant used below: {'63.2 % of the QUICK stage' if args.tau_method == 'quick' else '63.2 % of the TOTAL change'};  the other reading is shown in the last-but-one column)")
    print(f"{'duty':>5} {'final RPM':>9} {'K RPM/count':>11} {'theta ms':>9} {'tau s':>7} {'1/tau rad/s':>11} | {'S&K tau s':>9} {'S&K theta ms':>12} | {'quick stage at 0.3-0.5 s':>24} | {'runs':>4} | your formula: Kp, Ki")
    for r in rows:
        kp, ki, _ = gains(r["K"], r[tau_key], r["theta_s"], "yourformula", 1.0)
        r["kp_yf"], r["ki_yf"] = kp, ki
        print(f"{r['duty_pct']:4.0f}% {r['final_rpm']:9.0f} {r['K']:11.3f} {r['theta_s'] * 1000:9.0f} {r[tau_key]:7.2f} {1 / r[tau_key]:11.2f} | {r['tau_sk_s']:9.2f} {r['theta_sk_s'] * 1000:12.0f} | "
              f"{r['fast_share_pct']:21.0f} % | {r['n']:>4} | {kp:5.2f}, {ki:7.2f}")
    taus = np.array([r["tau_s"] for r in rows])
    spread = taus.max() / taus.min()
    warn = []
    if spread > 3:
        warn.append(f"tau changes {spread:.0f}x between steps ({taus.min():.2f} s to {taus.max():.2f} s): the response is NOT a single first-order lag. "
                    f"Your motor in water has a quick stage and a slow creep (the 'quick stage at 0.3-0.5 s' column shows how much of the final speed the quick stage delivers). "
                    f"Where the quick stage already passes 63.2 % of the final speed, tau is the quick stage's (small); where it does not, tau is the slow creep's (large). "
                    f"The baseline therefore depends on WHICH steps you read: use the steps at your WORKING speed.")
    for w in warn:
        print("\n!! " + w)
    print("\nPRECISION: the quick stage lasts only a few 10 ms samples, so theta and tau cannot be separated well (only their SUM is reliable, about "
          f"{np.median([r['theta_s'] + r['tau_s'] for r in rows if r['tau_s'] < 0.3]) * 1000:.0f} ms here). The IMC rule uses that sum (Ki = 1/(K (lambda + theta)) with lambda = tau), so it is the steadier of the two; "
          "'your formula' (Kp = 1/K, Ki = Kp/tau) can move by up to a factor of 2 in Ki with the smoothing window. Treat the baseline Ki as +-30 %.")
    # baseline from the steps nearest the working speed
    if args.duties:
        want = {round(float(x)) for x in args.duties.replace(",", " ").split()}
        near = [r for r in rows if round(r["duty_pct"]) in want]
        how = "the steps you chose"
    elif args.all_steps:
        near, how = list(rows), "ALL steps"
    else:
        near = sorted(rows, key=lambda r: abs(r["final_rpm"] - args.work_rpm))[:args.n_steps]
        how = f"the {args.n_steps} steps whose STEADY-STATE speed is nearest {args.work_rpm:g} RPM"
    if not near:
        sys.exit("none of the chosen steps exist in the data")
    near.sort(key=lambda r: r["duty_pct"])
    edge = [r for r in near if r["fast_share_pct"] == r["fast_share_pct"] and abs(r["fast_share_pct"] - 63.2) < 5]
    if edge and args.tau_method == "total":
        print("\n!! ON THE EDGE: " + ", ".join(f"{r['duty_pct']:.0f}% (the quick stage delivers {r['fast_share_pct']:.1f} % of the final speed)" for r in edge)
              + ". The 63.2 %-of-total reading jumps from about 0.05 s to about 1 s (Ki from ~47 to ~2) when that share falls below 63.2 %, so these steps are close to the cliff. "
              "Kp = 1/K is steady; Ki is the uncertain one. Check with  --tau-method quick  (reads the quick stage on its own, no cliff) and with more repeats.")
    shares = [r["fast_share_pct"] for r in near if r["fast_share_pct"] == r["fast_share_pct"]]
    if shares and float(np.median(shares)) < 58.0 and args.tau_method == "total":
        kp_m = float(np.median([1.0 / r["K"] for r in near]))
        ki_tot = float(np.median([(1.0 / r["K"]) / r["tau_s"] for r in near]))
        qs = [(1.0 / r["K"]) / r["tau_quick_s"] for r in near if r["tau_quick_s"] == r["tau_quick_s"] and r["tau_quick_s"] > 0]
        ki_q = float(np.median(qs)) if qs else float("nan")
        print(f"\n!! TWO-STAGE PLANT: at these steps the quick stage delivers only {np.median(shares):.0f} % of the final speed (the rest is a slow creep of seconds).")
        print(f"   The time constant therefore depends on WHICH stage you read: Ki = {ki_tot:.3g} from the total response (63.2 % of the whole change, the slow creep)")
        print(f"   but Ki = {ki_q:.3g} from the quick stage alone -- a factor of {ki_q / ki_tot:.0f}. Neither single number is 'the' answer: with ONE time constant the method cannot describe this plant.")
        print(f"   Kp = 1/K = {kp_m:.3g} is steady. Decide Ki with a short Ki sweep (see the advice in the thesis procedure), not from one reading.")
    print("\nHOW THE BASELINE IS CHOSEN: every step above is a run from 0 to X % duty. For each run, K comes from its STEADY STATE (the last 3 s), and theta and tau from its early response.")
    print("  The numbers for every duty are in the table. The baseline is the MEDIAN over the steps whose steady-state speed is closest to the speed you will actually run at, because the plant is not")
    print("  exactly linear (K and tau change with speed) and the controller works at that speed. Use --all-steps to use every run, or --duties \"14 18 22\" to pick them yourself.")
    print(f"\nBASELINE for the matrix: using {how}: {', '.join(f'{r['duty_pct']:.0f}% ({r['final_rpm']:.0f} RPM)' for r in near)}")
    res = {}
    for rule in ("yourformula", "imc", "simc"):
        kps, kis = [], []
        for r in near:
            kp, ki, desc = gains(r["K"], r[tau_key], r["theta_s"], rule, args.lambda_factor, args.lambda_s)
            kps.append(kp)
            kis.append(ki)
        if rule == "yourformula":
            desc2 = "Kp = 1/K, Ki = Kp/tau   (pole-zero cancellation = IMC with lambda = tau and the dead time ignored; this formula reproduces the ORIGINAL air gains 1.5641 / 41.2249 for K = 0.6393, A = 26.36 /s; whether it is the method your professor wants is for them to confirm)"
        elif rule == "imc":
            desc2 = ((f"IMC (Rivera): Kp = tau/(K (lambda + theta)), Ki = Kp/tau, lambda = {args.lambda_s:g} s for every step, i.e. closed-loop bandwidth about {1 / args.lambda_s:.3g} rad/s")
                      if args.lambda_s else
                      f"IMC (Rivera) with the dead time: Kp = tau/(K (lambda + theta)), Ki = Kp/tau, lambda = {args.lambda_factor:g} x tau of EACH step; median over the steps")
        else:
            desc2 = "SIMC tight (tau_c = theta): assumes the dead time is known exactly, which 100 Hz sampling cannot give: it is UNSTABLE on this plant in simulation, do not use"
        res[rule] = (float(np.median(kps)), float(np.median(kis)), desc2)
        print(f"  {rule:>12}:  Kp = {res[rule][0]:7.3f}   Ki = {res[rule][1]:8.3f}     [{res[rule][2]}]")
    kp, ki, desc = res[args.rule]
    loopgain = kp * float(np.median([r["K"] for r in near]))
    if loopgain > 4:
        print(f"\n!! Kp x K = {loopgain:.1f}: a loop gain this large at high frequency usually hunts or amplifies noise once the real delay of the speed measurement is included. Do not use it without checking.")
    open(os.path.join(args.out, "baseline_gains.txt"), "w").write(
        f"kp={kp:.4f} ki={ki:.4f}   # rule: {desc}; steps {', '.join(f'{r['duty_pct']:.0f}%' for r in near)} (~{args.work_rpm:g} RPM)\n")
    pd.DataFrame([{k: v for k, v in r.items() if k not in ("t", "y", "sm")} for r in rows]).to_csv(os.path.join(args.out, "textbook_fopdt.csv"), index=False)
    pd.DataFrame([{k: v for k, v in r.items() if k not in ("t", "y", "sm")} for r in rows_all]).to_csv(os.path.join(args.out, "textbook_fopdt_every_file.csv"), index=False)
    spread = max((r.get("tau_repeat_spread", 1.0) for r in rows), default=1.0)
    if spread > 1.5:
        print(f"\nNOTE: repeats of the same duty differ by up to {spread:.1f}x in tau (10 ms sampling): this is the resolution limit, not a different plant. The median of the repeats is used.")
    print(f"\n>>> NEW BASELINE (rule: {args.rule}):  Kp = {kp:.4f}   Ki = {ki:.4f}      -> {args.out}/baseline_gains.txt")
    print(f"    next:  sudo ./run_matrix.sh {args.work_rpm:g} 40 1 --baseline-file {args.out}/baseline_gains.txt --out matrix81 --pause 30 --max-duty 35")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cm = plt.get_cmap("viridis")
        fig, ax = plt.subplots(1, 2, figsize=(14, 5.4))
        for i, r in enumerate(rows_all):
            c = cm(i / max(1, len(rows_all) - 1))
            for a in ax:
                a.plot(r["t"], r["sm"], color=c, lw=1.4, label=f"{r['duty_pct']:.0f} %" if a is ax[0] else None)
                a.plot([r["theta_s"] + r["tau_s"]], [0.632 * r["final_rpm"]], "o", color=c, ms=6, mec="k")
        ax[0].set_title("Open-loop steps from rest. Dots = the 63.2 % point the textbook method reads"); ax[1].set_title("First 1.5 s"); ax[1].set_xlim(0, 1.5)
        for a in ax:
            a.set_xlabel("Time since the step (s)"); a.set_ylabel("Speed (RPM)"); a.grid(alpha=0.3)
        ax[0].legend(ncol=2, fontsize=8.5, loc="lower right")
        fig.tight_layout(); fig.savefig(os.path.join(args.out, "textbook_fopdt_points.png"), dpi=140); plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
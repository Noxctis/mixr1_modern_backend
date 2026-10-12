#!/usr/bin/env python3
"""
analyze_step.py (v2) - open-loop PWM step-response analyser and PI tuning check for MIXR-1.

What it does
  1. Reads every step file (CSV with Time_s and Raw_RPM; step levels taken from the file name  ..._<base%>_<step%>.csv).
  2. Fits   y(t) = y0 + K * (1 - exp(-A * (t - theta)))   for t > theta   (first order PLUS DEAD TIME theta).
     The old fit had no dead time; your runs show the speed only starts to rise ~30-40 ms after the command, which
     makes a plain first-order fit read the pole A too low.
  3. Plant gain K_plant = K / delta_PWM_counts (RPM per count), time constant tau = 1/A, delay theta.
  4. Compares with the plant your CURRENT gains were designed for (Kp = 1/K_plant, Ki = Kp*A  ->  K_plant = 1/Kp, A = Ki/Kp).
  5. Proposes PI gains (your formula, IMC with delay, SIMC conservative / tight) and, for each, PREDICTS the stability margins
     (phase margin, gain margin, crossover) including the feedback filter alpha. Low phase margin = overshoot / ringing.
  6. Optional model check: --evaluate-folder <matrix folder> predicts the phase margin of every Kp/alpha you tested and
     compares it with the overshoot you actually measured.

Usage
    python3 analyze_step.py                                          # all water_raw_fifo_*.csv in this folder
    python3 analyze_step.py "water_raw_fifo_*.csv"                   # a pattern / files
    python3 analyze_step.py noload="noload_*.csv" baffled="baffled_*.csv"      # label=pattern: compare conditions side by side
    python3 analyze_step.py baffled="baffled_*.csv" --evaluate-folder matrix_baffled_water
    python3 analyze_step.py --pick                                   # old behaviour: choose one file interactively

Options:  --out step_analysis  --t-step 1.0  --alpha 1.0  --current-kp 1.5641 --current-ki 41.2249  --min-delay 0.01  --no-plots
Needs: numpy, pandas, scipy, matplotlib.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares

LOOP_T = 0.01                         # control loop period (s); the feedback filter alpha runs at this rate
DEF_KP, DEF_KI = 1.5641, 41.2249      # the gains currently in config.hpp
ADC = 4095.0                          # PWM counts at 100 %
OMEGA = np.logspace(-1, np.log10(310), 5000)       # rad/s up to the Nyquist limit of the 100 Hz loop


# ============================================================================================ loading
def _norm(name):
    return re.sub(r"[^a-z0-9]", "", str(name).lower())


def load_step_csv(path):
    df = pd.read_csv(path)
    cols = {_norm(c): c for c in df.columns}
    tcol = next((cols[k] for k in ("times", "time", "elapseds", "ts", "t") if k in cols), None)
    rcol = next((cols[k] for k in ("rawrpm", "rpm", "rawrpm") if k in cols), None)
    if tcol is None or rcol is None:
        raise KeyError(f"{os.path.basename(path)}: needs a time column (Time_s) and a speed column (Raw_RPM); found {list(df.columns)}")
    pcol = next((cols[k] for k in ("pwm", "pwmcounts", "pwmpct", "pwmpercent", "pwmduty") if k in cols), None)
    t = df[tcol].to_numpy(float)
    rpm = df[rcol].to_numpy(float)
    pwm = df[pcol].to_numpy(float) if pcol else None
    meta = {"fifo_active": None, "late_p99_us": float("nan")}                 # what the daemon itself recorded (None = the file does not say)
    if "fifoactive" in cols:
        meta["fifo_active"] = str(df[cols["fifoactive"]].iloc[0]).strip().lower() == "true"
    if "lateus" in cols:
        meta["late_p99_us"] = float(np.nanpercentile(df[cols["lateus"]].to_numpy(float), 99))
    return t, rpm, pwm, meta


def parse_levels(path, pwm):
    m = re.search(r"(\d+(?:\.\d+)?)_(\d+(?:\.\d+)?)\.csv$", os.path.basename(path))
    if m:
        return float(m.group(1)), float(m.group(2))
    if pwm is not None and np.ptp(pwm) > 0:
        scale = 100.0 / ADC if np.nanmax(pwm) > 100 else 1.0
        n = max(3, len(pwm) // 20)
        return float(np.median(pwm[:n]) * scale), float(np.median(pwm[-n:]) * scale)
    raise ValueError(f"{os.path.basename(path)}: cannot tell the step levels - name the file like water_raw_fifo_<base%>_<step%>.csv")


# ============================================================================================ the fit
def fit_fopdt(t, rpm, t_step, two_mode=False, fast_tau=None):
    """Fit y0 + K(1-exp(-A(t-theta))) to one step. Returns a dict (or raises ValueError)."""
    tr = t - t_step
    pre = (tr >= -0.8) & (tr < -0.05)
    if pre.sum() < 15:
        raise ValueError("need at least ~0.2 s of baseline before the step")
    y0 = float(np.median(rpm[pre]))
    noise = max(float(np.std(rpm[pre])), 0.3)
    pre_long = (tr >= -3.0) & (tr < -0.05)                                     # a longer window makes the slope estimate ~8x less noisy
    pw = pre_long if pre_long.sum() >= 150 else pre
    base_slope = float(np.polyfit(tr[pw], rpm[pw], 1)[0])                      # RPM per second before the step: should be ~0
    sxx = float(np.sum((tr[pw] - tr[pw].mean()) ** 2))
    base_slope_se = float(np.std(rpm[pw] - np.polyval(np.polyfit(tr[pw], rpm[pw], 1), tr[pw])) / np.sqrt(max(sxx, 1e-9)))
    post = tr >= 0
    if post.sum() < 30:
        raise ValueError("record too short after the step")
    n_tail = max(15, int(0.2 * post.sum()))
    y_final = float(np.mean(rpm[post][-n_tail:]))
    dss = y_final - y0
    if abs(dss) < 5 * noise:
        raise ValueError(f"step response ({dss:+.1f} RPM) is not above the noise ({noise:.1f} RPM): step too small or file not a step")
    sel = tr >= -0.3
    ts, ys = tr[sel], rpm[sel]
    sm = pd.Series(ys).rolling(5, center=True, min_periods=1).mean().to_numpy()
    k63 = np.where(((sm - y0) / dss >= 0.632) & (ts > 0))[0]
    t63 = float(ts[k63[0]]) if len(k63) else 0.15

    def model(p, tt):
        return y0 + p[0] * (1 - np.exp(-p[1] * np.maximum(tt - p[2], 0.0)))

    def resid(p):
        return model(p, ts) - ys

    if two_mode:
        kf, af, ks, as_, th, fitc = two_mode_core(ts, ys, y0, noise, dss, fast_tau)
        res = fitc - ys
        sst = float(np.sum((ys[ts > 0] - np.mean(ys[ts > 0])) ** 2))
        return dict(base_slope=base_slope, base_slope_se=base_slope_se, y0=y0, y_final=y_final, K=kf, A=af, theta=th, K_se=float("nan"), A_se=float("nan"), theta_se=float("nan"),
                    R2=1 - float(res[ts > 0] @ res[ts > 0]) / sst if sst > 0 else float("nan"), rmse=float(np.sqrt(np.mean(res[ts > 0] ** 2))), noise=noise,
                    ts=ts, ys=ys, fit=fitc, t_step=t_step, Ks=ks, As=as_)
    lo_k, hi_k = sorted((0.2 * dss, 3.0 * dss))
    best = None
    for th0 in (0.0, 0.02, 0.04, 0.08):
        a0 = 1.0 / max(t63 - th0, 0.01)
        x0 = [dss, float(np.clip(a0, 1.0, 300.0)), th0]
        try:
            r = least_squares(resid, x0, bounds=([lo_k, 0.5, 0.0], [hi_k, 400.0, 0.3]), loss="soft_l1", f_scale=1.5 * noise)
        except Exception:                                             # noqa: BLE001
            continue
        if best is None or r.cost < best.cost:
            best = r
    if best is None:
        raise ValueError("fit did not converge")
    p = best.x
    res = resid(p)
    dof = max(1, len(ys) - 3)
    s2 = float(res @ res) / dof
    try:
        cov = np.linalg.pinv(best.jac.T @ best.jac) * s2
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except Exception:                                                 # noqa: BLE001
        se = np.full(3, np.nan)
    sst = float(np.sum((ys[ts > 0] - np.mean(ys[ts > 0])) ** 2))
    r2 = 1 - float(res[ts > 0] @ res[ts > 0]) / sst if sst > 0 else np.nan
    return dict(base_slope=base_slope, base_slope_se=base_slope_se, y0=y0, y_final=y_final, K=float(p[0]), A=float(p[1]), theta=float(p[2]), K_se=float(se[0]), A_se=float(se[1]), theta_se=float(se[2]),
                R2=float(r2), rmse=float(np.sqrt(np.mean(res[ts > 0] ** 2))), noise=noise, ts=ts, ys=ys, fit=model(p, ts), t_step=t_step)


def two_mode_core(ts, ys, y0, noise, dss, fast_tau=None):
    """Fit y0 + Kf(1-exp(-Af(t-theta))) + Ks(1-exp(-As(t-theta))): a FAST mode (motor + impeller) and a SLOW mode (the water spinning up).
    Works on the sign-normalised signal so down steps are handled. Returns (Kf, Af, Ks, As, theta) in the original sign and the fitted curve."""
    sgn = 1.0 if dss >= 0 else -1.0
    z = sgn * (ys - y0)
    adss = abs(dss)

    def model(p, tt):
        u = np.maximum(tt - p[4], 0.0)
        return p[0] * (1 - np.exp(-p[1] * u)) + p[2] * (1 - np.exp(-p[3] * u))

    best = None
    for th0 in (0.0, 0.03, 0.06):
        for ks in (0.25, 0.5, 0.75):
            for as0 in (0.25, 1.0):
                af0 = 1.0 / fast_tau if fast_tau else 20.0
                lo_af, hi_af = (af0 * 0.999, af0 * 1.001) if fast_tau else (3.0, 400.0)         # --fast-tau: the fast lag cannot be resolved at 100 Hz, so fix it
                x0 = [adss * (1 - ks), af0, adss * ks, as0, th0]
                try:
                    r = least_squares(lambda p: model(p, ts) - z, x0, bounds=([0.0, lo_af, 0.0, 0.02, 0.0], [1.5 * adss, hi_af, 1.5 * adss, 3.0, 0.3]),
                                      loss="soft_l1", f_scale=1.5 * noise)
                except Exception:                                      # noqa: BLE001
                    continue
                if best is None or r.cost < best.cost:
                    best = r
    if best is None:
        raise ValueError("two-mode fit did not converge")
    kf, af, ks, as_, th = best.x
    if as_ > af:                                                       # keep the labels: fast = larger rate
        kf, af, ks, as_ = ks, as_, kf, af
    p = [kf, af, ks, as_, th]
    return sgn * kf, af, sgn * ks, as_, th, y0 + sgn * model(p, ts)


def startup_duty(path):
    """Duty (%) the motor was started at, from the file name: map_<duty>.csv  or  <label>_r<k>_<base>_<step>.csv (the start-up goes to <base>)."""
    m = re.search(r"(\d+(?:\.\d+)?)(?:_(\d+(?:\.\d+)?))?\.csv$", os.path.basename(path))
    if not m:
        raise ValueError(f"{os.path.basename(path)}: cannot tell the duty from the file name")
    return float(m.group(1))


def fit_startup(t, rpm, until, two_mode=False, fast_tau=None):
    """Fit K(1-exp(-A(t-theta))) to the ramp from rest (speed 0 at t = 0)."""
    sel = (t >= 0) & ((t <= until) if until else (t >= 0))
    ts, ys = t[sel], rpm[sel]
    if len(ts) < 30:
        raise ValueError("record too short")
    n_tail = max(15, int(0.2 * len(ys)))
    y_final = float(np.mean(ys[-n_tail:]))
    noise = max(float(np.std(ys[-n_tail:])), 0.3)
    if y_final < 5 * noise:
        raise ValueError(f"the speed stayed near zero ({y_final:.1f} RPM): duty below the friction dead zone?")
    if two_mode:
        kf, af, ks, as_, th, fitc = two_mode_core(ts, ys, 0.0, noise, y_final, fast_tau)
        res = fitc - ys
        sst = float(np.sum((ys - ys.mean()) ** 2))
        return dict(base_slope=0.0, y0=0.0, y_final=y_final, K=kf, A=af, theta=th, K_se=float("nan"), A_se=float("nan"), theta_se=float("nan"),
                    R2=1 - float(res @ res) / sst if sst > 0 else float("nan"), rmse=float(np.sqrt(np.mean(res ** 2))), noise=noise, ts=ts, ys=ys, fit=fitc, t_step=0.0,
                    Ks=ks, As=as_)
    sm = pd.Series(ys).rolling(5, center=True, min_periods=1).mean().to_numpy()
    k63 = np.where(sm >= 0.632 * y_final)[0]
    t63 = float(ts[k63[0]]) if len(k63) else 0.5

    def model(p, tt):
        return p[0] * (1 - np.exp(-p[1] * np.maximum(tt - p[2], 0.0)))

    best = None
    for th0 in (0.0, 0.02, 0.05, 0.1):
        x0 = [y_final, float(np.clip(1.0 / max(t63 - th0, 0.01), 0.5, 300.0)), th0]
        try:
            r = least_squares(lambda p: model(p, ts) - ys, x0, bounds=([0.5 * y_final, 0.2, 0.0], [2.0 * y_final, 400.0, 0.5]), loss="soft_l1", f_scale=1.5 * noise)
        except Exception:                                             # noqa: BLE001
            continue
        if best is None or r.cost < best.cost:
            best = r
    if best is None:
        raise ValueError("fit did not converge")
    p = best.x
    res = model(p, ts) - ys
    sst = float(np.sum((ys - ys.mean()) ** 2))
    return dict(base_slope=0.0, y0=0.0, y_final=y_final, K=float(p[0]), A=float(p[1]), theta=float(p[2]), K_se=float("nan"), A_se=float("nan"), theta_se=float("nan"),
                R2=1 - float(res @ res) / sst if sst > 0 else float("nan"), rmse=float(np.sqrt(np.mean(res ** 2))), noise=noise, ts=ts, ys=ys, fit=model(p, ts), t_step=0.0)


def legacy_fit_A(t, rpm, t_step):
    """The v1 fit (no dead time) - used only to show how much it biases A."""
    from scipy.optimize import curve_fit
    pre = (t - t_step) < 0
    m = (t - t_step) >= 0
    if pre.sum() < 5 or m.sum() < 20:
        return float("nan")
    base = rpm[pre].mean()
    tt, yy = (t - t_step)[m], rpm[m]
    k0 = float(np.mean(yy[-15:]) - base)
    try:
        with np.errstate(all="ignore"):
            popt, _ = curve_fit(lambda x, K, A: base + K * (1 - np.exp(-A * x)), tt, yy, p0=[k0, 10.0],
                                bounds=([min(0.2 * k0, 3 * k0), 0.1], [max(0.2 * k0, 3 * k0), 500.0]))
        return float(popt[1])
    except Exception:                                                 # noqa: BLE001
        return float("nan")


# ============================================================================================ control maths
def margins(kp, ki, alpha, K, tau, theta):
    """Phase margin (deg), gain margin (dB) and crossover (rad/s) of  C(s) * G(s) * exp(-s*theta) * F(s)
       C = Kp + Ki/s,  G = K/(tau s + 1),  F = alpha / (1 - (1-alpha) exp(-s T))  (the feedback EMA, T = loop period)."""
    s = 1j * OMEGA
    L = (kp + ki / s) * (K / (tau * s + 1)) * np.exp(-s * theta) * (alpha / (1 - (1 - alpha) * np.exp(-s * LOOP_T)))
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    cross = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    if len(cross) == 0:
        return float("nan"), float("nan"), float("nan")
    i = cross[-1]
    pm = 180.0 + np.degrees(ph[i])
    j = np.where((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi))[0]
    gm = -20 * np.log10(mag[j[0]]) if len(j) else float("inf")
    return float(pm), float(gm), float(OMEGA[i])


def margins2(kp, ki, alpha, Kf, tauf, theta, Ks, taus):
    """Margins of the PI loop on the two-mode plant  G = (Kf/(tauf s+1) + Ks/(taus s+1)) * exp(-s theta)  with the feedback filter alpha."""
    s = 1j * OMEGA
    L = (kp + ki / s) * (Kf / (tauf * s + 1) + Ks / (taus * s + 1)) * np.exp(-s * theta) * (alpha / (1 - (1 - alpha) * np.exp(-s * LOOP_T)))
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    cross = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    if len(cross) == 0:
        return float("nan"), float("nan"), float("nan")
    i = cross[-1]
    j = np.where((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi))[0]
    return (-20 * np.log10(mag[j[0]]) if len(j) else float("inf")), 180.0 + np.degrees(ph[i]), float(OMEGA[i])


def imc_design(df, args):
    """IMC-PI (Rivera) for the FAST part of the water plant, checked against the scatter of the measured plants.
       Fast part: K_f e^(-theta s)/(tau_f s + 1); IMC filter time constant lambda -> Kp = (tau_f + theta/2)/(K_f (lambda + theta/2)), Ki = 1/(K_f (lambda + theta/2)).
       The fast lag tau_f cannot be resolved at 100 Hz, so it is FIXED (--fast-tau, default 20 ms); the slow part (water spin-up) is kept in the margin check.
       RECOMMENDED = the fastest lambda whose WORST-CASE gain margin >= --min-gm and phase margin >= --min-pm over the envelope of the measured plants."""
    lines = []
    tauf = args.fast_tau if args.fast_tau else 0.02
    bad = df["R2"].lt(0.4)
    if "warning" in df.columns:
        bad = bad | df["warning"].fillna("").astype(str).str.contains("drifting")
    use = df[~bad]
    lines.append("\nIMC DESIGN FOR THE WATER PLANT (fast part, with the slow water spin-up kept in the stability check)")
    excl = df[bad]
    if len(use) < 2:
        use, excl = df, df.iloc[0:0]
        lines.append("  (fewer than 2 good fits: using all of them, treat the result as a rough guide)")
    for r in excl.itertuples():
        lines.append(f"  excluded: {r.file}  (R2 {r.R2:.2f}{', base level still drifting' if 'drifting' in str(getattr(r, 'warning', '')) else ''})")
    kf, th = use["K_plant"].to_numpy(float), use["delay_s"].to_numpy(float)
    ks, ts = use["K_slow_plant"].to_numpy(float), use["tau_slow_s"].to_numpy(float)
    Kn, thn, Ksn, tsn = float(np.median(kf)), float(np.median(th)), float(np.median(ks)), float(np.median(ts))
    lines.append(f"  {len(use)} usable fits.  Nominal plant: K_fast {Kn:.3f} RPM/count (range {kf.min():.3f}-{kf.max():.3f}), dead time {thn * 1000:.0f} ms, fast lag fixed at {tauf * 1000:.0f} ms,")
    lines.append(f"  slow part K {Ksn:.3f} with tau {tsn:.1f} s (range K {ks.min():.3f}-{ks.max():.3f}, tau {ts.min():.1f}-{ts.max():.1f} s).")
    lines.append(f"  Worst case checked: fast gain {kf.max():.3f}, TOTAL effective delay up to {args.design_delay * 1000:.0f} ms treated as a PURE delay (dead time + fast lag + speed-estimate latency), slow part at both extremes.")
    tau_wc = 0.005                                           # worst case: ALL of the effective delay is a pure delay (more severe than part lag, part delay)
    env = [(Kf, tauf, float(th.min()), Ks_, ts_) for Kf in (kf.min(), kf.max()) for Ks_, ts_ in ((ks.min(), ts.max()), (ks.max(), ts.min()))]
    env += [(Kf, tau_wc, max(args.design_delay - tau_wc, 0.0), Ks_, ts_) for Kf in (kf.min(), kf.max()) for Ks_, ts_ in ((ks.min(), ts.max()), (ks.max(), ts.min()))]
    rows, rec = [], None
    cur = [margins2(args.current_kp, args.current_ki, args.alpha, *c) for c in env]
    rows.append(dict(design="YOUR CURRENT GAINS", lam=float("nan"), Kp=args.current_kp, Ki=args.current_ki, gm_nom=margins2(args.current_kp, args.current_ki, args.alpha, Kn, tauf, thn, Ksn, tsn)[0],
                     pm_nom=margins2(args.current_kp, args.current_ki, args.alpha, Kn, tauf, thn, Ksn, tsn)[1], gm_worst=min(x[0] for x in cur), pm_worst=min(x[1] for x in cur),
                     wc=margins2(args.current_kp, args.current_ki, args.alpha, Kn, tauf, thn, Ksn, tsn)[2]))
    for lam in (0.04, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.15, 0.20, 0.30, 0.50):
        kp = (tauf + thn / 2) / (Kn * (lam + thn / 2))
        ki = 1.0 / (Kn * (lam + thn / 2))
        nom = margins2(kp, ki, args.alpha, Kn, tauf, thn, Ksn, tsn)
        wc = [margins2(kp, ki, args.alpha, *c) for c in env]
        row = dict(design=f"IMC lambda = {lam:.2f} s", lam=lam, Kp=kp, Ki=ki, gm_nom=nom[0], pm_nom=nom[1], gm_worst=min(x[0] for x in wc), pm_worst=min(x[1] for x in wc), wc=nom[2])
        rows.append(row)
        if rec is None and row["gm_worst"] >= args.min_gm and row["pm_worst"] >= args.min_pm:
            rec = row
    lines.append("")
    lines.append(fmt_tab([[r["design"], f"{r['Kp']:.3f}", f"{r['Ki']:.2f}", f"{r['gm_nom']:.1f}", f"{r['pm_nom']:.0f}", f"{r['gm_worst']:.1f}", f"{r['pm_worst']:.0f}", f"{r['wc']:.1f}"] for r in rows],
                         ["design", "Kp", "Ki", "GM dB (nominal)", "PM deg (nominal)", "GM dB (WORST case)", "PM deg (WORST)", "crossover rad/s"]))
    lines.append("")
    if rec is not None:
        lines.append(f">>> RECOMMENDED (fastest IMC design with worst-case gain margin >= {args.min_gm:g} dB and phase margin >= {args.min_pm:g} deg):  "
                     f"Kp = {rec['Kp']:.3f}   Ki = {rec['Ki']:.2f}   (lambda = {rec['lam']:.2f} s, filter alpha = {args.alpha:g})")
    else:
        best = max(rows[1:], key=lambda r: r["gm_worst"])
        lines.append(f">>> No IMC design met the margin targets (worst-case GM >= {args.min_gm:g} dB, PM >= {args.min_pm:g} deg). Most robust tried: Kp = {best['Kp']:.3f}, Ki = {best['Ki']:.2f}.")
    lines.append("    Treat this as a STARTING POINT: confirm it with the closed-loop test (run_matrix.sh) and the tank insert/remove test.")
    return lines, rows


def candidates(K, tau, theta, floor):
    th = max(theta, floor)
    out = [("Your formula  (Kp = 1/K, Ki = Kp*A)", 1.0 / K, 1.0 / K / tau)]
    kp = tau / (K * (tau + th))
    out.append(("IMC with delay, lambda = tau", kp, kp / tau))
    for name, tc in (("SIMC conservative (tau_c = 2*delay)", 2 * th), ("SIMC tight (tau_c = delay)", th)):
        kp = tau / (K * (tc + th))
        ti = min(tau, 4 * (tc + th))
        out.append((name, kp, kp / ti))
    return out


# ============================================================================================ one file
def analyse_file(path, label, args):
    t, rpm, pwm, meta = load_step_csv(path)
    if args.startup:                                                  # the ramp from rest to the base duty
        duty = startup_duty(path)
        fit = fit_startup(t, rpm, args.t_step if args.t_step_given else None, args.two_mode, args.fast_tau)
        counts = duty / 100.0 * ADC
        K_plant, tau = fit["K"] / counts, 1.0 / fit["A"]
        row = dict(label=label, file=os.path.basename(path), direction="start", base_pct=0.0, step_pct=duty, baseline_rpm=0.0, final_rpm=fit["y_final"], delta_rpm=fit["K"],
                   K_plant=K_plant, K_plant_se=float("nan"), pole_A=fit["A"], pole_A_se=float("nan"), tau_s=tau, delay_s=fit["theta"], delay_se=float("nan"),
                   rise_10_90_s=2.197 / fit["A"], settle_2pct_s=fit["theta"] + 4.0 / fit["A"], bandwidth_hz=fit["A"] / (2 * np.pi), R2=fit["R2"], rmse_rpm=fit["rmse"],
                   noise_rpm=fit["noise"], A_legacy_no_delay=float("nan"), warning="", Kp_legacy=1.0 / K_plant, Ki_legacy=fit["A"] / K_plant)
        if "Ks" in fit:
            row.update(K_slow_plant=fit["Ks"] / counts, A_slow=fit["As"], tau_slow_s=1.0 / fit["As"], K_dc_plant=(fit["K"] + fit["Ks"]) / counts)
        row.update(fifo_active=meta["fifo_active"], late_p99_us=meta["late_p99_us"])
        return row, fit, (t, rpm)
    base_pct, step_pct = parse_levels(path, pwm)
    fit = fit_fopdt(t, rpm, args.t_step, args.two_mode, args.fast_tau)
    d_counts = (step_pct - base_pct) / 100.0 * ADC
    if abs(d_counts) < 1:
        raise ValueError("base and step levels are equal")
    K_plant = fit["K"] / d_counts
    tau = 1.0 / fit["A"]
    th_eff = max(fit["theta"], args.min_delay)
    # does the speed move before the assumed step instant? (wrong --t-step)
    early = (t - args.t_step > -0.2) & (t - args.t_step < -0.03)
    warn = ""
    if early.sum() > 3 and abs(np.mean(rpm[early]) - fit["y0"]) > 6 * fit["noise"]:
        warn = "speed already changing before the step instant - check --t-step"
    if abs(fit["base_slope"]) > max(0.005 * abs(fit["y0"]), 1.5) and abs(fit["base_slope"]) > 3.0 * fit.get("base_slope_se", 0.0):
        warn = (warn + "; " if warn else "") + f"the base level was still drifting ({fit['base_slope']:+.1f} RPM/s) before the step: record with a longer hold (the fit will be biased)"
    row = dict(label=label, file=os.path.basename(path), direction="up" if step_pct > base_pct else "down", base_pct=base_pct, step_pct=step_pct,
               baseline_rpm=fit["y0"], final_rpm=fit["y_final"], delta_rpm=fit["K"], K_plant=K_plant, K_plant_se=abs(fit["K_se"] / d_counts),
               pole_A=fit["A"], pole_A_se=fit["A_se"], tau_s=tau, delay_s=fit["theta"], delay_se=fit["theta_se"],
               rise_10_90_s=2.197 / fit["A"], settle_2pct_s=fit["theta"] + 4.0 / fit["A"], bandwidth_hz=fit["A"] / (2 * np.pi),
               R2=fit["R2"], rmse_rpm=fit["rmse"], noise_rpm=fit["noise"], A_legacy_no_delay=legacy_fit_A(t, rpm, args.t_step), warning=warn)
    legacy_kp = 1.0 / K_plant
    row.update(Kp_legacy=legacy_kp, Ki_legacy=legacy_kp * fit["A"])
    if "Ks" in fit:
        row.update(K_slow_plant=fit["Ks"] / d_counts, A_slow=fit["As"], tau_slow_s=1.0 / fit["As"], K_dc_plant=(fit["K"] + fit["Ks"]) / d_counts)
    row.update(fifo_active=meta["fifo_active"], late_p99_us=meta["late_p99_us"])
    return row, fit, (t, rpm)


def plot_file(row, fit, path, outdir):
    fig, (a, b) = plt.subplots(2, 1, figsize=(9, 6.6), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ts = fit["ts"]
    a.scatter(ts, fit["ys"], s=9, color="lightgray", label="measured (100 Hz)")
    a.plot(ts, fit["fit"], "r-", lw=2, label=f"fit: K = {row['delta_rpm']:.1f} RPM,  A = {row['pole_A']:.1f} /s,  delay = {row['delay_s'] * 1000:.0f} ms")
    a.axhline(fit["y0"], color="gray", ls=":", lw=1)
    a.axhline(fit["y_final"], color="gray", ls=":", lw=1)
    a.axvline(0, color="k", lw=0.8)
    a.axvline(row["delay_s"], color="orange", ls="--", label=f"dead time {row['delay_s'] * 1000:.0f} ms")
    a.axvline(row["settle_2pct_s"], color="blue", ls="--", alpha=0.7, label=f"settles (2 %) at {row['settle_2pct_s']:.3f} s")
    a.set_ylabel("Speed (RPM)")
    a.set_title(f"{row['file']}:  {row['base_pct']:g} % -> {row['step_pct']:g} % duty   |   K_plant = {row['K_plant']:.3f} RPM/count,  tau = {row['tau_s'] * 1000:.0f} ms,  R\u00b2 = {row['R2']:.3f}",
                fontsize=10)
    a.legend(fontsize=8.5, loc="lower right")
    a.grid(alpha=0.3)
    b.plot(ts[ts > -0.3], (fit["ys"] - fit["fit"])[ts > -0.3], ".", ms=3, color="gray")
    b.axhline(0, color="k", lw=0.8)
    b.set_ylabel("Residual (RPM)")
    b.set_xlabel("Time since the step (s)")
    b.grid(alpha=0.3)
    a.set_xlim(-0.15, (float(ts.max()) * 1.01) if "tau_slow_s" in row else max(0.5, row["delay_s"] + 10.0 / row["pole_A"]))      # zoom on the transient (whole record in two-mode)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "plots", os.path.splitext(os.path.basename(path))[0] + ".png"), dpi=170)
    plt.close(fig)


# ============================================================================================ reporting
def fmt_tab(rows, header):
    w = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(header)]
    out = ["  ".join(str(h).rjust(w[i]) for i, h in enumerate(header)), "  ".join("-" * x for x in w)]
    out += ["  ".join(str(c).rjust(w[i]) for i, c in enumerate(r)) for r in rows]
    return "\n".join(out)


def verdict(Kp_ratio_K, A_ratio):
    msgs = []
    if 0.85 <= Kp_ratio_K <= 1.15 and 0.8 <= A_ratio <= 1.2:
        msgs.append("The plant matches the one your current gains were designed for (K within 15 %, A within 20 %): the current gains still fit this condition.")
    else:
        if Kp_ratio_K < 0.85:
            msgs.append(f"Plant gain is {Kp_ratio_K:.2f}x the design value: the same Kp now gives LESS loop gain (slower, calmer PWM, more phase margin).")
        elif Kp_ratio_K > 1.15:
            msgs.append(f"Plant gain is {Kp_ratio_K:.2f}x the design value: the same Kp now gives MORE loop gain (more aggressive, more PWM jitter, less margin).")
        if A_ratio < 0.8:
            msgs.append(f"The plant is slower than designed (A is {A_ratio:.2f}x): the current Ki/Kp zero no longer lines up with the pole.")
        elif A_ratio > 1.2:
            msgs.append(f"The plant is faster than designed (A is {A_ratio:.2f}x).")
    return msgs


def report_label(label, rows, args, outdir, lines):
    df = pd.DataFrame(rows)
    med = df[["K_plant", "pole_A", "delay_s", "R2"]].median()
    K, A, th = float(med.K_plant), float(med.pole_A), float(med.delay_s)
    tau = 1.0 / A
    lines.append(f"\n=== {label}:  {len(df)} step file(s) ===")
    if "fifo_active" in df.columns:
        known = df.fifo_active.dropna()
        if len(known):
            n_on = int(known.astype(bool).sum())
            late = df.late_p99_us.dropna()
            lines.append(f"Real-time scheduling (from the daemon's own record): SCHED_FIFO active in {n_on} of {len(known)} file(s)"
                         + (f"; 99th-percentile loop lateness {late.max():.0f} us (the loop period is 10000 us)" if len(late) else "")
                         + ("" if n_on == len(known) else "   <- some runs had NO real-time priority: timing and speed estimates are noisier"))
        else:
            lines.append("Real-time scheduling: these files do not record it (the FIFO column in tuning_table.csv is then only a guess from the file name).")
    tab = [[r.file[:34], r.direction, f"{r.base_pct:g}->{r.step_pct:g}", f"{r.baseline_rpm:.0f}->{r.final_rpm:.0f}", f"{r.K_plant:.3f}", f"{r.pole_A:.1f}",
            f"{r.delay_s * 1000:.0f}", f"{r.R2:.3f}", f"{r.A_legacy_no_delay:.1f}"] for r in df.itertuples()]
    lines.append(fmt_tab(tab, ["file", "dir", "duty %", "RPM", "K_plant", "A (1/s)", "delay ms", "R2", "A if no delay"]))
    bad = df[df.warning != ""]
    for r in bad.itertuples():
        lines.append(f"  WARNING {r.file}: {r.warning}")
    if len(df) >= 2:
        spread = (df.K_plant.max() - df.K_plant.min()) / abs(med.K_plant) * 100
        lines.append(f"\nK_plant spread between files: {spread:.0f} %" + ("   <- large: the motor is not linear over these steps, or the runs are not repeatable" if spread > 25 else ""))
    both = df.groupby("direction")[["K_plant", "pole_A"]].median()
    if len(both) == 2 and abs(both.K_plant["up"] - both.K_plant["down"]) / abs(med.K_plant) > 0.2:
        lines.append(f"Up steps K = {both.K_plant['up']:.3f}, down steps K = {both.K_plant['down']:.3f}: friction / dead-zone makes the plant direction dependent.")
    if (df.direction == "start").all():
        lines.append("\nNOTE: these are fits of the START-UP from rest (a large step from 0 duty). K_plant here is the average gain from 0, lower than the small-signal gain around an operating point\n"
                     "      because friction uses up about 36 RPM (the dead zone). Tune from the small-signal STEP files; use this for the start-up delay / time constant and as a cross-check.")
    if "K_slow_plant" in df.columns:
        lines.append("\nTWO-MODE FIT: a FAST part (motor + impeller, used for the PI design) and a SLOW part (the water spinning up, which the integrator treats as a slow disturbance).")
        tab2 = [[r.file[:30], f"{r.K_plant:.3f}", f"{r.pole_A:.1f}", f"{r.delay_s * 1000:.0f}", f"{r.K_slow_plant:.3f}", f"{r.tau_slow_s:.1f}",
                 f"{r.K_dc_plant:.3f}", f"{r.K_plant / r.K_dc_plant * 100:.0f} %", f"{r.R2:.3f}"] for r in df.itertuples()]
        lines.append(fmt_tab(tab2, ["file", "K fast", "A fast", "delay ms", "K slow", "tau slow s", "K total (DC)", "fast share", "R2"]))
    lines.append(f"\nMedian plant:  K_plant = {K:.4f} RPM/count    A = {A:.1f} /s (tau = {tau * 1000:.0f} ms)    dead time = {th * 1000:.0f} ms")
    des_K, des_A = 1.0 / args.current_kp, args.current_ki / args.current_kp
    lines.append(f"Design plant implied by your current gains (Kp {args.current_kp:g}, Ki {args.current_ki:g}):  K_plant = {des_K:.4f}    A = {des_A:.1f} /s")
    lines.append(f"Ratio measured / design:  K x{K / des_K:.2f}    A x{A / des_A:.2f}")
    for m in verdict(K / des_K, A / des_A):
        lines.append("  -> " + m)
    bias = float(np.nanmedian(df.A_legacy_no_delay)) / A if df.A_legacy_no_delay.notna().any() else float("nan")
    if np.isfinite(bias) and abs(bias - 1) > 0.08:
        lines.append(f"The old no-dead-time fit would have read A about {abs(1 - bias) * 100:.0f} % {'low' if bias < 1 else 'high'} on this data.")
    if "K_slow_plant" in df.columns:
        imc_lines, imc_rows = imc_design(df, args)
        lines += imc_lines
        pd.DataFrame(imc_rows).to_csv(os.path.join(outdir, f"imc_gains_{re.sub(r'[^A-Za-z0-9]+', '_', label)}.csv"), index=False)
        lines.append("\n(The single-pole 'Your formula / IMC / SIMC' rows are not shown in two-mode: they depend on the fast pole, which 100 Hz data cannot resolve.)")
        return dict(label=label, K=K, A=A, theta=th, tau=tau, df=df)
    th_eff = max(th, args.min_delay)
    cur = margins(args.current_kp, args.current_ki, args.alpha, K, tau, th_eff)
    lines.append(f"\nGAINS ON THIS PLANT   (feedback filter alpha = {args.alpha:g}; effective delay {th_eff * 1000:.0f} ms)")
    rows_c = [["YOUR CURRENT GAINS", f"{args.current_kp:.4f}", f"{args.current_ki:.2f}", f"{cur[0]:.0f}", f"{cur[1]:.1f}", f"{cur[2]:.1f}"]]
    for name, kp, ki in candidates(K, tau, th, args.min_delay):
        pm, gm, wc = margins(kp, ki, args.alpha, K, tau, th_eff)
        rows_c.append([name, f"{kp:.4f}", f"{ki:.2f}", f"{pm:.0f}", f"{gm:.1f}", f"{wc:.1f}"])
    lines.append(fmt_tab(rows_c, ["design", "Kp", "Ki", "phase margin deg", "gain margin dB", "crossover rad/s"]))
    lines.append("Rule of thumb: phase margin >= 45 deg is calm, 30-45 deg overshoots a little, < 30 deg rings. Gain margin >= 6 dB means the plant gain could double before instability.")
    return dict(label=label, K=K, A=A, theta=th, tau=tau, df=df)


def margin_plot(info, args, outdir):
    K, tau, th = info["K"], info["tau"], max(info["theta"], args.min_delay)
    kps = np.logspace(np.log10(0.15), np.log10(3.5), 70)
    fig, axs = plt.subplots(1, 2, figsize=(12.5, 4.8), sharey=True)
    for ax, mode in zip(axs, ("tied", "fixed")):
        for al, col in ((1.0, "#1E8449"), (0.5, "#E08A00"), (0.3, "#C0392B")):
            pm = [margins(kp, (kp / tau if mode == "tied" else args.current_ki), al, K, tau, th)[0] for kp in kps]
            ax.plot(kps, pm, color=col, lw=2, label=f"alpha = {al:g}")
        ax.axhline(45, color="gray", ls="--", lw=1)
        ax.axhline(30, color="gray", ls=":", lw=1)
        pm0 = margins(args.current_kp, args.current_ki, args.alpha, K, tau, th)[0]
        if mode == "fixed":
            ax.plot([args.current_kp], [pm0], "k*", ms=14, label="your current gains")
        ax.set_xscale("log")
        ax.set_xticks([0.2, 0.5, 1, 2, 3])
        ax.set_xticklabels(["0.2", "0.5", "1", "2", "3"])
        ax.set_xlabel("Kp")
        ax.set_title("(a) Ki scaled with Kp (Ki = Kp*A)" if mode == "tied" else f"(b) Ki fixed at {args.current_ki:g}", fontsize=11)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=9)
    axs[0].set_ylabel("Predicted phase margin (deg)")
    fig.suptitle(f"{info['label']}: predicted phase margin on the measured plant (K = {K:.3f}, tau = {tau * 1000:.0f} ms, delay = {th * 1000:.0f} ms).  "
                 f"Dashed 45 deg = calm, dotted 30 deg = ringing", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"margins_{re.sub(r'[^A-Za-z0-9]+', '_', info['label'])}.png"), dpi=170)
    plt.close(fig)


def evaluate_folder(folder, info, args, outdir, lines):
    raw = os.path.join(folder, "results_raw.txt")
    if not os.path.exists(raw):
        lines.append(f"\n(--evaluate-folder: {raw} not found)")
        return
    runs = {}
    for ln in open(raw):
        p = ln.split()
        if len(p) < 3 or "=" not in p[1]:
            continue
        kv = {k: float(v) for k, v in (x.split("=") for x in p[1:])}
        if kv.get("aborted", 0) > 0:
            continue
        runs.setdefault((round(kv["kp"], 3), round(kv["ki"], 2), round(kv["alpha"], 3)), []).append(kv)
    K, tau, th = info["K"], info["tau"], max(info["theta"], args.min_delay)
    rows = []
    for (kp, ki, al), rs in runs.items():
        pm, gm, wc = margins(kp, ki, al, K, tau, th)
        rows.append((kp, ki, al, pm, gm, float(np.mean([r["overshoot_pct"] for r in rs])), float(np.mean([r["settle5_s"] for r in rs])), float(np.mean([r["pwm_jitter_pct"] for r in rs]))))
    df = pd.DataFrame(rows, columns=["kp", "ki", "alpha", "pm", "gm", "overshoot", "settle", "jitter"]).dropna(subset=["pm"])
    if len(df) < 4:
        lines.append("\n(--evaluate-folder: not enough settings to compare)")
        return
    from scipy.stats import spearmanr
    rho = spearmanr(df.pm, df.overshoot)[0]
    lines.append(f"\n=== MODEL CHECK on {folder}: predicted phase margin vs measured overshoot ({len(df)} settings) ===")
    lines.append(f"Spearman rank correlation = {rho:+.2f}   (-1 = the model ranks every setting's overshoot perfectly; 0 = no predictive value)")
    low = df.sort_values("pm").head(8)
    hi = df.sort_values("pm", ascending=False).head(8)
    lines.append("Lowest predicted margin:  " + ",  ".join(f"Kp {r.kp:g}/a {r.alpha:g}: {r.pm:.0f} deg ({r.overshoot:.0f} % ovr)" for r in low.itertuples()))
    lines.append("Highest predicted margin: " + ",  ".join(f"Kp {r.kp:g}/Ki {r.ki:g}/a {r.alpha:g}: {r.pm:.0f} deg ({r.overshoot:.1f} % ovr)" for r in hi.itertuples()))
    df.to_csv(os.path.join(outdir, "model_check.csv"), index=False)
    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    sc = ax.scatter(df.pm, df.overshoot, c=df.alpha, cmap="viridis", s=45, edgecolor="k", lw=0.4)
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("filter alpha")
    ax.set_xlabel("Predicted phase margin (deg)")
    ax.set_ylabel("Measured startup overshoot (%)")
    ax.set_title(f"Does the measured plant predict the overshoot?  rho = {rho:+.2f}", fontsize=11)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "model_check.png"), dpi=170)
    plt.close(fig)


# ============================================================================================ main
def main():
    ap = argparse.ArgumentParser(description="MIXR-1 step-response analyser and PI tuning check")
    ap.add_argument("inputs", nargs="*", help="files / patterns, optionally label=pattern")
    ap.add_argument("--out", default="step_analysis")
    ap.add_argument("--t-step", type=float, default=None, help="time of the PWM step inside the file (s), default 1.0; with --startup: end of the start-up window")
    ap.add_argument("--two-mode", action="store_true", help="fit a FAST and a SLOW exponential (use when the speed keeps creeping for seconds after the quick response, e.g. water spinning up)")
    ap.add_argument("--fast-tau", type=float, default=None, help="with --two-mode: fix the FAST lag (s) instead of fitting it (the 100 Hz data cannot resolve it); e.g. 0.02. Makes the other numbers far more stable")
    ap.add_argument("--design-delay", type=float, default=0.06, help="worst-case TOTAL effective delay (s) to keep stable at (dead time + fast lag + speed-estimate latency); default 0.06")
    ap.add_argument("--min-gm", type=float, default=6.0, help="gain margin (dB) the recommended IMC design must keep in the worst case")
    ap.add_argument("--min-pm", type=float, default=45.0, help="phase margin (deg) the recommended IMC design must keep in the worst case")
    ap.add_argument("--from-table", default=None, metavar="tuning_table.csv", help="skip the fitting: run only the IMC design from an existing tuning_table.csv made with --two-mode")
    ap.add_argument("--startup", action="store_true", help="fit the RAMP FROM REST at the start of each file (map_*.csv or run_steps files) instead of the step")
    ap.add_argument("--alpha", type=float, default=1.0, help="feedback filter alpha assumed for the margin predictions")
    ap.add_argument("--current-kp", type=float, default=DEF_KP)
    ap.add_argument("--current-ki", type=float, default=DEF_KI)
    ap.add_argument("--min-delay", type=float, default=0.01, help="lower bound on the delay used for tuning (s): the 10 ms loop itself")
    ap.add_argument("--evaluate-folder", default=None, help="a run_matrix.sh folder: predict phase margin for every tested setting and compare with its overshoot")
    ap.add_argument("--pick", action="store_true", help="choose one file interactively (old behaviour)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    args.t_step_given = args.t_step is not None
    if args.t_step is None:
        args.t_step = 1.0

    if args.from_table:
        tbl = pd.read_csv(args.from_table)
        need = ["K_plant", "delay_s", "K_slow_plant", "tau_slow_s", "R2", "file"]
        miss = [c for c in need if c not in tbl.columns]
        if miss:
            sys.exit(f"{args.from_table} needs a table made with --two-mode (missing columns {miss})")
        os.makedirs(args.out, exist_ok=True)
        lines, rows = imc_design(tbl, args)
        text = "\n".join(lines)
        print(text)
        pd.DataFrame(rows).to_csv(os.path.join(args.out, "imc_gains.csv"), index=False)
        open(os.path.join(args.out, "imc_design.txt"), "w").write(text + "\n")
        print(f"\n[done] IMC design written to '{args.out}/'")
        return
    groups = {}
    tokens = args.inputs or ["water_raw_fifo_*.csv"]
    for tok in tokens:
        label, pat = tok.split("=", 1) if "=" in tok else ("data", tok)
        files = sorted(glob.glob(pat)) if any(ch in pat for ch in "*?[") else [pat]
        files = [f for f in files if os.path.isfile(f)]
        groups.setdefault(label, []).extend(files)
    if not any(groups.values()):
        sys.exit(f"No step files found for {tokens}. Pass files or a pattern, e.g.  python3 analyze_step.py \"water_raw_fifo_*.csv\"")
    if args.pick:
        allf = [(l, f) for l, fs in groups.items() for f in fs]
        for i, (l, f) in enumerate(allf):
            print(f"[{i + 1}] {f}")
        try:
            sel = allf[int(input("\nEnter file number: ")) - 1]
        except (ValueError, IndexError):
            sys.exit("Invalid selection.")
        groups = {sel[0]: [sel[1]]}

    os.makedirs(os.path.join(args.out, "plots"), exist_ok=True)
    all_rows, infos, lines = [], [], []
    for label, files in groups.items():
        rows = []
        for f in files:
            try:
                row, fit, _ = analyse_file(f, label, args)
            except (ValueError, KeyError) as e:
                print(f"  skip {os.path.basename(f)}: {e}")
                continue
            rows.append(row)
            if not args.no_plots:
                plot_file(row, fit, f, args.out)
        if rows:
            all_rows += rows
            infos.append(report_label(label, rows, args, args.out, lines))
    if not all_rows:
        sys.exit("No usable step files.")

    # side-by-side comparison of conditions
    if len(infos) >= 2:
        lines.append("\n=== CONDITIONS SIDE BY SIDE (median of each) ===")
        base = infos[0]
        rows_c = [[i["label"], f"{i['K']:.3f}", f"x{i['K'] / base['K']:.2f}", f"{i['A']:.1f}", f"x{i['A'] / base['A']:.2f}", f"{i['theta'] * 1000:.0f}",
                   f"{1.0 / i['K']:.3f}", f"{1.0 / i['K'] / i['tau']:.1f}"] for i in infos]
        lines.append(fmt_tab(rows_c, ["condition", "K_plant", f"vs {base['label']}", "A (1/s)", f"vs {base['label']}", "delay ms", "Kp (your formula)", "Ki (your formula)"]))
    for info in infos:
        if not args.no_plots:
            margin_plot(info, args, args.out)
    if args.evaluate_folder and infos:
        evaluate_folder(args.evaluate_folder, infos[0], args, args.out, lines)

    cols_old = ["Test_Region", "Mode", "Controller", "FIFO_Enabled", "Baseline_PWM_pct", "Step_PWM_pct", "Baseline_RPM", "Pole_A", "Rise_Time_s", "Settling_Time_s", "Bandwidth_Hz", "Kp", "Ki"]
    df = pd.DataFrame(all_rows)
    out = pd.DataFrame({"Test_Region": df.base_pct.map(lambda x: f"{x:g}%") + " to " + df.step_pct.map(lambda x: f"{x:g}%"), "Mode": df.label, "Controller": "OpenLoop",
                        "FIFO_Enabled": [(int(f) if f is not None and f == f else int("fifo" in n.lower())) for f, n in zip(df.fifo_active, df.file)], "Baseline_PWM_pct": df.base_pct, "Step_PWM_pct": df.step_pct,
                        "Baseline_RPM": df.baseline_rpm.round(2), "Pole_A": df.pole_A.round(4), "Rise_Time_s": df.rise_10_90_s.round(4), "Settling_Time_s": df.settle_2pct_s.round(4),
                        "Bandwidth_Hz": df.bandwidth_hz.round(4), "Kp": df.Kp_legacy.round(4), "Ki": df.Ki_legacy.round(4)})
    out = pd.concat([out, df.drop(columns=["label", "base_pct", "step_pct", "baseline_rpm", "pole_A", "rise_10_90_s", "settle_2pct_s", "bandwidth_hz", "Kp_legacy", "Ki_legacy"]).round(5)], axis=1)
    out.to_csv(os.path.join(args.out, "tuning_table.csv"), index=False)
    text = "\n".join(lines)
    open(os.path.join(args.out, "recommended_gains.txt"), "w").write(text + "\n")
    print(text)
    print(f"\n[done] {len(df)} file(s). Tables, plots and recommended_gains.txt are in '{args.out}/'")


if __name__ == "__main__":
    main()
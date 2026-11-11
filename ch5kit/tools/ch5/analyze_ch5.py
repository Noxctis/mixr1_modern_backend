#!/usr/bin/env python3
"""Chapter 5 data analysis for everything that is NOT the PPR study (that one is Tools/ppr/analyze_ppr_study.py).

  python3 analyze_ch5.py duty            --csv up.csv down.csv --labels up,down --hold 12 [--window 4,11]      TC-U6  duty vs rpm, breakaway, hysteresis
  python3 analyze_ch5.py steps           --csv run1.csv run2.csv ... [--band 0.05]                             TC-S1  rise, overshoot, settling, steady-state error
  python3 analyze_ch5.py torque-drift    --csv torque_cold.csv [--window-n 40]                                 TC-U3 + Table 5.5 tare thresholds
  python3 analyze_ch5.py torque-validate --csv loads.csv --u-mass 0.0005 --u-arm 0.0002 [--g 9.782]            TC-U4  Table 5.6, Figure 5.9
  python3 analyze_ch5.py power-unc       --speeds 100,500,1000 --torques 0.01,0.05,0.12 ...                    Eq. 5.3 uncertainty, dominant source
  python3 analyze_ch5.py endurance       --csv long.csv [--platform plat.csv] [--stdout run.txt]               TC-S2 / TC-S3
  python3 analyze_ch5.py integrity       --daemon-csv d.csv --dash-csv dash.csv [--mat dash.mat]               TC-S4 / TC-S6
  python3 analyze_ch5.py plausibility    --summary seq_summary.csv --lit rushton.csv [--label "..."]          TC-S5
  python3 analyze_ch5.py rawfilt         --csv run.csv --t0 20 --t1 30                                         Figure 5.3
  python3 analyze_ch5.py disturbance     --csv run.csv --events run.events.csv --t0-unix <t0_unix from the run's stdout>   load-disturbance recovery (TC-S1)
  python3 analyze_ch5.py torque-mean     --csv load1.csv load2.csv ... [--skip 3]                              mean torque per reference load -> loads.csv
  python3 analyze_ch5.py mark            --out run.events.csv                                                  operator event marker (load disturbance, baffles, ...)
  python3 analyze_ch5.py platform-log    --out plat.csv [--period 10]                                          temperature / throttling / clock logger (run beside long tests)
  python3 analyze_ch5.py selftest                                                                              synthetic data through every command

Daemon test CSVs (columns elapsed_s, raw_rpm, filtered_rpm, target_rpm, pwm, power_w, torque_nm, loop_period_us, late_us, ...) come from
`mixr1_daemon --test`. Output goes to --out (default ch5_results): CSV tables, LaTeX tables in the thesis format, PDF + PNG figures.
Nothing here invents data: where a literature curve or a reference load is needed you supply it.
"""
import argparse, math, os, re, subprocess, sys, time, warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
warnings.filterwarnings("ignore")
plt.rcParams.update({"font.family": "serif", "font.size": 10, "axes.labelsize": 11, "savefig.dpi": 300, "savefig.bbox": "tight", "axes.grid": True,
                     "grid.linestyle": ":", "grid.alpha": 0.6, "figure.constrained_layout.use": True})
G_STD = 9.80665


def outdir(a):
    os.makedirs(a.out, exist_ok=True); os.makedirs(os.path.join(a.out, "latex"), exist_ok=True); return a.out


def save(fig, a, name):
    fig.savefig(os.path.join(outdir(a), name + ".pdf")); fig.savefig(os.path.join(outdir(a), name + ".png")); plt.close(fig)


def f(x, nd=2):
    return "--" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"


def tex(df, a, name, caption, label):
    cols = list(df.columns)
    L = ["\\begin{table}[htbp]", "    \\centering", f"    \\caption{{{caption}}}", f"    \\label{{{label}}}", "    \\small",
         "    \\begin{tabular}{@{}" + "r" * len(cols) + "@{}}", "        \\toprule", "        " + " & ".join(cols) + " \\\\", "        \\midrule"]
    L += ["        " + " & ".join(str(v) for v in r.values) + " \\\\" for _, r in df.iterrows()]
    L += ["        \\bottomrule", "    \\end{tabular}", "\\end{table}", ""]
    open(os.path.join(outdir(a), "latex", name + ".tex"), "w").write("\n".join(L))


def window_of(arg):
    lo, hi = [float(x) for x in arg.split(",")]; return lo, hi


# ============================================================== TC-U6
def cmd_duty(a):
    labels = a.labels.split(",") if a.labels else [os.path.basename(p) for p in a.csv]
    lo, hi = window_of(a.window or f"{0.35 * a.hold},{0.95 * a.hold}")
    rows = []
    for p, lab in zip(a.csv, labels):
        df = pd.read_csv(p)
        for s, g in df.groupby("step_index"):
            if s < 0:
                continue
            w = g[(g["elapsed_s"] - s * a.hold >= lo) & (g["elapsed_s"] - s * a.hold < hi)]
            if len(w) < 20:
                continue
            rows.append(dict(run=lab, step=int(s), duty_pct=100 * w["pwm"].mean() / 4095.0, rpm=w["raw_rpm"].mean(), rpm_sd=w["raw_rpm"].std(ddof=1),
                             power_w=w["power_w"].mean() if "power_w" in w and w["power_w"].notna().any() else np.nan,
                             current_ma=w["current_ma"].mean() if "current_ma" in w else np.nan, bus_v=w["bus_v"].mean() if "bus_v" in w else np.nan))
    R = pd.DataFrame(rows); R.to_csv(os.path.join(outdir(a), "duty_sweep.csv"), index=False)
    bk = []
    for lab, g in R.groupby("run", sort=False):
        g = g.sort_values("step")
        moving = g[g["rpm"] >= a.moving_rpm]
        bk.append(dict(run=lab, first_moving_duty_pct=moving["duty_pct"].min() if len(moving) else np.nan,
                       max_rpm=g["rpm"].max(), max_duty_pct=g["duty_pct"].max(), max_power_w=g["power_w"].max()))
    B = pd.DataFrame(bk); B.to_csv(os.path.join(outdir(a), "breakaway.csv"), index=False)
    print(B.to_string(index=False))
    fig, ax = plt.subplots(1, 2, figsize=(10.5, 4))
    for lab, g in R.groupby("run", sort=False):
        ax[0].errorbar(g["duty_pct"], g["rpm"], yerr=g["rpm_sd"], fmt="o-", ms=3.5, capsize=2, label=lab)
        ax[1].plot(g["duty_pct"], g["power_w"], "o-", ms=3.5, label=lab)
    ax[0].axhline(a.moving_rpm, color="k", ls=":", lw=0.8)
    ax[0].set(xlabel="Duty (%)", ylabel="Steady-state speed (rpm)", title="Open-loop duty against speed"); ax[1].set(xlabel="Duty (%)", ylabel="Electrical power (W)", title="Power against duty")
    ax[0].legend(); save(fig, a, "fig_duty_vs_rpm")
    t = B.rename(columns={"run": "Run", "first_moving_duty_pct": "Breakaway duty (\\%)", "max_rpm": "Top speed (rpm)", "max_duty_pct": "Top duty (\\%)", "max_power_w": "Peak power (W)"})
    for c in t.columns[1:]:
        t[c] = t[c].map(lambda v: f(v, 1))
    tex(t, a, "tab_duty_breakaway", "Open-loop deadband and top speed from the duty sweep with the vessel at working volume.", "tab:duty-breakaway")


# ============================================================== TC-S1
def step_metrics(df, band=0.05, col="filtered_rpm"):
    t, tgt, y = df["elapsed_s"].to_numpy(), df["target_rpm"].to_numpy(), df[col].to_numpy()
    edges = [0] + [i for i in range(1, len(tgt)) if abs(tgt[i] - tgt[i - 1]) > 0.5] + [len(tgt)]
    out = []
    for s, e in zip(edges[:-1], edges[1:]):
        T = tgt[s]
        if T <= 0 or e - s < 100:
            continue
        pre = y[max(0, s - 50):s]
        y0 = float(pre.mean()) if s > 0 and len(pre) else 0.0
        tail = max(50, int(0.3 * (e - s))); yf = float(y[e - tail:e].mean())
        d = yf - y0
        if abs(d) < 1e-6:
            continue
        sg = np.sign(d); seg = y[s:e]; ts = t[s:e] - t[s]
        def cross(frac):
            idx = np.where(sg * (seg - y0) >= frac * abs(d))[0]; return ts[idx[0]] if len(idx) else np.nan
        r10, r90 = cross(0.1), cross(0.9)
        peak = seg.max() if sg > 0 else seg.min()
        os_ = max(0.0, sg * (peak - yf) / abs(d) * 100)
        out_band = np.where(np.abs(seg - yf) > max(band * abs(d), 1.0))[0]
        settle = ts[out_band[-1]] if len(out_band) else 0.0
        if len(out_band) and out_band[-1] >= len(seg) - 2:
            settle = np.nan
        raw_sd = float(df["raw_rpm"].to_numpy()[e - tail:e].std(ddof=1))
        out.append(dict(t_start_s=t[s], from_rpm=y0, target_rpm=T, final_rpm=yf, sse_rpm=T - yf, sse_pct=100 * (T - yf) / T, rise10_90_s=r90 - r10,
                        overshoot_pct=os_, settle_s=settle, raw_sd_rpm=raw_sd))
    return pd.DataFrame(out)


def cmd_steps(a):
    allm = []
    fig, ax = plt.subplots(len(a.csv), 1, figsize=(8, 2.8 * len(a.csv)), squeeze=False)
    for i, p in enumerate(a.csv):
        df = pd.read_csv(p); m = step_metrics(df, a.band); m.insert(0, "run", os.path.basename(p)); allm.append(m)
        ax[i, 0].plot(df["elapsed_s"], df["target_rpm"], "k--", lw=1, label="setpoint"); ax[i, 0].plot(df["elapsed_s"], df["raw_rpm"], color="gray", lw=0.6, alpha=0.6, label="raw")
        ax[i, 0].plot(df["elapsed_s"], df["filtered_rpm"], lw=1.2, label="filtered"); ax[i, 0].set(ylabel="rpm", title=os.path.basename(p), xlabel="Time (s)" if i == len(a.csv) - 1 else "")
        if "power_w" in df and df["power_w"].notna().any():
            a2 = ax[i, 0].twinx(); a2.plot(df["elapsed_s"], df["power_w"], color="C3", lw=0.7, alpha=0.6); a2.set_ylabel("Power (W)", color="C3"); a2.grid(False)
    ax[0, 0].legend(fontsize=8); save(fig, a, "fig_step_responses")
    M = pd.concat(allm, ignore_index=True); M.to_csv(os.path.join(outdir(a), "step_metrics.csv"), index=False); print(M.round(3).to_string(index=False))
    t = pd.DataFrame({"Run": M["run"].str.slice(-24), "From (rpm)": M["from_rpm"].map(lambda v: f(v, 0)), "Setpoint (rpm)": M["target_rpm"].map(lambda v: f(v, 0)),
                      "Rise 10--90 (s)": M["rise10_90_s"].map(lambda v: f(v, 2)), "Overshoot (\\%)": M["overshoot_pct"].map(lambda v: f(v, 1)),
                      f"Settling {int(a.band * 100)}\\% (s)": M["settle_s"].map(lambda v: f(v, 2)), "Steady-state error (\\%)": M["sse_pct"].map(lambda v: f(v, 2))})
    tex(t, a, "tab_step_response", "Closed-loop step response: rise time, overshoot, settling time and steady-state error (filtered speed; final value from the last 30 % of each segment).", "tab:step-response")


# ============================================================== TC-U3 / Table 5.5
def cmd_torque_drift(a):
    df = pd.read_csv(a.csv)
    tc = next(c for c in df.columns if "time" in c.lower() or c.lower().startswith("elapsed")); yc = next(c for c in df.columns if "torque" in c.lower())
    if "ok" in df: df = df[df["ok"] == 1]
    df = df.dropna(subset=[yc]); t = df[tc].to_numpy(float); y = df[yc].to_numpy(float); rate = 1.0 / np.median(np.diff(t))
    n = int(len(y) * a.steady_frac); ys = y[-n:]; ts = t[-n:]
    slope = np.polyfit(ts / 3600.0, ys, 1)[0]
    early = y[t < t[0] + 300].mean(); late = ys.mean()
    tau = np.nan
    try:
        from scipy.optimize import curve_fit
        fn = lambda x, a0, A, tt: a0 + A * np.exp(-x / tt)
        p, _ = curve_fit(fn, t - t[0], y, p0=[late, early - late, 600.0], maxfev=20000); tau = p[2]
    except Exception:
        pass
    N = a.window_n; k = len(ys) // N; W = ys[:k * N].reshape(k, N)
    wstd = W.std(axis=1, ddof=1); wdrift = np.abs(W[:, N // 2:].mean(axis=1) - W[:, :N // 2].mean(axis=1)); wmean = W.mean(axis=1)
    res = dict(duration_h=(t[-1] - t[0]) / 3600, rate_hz=rate, first5min_mean=early, steady_mean=late, warmup_change=early - late, warmup_tau_s=tau,
               steady_sd=ys.std(ddof=1), steady_p2p=np.ptp(ys), steady_slope_per_h=slope)
    for nm, v in res.items(): print(f"{nm:>20s}: {v:.6g}")
    cfg = dict(max_std=0.005, max_drift=0.005, max_abs=0.05)         # the defaults in torque_autotare.hpp Settings
    rows = [dict(Parameter="window (readings)", Configured=str(N), Measured="window = " + f"{N / rate:.1f} s", Basis=f"{rate:.1f} Hz sampling"),
            dict(Parameter="max\\_std (N\\,m)", Configured=f(cfg["max_std"], 4), Measured=f"median {f(np.median(wstd), 4)}, p99 {f(np.percentile(wstd, 99), 4)}, max {f(wstd.max(), 4)}",
                 Basis=f"suggest 3x median = {f(3 * np.median(wstd), 4)}, never below p99"),
            dict(Parameter="max\\_drift (N\\,m)", Configured=f(cfg["max_drift"], 4), Measured=f"median {f(np.median(wdrift), 4)}, p99 {f(np.percentile(wdrift, 99), 4)}, max {f(wdrift.max(), 4)}",
                 Basis=f"suggest above p99 = {f(np.percentile(wdrift, 99) * 1.5, 4)}"),
            dict(Parameter="max\\_abs (N\\,m)", Configured=f(cfg["max_abs"], 3), Measured=f"warm-up offset {f(abs(res['warmup_change']), 4)}, steady offset {f(abs(late), 4)}",
                 Basis="must exceed the offset the rig really has at rest")]
    T = pd.DataFrame(rows); T.to_csv(os.path.join(outdir(a), "tare_thresholds.csv"), index=False); tex(T, a, "tab_tare_thresholds", "Automatic-tare acceptance thresholds as configured, against the idle noise and drift measured over the unloaded cold-start log.", "tab:tare-thresholds")
    pd.Series(res).to_csv(os.path.join(outdir(a), "torque_drift_summary.csv"), header=["value"])
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].plot((t - t[0]) / 60, y, lw=0.5); ax[0].set(xlabel="Time since start (min)", ylabel="Unloaded torque reading (N$\\cdot$m)", title="Cold-start drift")
    ax[1].hist(wstd, bins=30); ax[1].axvline(cfg["max_std"], color="r", ls="--", label="configured max\\_std"); ax[1].set(xlabel=f"SD of {N}-reading windows (N$\\cdot$m)", ylabel="Windows", title="Idle noise at the tare window"); ax[1].legend()
    save(fig, a, "fig_torque_drift")


# ============================================================== TC-U4
def cmd_torque_validate(a):
    D = pd.read_csv(a.csv)          # columns: mass_kg, arm_m, measured_nm [, dir (up/down), run]
    g = a.g; D["ref_nm"] = D["mass_kg"] * g * D["arm_m"]
    rel = np.sqrt((a.u_mass / D["mass_kg"].where(D["mass_kg"] > 0)) ** 2 + (a.u_arm / D["arm_m"]) ** 2 + (a.u_g / g) ** 2)
    D["u_ref_nm"] = (rel * D["ref_nm"]).fillna(0)
    zero = D[D["mass_kg"] == 0]["measured_nm"].mean() if (D["mass_kg"] == 0).any() else 0.0
    D["measured_zeroed"] = D["measured_nm"] - (0 if np.isnan(zero) else zero)
    D["err_nm"] = D["measured_zeroed"] - D["ref_nm"]; D["err_pct"] = 100 * D["err_nm"] / D["ref_nm"].where(D["ref_nm"] > 0)
    L = D[D["ref_nm"] > 0]
    A = L.groupby("ref_nm").agg(u_ref=("u_ref_nm", "mean"), meas=("measured_zeroed", "mean"), meas_sd=("measured_zeroed", "std"), n=("measured_zeroed", "size")).reset_index()
    A["err_nm"] = A["meas"] - A["ref_nm"]; A["err_pct"] = 100 * A["err_nm"] / A["ref_nm"]
    slope, icpt = np.polyfit(L["ref_nm"], L["measured_zeroed"], 1)
    resid = L["measured_zeroed"] - (slope * L["ref_nm"] + icpt)
    hyst = np.nan
    if "dir" in D:
        u, d = L[L["dir"] == "up"].groupby("ref_nm")["measured_zeroed"].mean(), L[L["dir"] == "down"].groupby("ref_nm")["measured_zeroed"].mean()
        c = u.index.intersection(d.index); hyst = float((u[c] - d[c]).abs().max()) if len(c) else np.nan
    print(f"g = {g}; zero reading {zero:.5f} N.m; fit measured = {slope:.5f} x ref + {icpt:.5f}; residual SD {resid.std(ddof=2):.5f} N.m; max hysteresis {hyst:.5f} N.m")
    A.to_csv(os.path.join(outdir(a), "torque_validation.csv"), index=False)
    pd.Series(dict(g=g, zero_nm=zero, slope=slope, intercept_nm=icpt, resid_sd_nm=resid.std(ddof=2), max_abs_err_nm=A["err_nm"].abs().max(),
                   max_abs_err_pct=A["err_pct"].abs().max(), hysteresis_nm=hyst, mean_u_ref_pct=float((A["u_ref"] / A["ref_nm"]).mean() * 100))).to_csv(os.path.join(outdir(a), "torque_validation_summary.csv"), header=["value"])
    T = pd.DataFrame({"Reference (N\\,m)": A["ref_nm"].map(lambda v: f(v, 4)), "$\\pm u$ (N\\,m)": A["u_ref"].map(lambda v: f(v, 4)), "Measured (N\\,m)": A["meas"].map(lambda v: f(v, 4)),
                      "Error (N\\,m)": A["err_nm"].map(lambda v: f(v, 4)), "Error (\\%)": A["err_pct"].map(lambda v: f(v, 2))})
    tex(T, a, "tab_torque_validation", "Torque validation against the reference $\\tau = m g r$: reference with its standard uncertainty, measured value after zeroing, and error.", "tab:torque-validation")
    fig, ax = plt.subplots(1, 2, figsize=(10.5, 4)); mx = max(A["ref_nm"].max(), A["meas"].max()) * 1.05
    ax[0].errorbar(A["ref_nm"], A["meas"], xerr=A["u_ref"], yerr=A["meas_sd"].fillna(0), fmt="o", capsize=2); ax[0].plot([0, mx], [0, mx], "k--", lw=1, label="1:1")
    ax[0].plot([0, mx], [icpt, slope * mx + icpt], "C1-", lw=0.9, label=f"fit, slope {slope:.4f}"); ax[0].set(xlabel="Reference torque (N$\\cdot$m)", ylabel="Measured torque (N$\\cdot$m)", title="Measured against reference"); ax[0].legend()
    ax[1].errorbar(A["ref_nm"], A["err_nm"], yerr=A["u_ref"], fmt="o-", capsize=2); ax[1].axhline(0, color="k", lw=0.8); ax[1].set(xlabel="Reference torque (N$\\cdot$m)", ylabel="Error (N$\\cdot$m)", title="Residual (bars: reference uncertainty)")
    save(fig, a, "fig_torque_validation")


# ============================================================== Eq 5.3 uncertainty
def cmd_power_unc(a):
    n = np.array([float(x) for x in a.speeds.split(",")]); tq = np.array([float(x) for x in a.torques.split(",")])
    u_tq = np.sqrt(a.u_torque_abs ** 2 + (a.u_torque_pct / 100 * tq) ** 2); rt = u_tq / tq; rn = a.u_speed_pct / 100 * np.ones_like(n)
    rr, rd, rm = a.u_rho_pct / 100, a.u_D_pct / 100, a.u_mu_pct / 100
    # Npo = 2 pi tau / (rho n^2 D^5): n enters P = tau*omega AND n^3, so its errors are fully correlated and the exponent is 2 (not sqrt(1+9))
    u_npo = np.sqrt(rt ** 2 + (2 * rn) ** 2 + rr ** 2 + (5 * rd) ** 2)
    u_re = np.sqrt(rn ** 2 + rr ** 2 + (2 * rd) ** 2 + rm ** 2)
    u_p = np.sqrt(rt ** 2 + rn ** 2)
    R = pd.DataFrame(dict(speed_rpm=n, torque_nm=tq, u_torque_pct=100 * rt, u_speed_pct=100 * rn, u_P_pct=100 * u_p, u_Npo_pct=100 * u_npo, u_Nre_pct=100 * u_re,
                          share_torque=rt ** 2 / u_npo ** 2, share_speed=(2 * rn) ** 2 / u_npo ** 2, share_D=(5 * rd) ** 2 / u_npo ** 2, share_rho=rr ** 2 / u_npo ** 2))
    R["dominant"] = R[["share_torque", "share_speed", "share_D", "share_rho"]].idxmax(axis=1).str.replace("share_", "")
    R.to_csv(os.path.join(outdir(a), "power_uncertainty.csv"), index=False); print(R.round(3).to_string(index=False))
    T = pd.DataFrame({"$n$ (rpm)": R["speed_rpm"].map(lambda v: f(v, 0)), "$\\tau$ (N\\,m)": R["torque_nm"].map(lambda v: f(v, 3)), "$u(\\tau)/\\tau$ (\\%)": R["u_torque_pct"].map(f),
                      "$u(n)/n$ (\\%)": R["u_speed_pct"].map(f), "$u(P)/P$ (\\%)": R["u_P_pct"].map(f), "$u(N_{Po})/N_{Po}$ (\\%)": R["u_Npo_pct"].map(f), "Dominant": R["dominant"]})
    tex(T, a, "tab_power_uncertainty", "Propagated relative standard uncertainty of shaft power and Power number; $N_{Po} \\propto \\tau / (\\rho n^2 D^5)$.", "tab:power-uncertainty")


# ============================================================== TC-S2 / S3
def cmd_endurance(a):
    df = pd.read_csv(a.csv); dur = df["elapsed_s"].iloc[-1]; per = df["loop_period_us"].to_numpy()[1:]; late = df["late_us"].to_numpy()
    S = dict(duration_s=dur, rows=len(df), rows_expected=int(dur * 100), mean_period_us=per.mean(), sd_period_us=per.std(ddof=1), p50_us=np.percentile(per, 50),
             p99_us=np.percentile(per, 99), p999_us=np.percentile(per, 99.9), max_period_us=per.max(), frac_over_15ms=float((per > 15000).mean()), n_over_15ms=int((per > 15000).sum()),
             late_p99_us=np.percentile(late, 99), late_max_us=late.max())
    m = df["raw_rpm"] > 5
    if m.any():
        S["rpm_mean"] = df.loc[m, "raw_rpm"].mean(); S["rpm_sd"] = df.loc[m, "raw_rpm"].std(ddof=1)
        S["rpm_drift_per_h"] = np.polyfit(df.loc[m, "elapsed_s"] / 3600, df.loc[m, "filtered_rpm"], 1)[0]
    if "torque_nm" in df and df["torque_nm"].notna().sum() > 100:
        tq = df.dropna(subset=["torque_nm"]); S["torque_mean_nm"] = tq["torque_nm"].mean(); S["torque_drift_nm_per_h"] = np.polyfit(tq["elapsed_s"] / 3600, tq["torque_nm"], 1)[0]
    if "power_w" in df and df["power_w"].notna().any(): S["power_mean_w"] = df["power_w"].mean(); S["power_max_w"] = df["power_w"].max()
    faults = []
    if a.stdout:
        for ln in open(a.stdout, errors="replace"):
            if re.search(r"\[SAFETY\]|ABORT|TRIP|WARNING|CRITICAL|Cannot|dropout", ln): faults.append(ln.strip())
    S["fault_lines"] = len(faults); S["completed"] = bool(abs(dur - a.expect_s) < 2.0) if a.expect_s else np.nan
    if a.platform:
        P = pd.read_csv(a.platform); S["temp_start_c"], S["temp_end_c"], S["temp_max_c"] = P["temp_c"].iloc[0], P["temp_c"].iloc[-1], P["temp_c"].max()
        S["throttled_ever"] = bool(P["throttled"].astype(str).str.lower().ne("0x0").any())
    pd.Series(S).to_csv(os.path.join(outdir(a), "endurance_summary.csv"), header=["value"])
    for k, v in S.items(): print(f"{k:>24s}: {v}")
    if faults: open(os.path.join(outdir(a), "endurance_faults.txt"), "w").write("\n".join(faults))
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0, 0].hist(per / 1e3, bins=np.linspace(per.min() / 1e3, min(per.max() / 1e3, 14), 80), log=True); ax[0, 0].set(xlabel="Loop period (ms)", ylabel="Loops", title="Loop-period distribution")
    ax[0, 1].plot(df["elapsed_s"] / 60, df["filtered_rpm"], lw=0.5); ax[0, 1].set(xlabel="Time (min)", ylabel="Filtered speed (rpm)", title="Speed over the run")
    if "torque_nm" in df: ax[1, 0].plot(df["elapsed_s"] / 60, df["torque_nm"], lw=0.5); ax[1, 0].set(xlabel="Time (min)", ylabel="Torque (N$\\cdot$m)", title="Torque over the run")
    if a.platform: ax[1, 1].plot((P["unix_s"] - P["unix_s"].iloc[0]) / 60, P["temp_c"]); ax[1, 1].set(xlabel="Time (min)", ylabel="SoC temperature ($^\\circ$C)", title="Thermal behaviour")
    save(fig, a, "fig_endurance")


# ============================================================== TC-S4 / S6
TOL = {"time_s": .005, "Raw_RPM": .005, "Filtered_RPM": .005, "Revolutions": .5, "Torque_Nm": .0005, "Power_W": .0005, "N_Re": .005, "N_Po": .0005, "PWM_pct": .005,
       "Target_RPM": .005, "Bus_V": .005, "Current_mA": .5, "Elec_Power_W": .005}
HDR = {"t (s)": "time_s", "Raw RPM": "Raw_RPM", "Filtered RPM": "Filtered_RPM", "Revolutions": "Revolutions", "Torque": "Torque_Nm", "Power (W)": "Power_W", "N_Re": "N_Re",
       "N_Po": "N_Po", "PWM (%)": "PWM_pct", "Target RPM": "Target_RPM", "Bus (V)": "Bus_V", "Current (mA)": "Current_mA", "Elec. Power (W)": "Elec_Power_W"}


def cmd_integrity(a):
    d = pd.read_csv(a.daemon_csv); s = pd.read_csv(a.dash_csv); S = {}
    td = d["elapsed_s"].to_numpy(); ts = s["t (s)"].to_numpy()
    S["daemon_rows"] = len(d); S["dashboard_rows"] = len(s); S["daemon_duration_s"] = td[-1]; S["dashboard_duration_s"] = ts[-1] - ts[0]
    dts = np.diff(ts); med = np.median(dts); gaps = dts > 1.5 * med
    S["dashboard_median_dt_s"] = med; S["dashboard_gaps"] = int(gaps.sum()); S["dashboard_missing_est"] = int(np.sum(np.round(dts[gaps] / med) - 1))
    S["dashboard_nonmonotonic"] = int((dts <= 0).sum())
    S["record_loss_pct"] = 100 * (1 - len(s) / max(1, len(d)))
    if "revs" in d and "Revolutions" in s:
        S["revs_end_daemon"] = d["revs"].iloc[-1]; S["revs_end_dashboard"] = s["Revolutions"].iloc[-1]
    if a.mat:
        from scipy.io import loadmat
        m = loadmat(a.mat); worst = {}
        for h, v in HDR.items():
            if h in s and v in m:
                x = s[h].to_numpy(float); y = np.asarray(m[v]).ravel().astype(float); n = min(len(x), len(y)); worst[v] = float(np.nanmax(np.abs(x[:n] - y[:n]))) if n else np.nan
                if len(x) != len(y): S[f"mat_length_mismatch_{v}"] = (len(x), len(y))
        S["mat_roundtrip_worst"] = worst; S["mat_roundtrip_ok"] = all(worst[k] <= TOL[k] * 1.001 for k in worst)
    pd.Series({k: str(v) for k, v in S.items()}).to_csv(os.path.join(outdir(a), "integrity_summary.csv"), header=["value"])
    for k, v in S.items(): print(f"{k:>28s}: {v}")
    fig, ax = plt.subplots(figsize=(6, 3.5)); ax.hist(dts * 1e3, bins=60, log=True); ax.set(xlabel="Interval between received records (ms)", ylabel="Count", title="Dashboard record spacing"); save(fig, a, "fig_logging_intervals")


# ============================================================== TC-S5
def cmd_plausibility(a):
    S = pd.read_csv(a.summary); L = pd.read_csv(a.lit).sort_values("N_Re")
    S = S[(S["N_Re"] > 0) & (S["N_Po"] > 0)].copy()
    ok = (S["N_Re"] >= L["N_Re"].min()) & (S["N_Re"] <= L["N_Re"].max())
    S["lit_N_Po"] = np.where(ok, 10 ** np.interp(np.log10(S["N_Re"]), np.log10(L["N_Re"]), np.log10(L["N_Po"])), np.nan)
    S["dev_pct"] = 100 * (S["N_Po"] - S["lit_N_Po"]) / S["lit_N_Po"]
    S.to_csv(os.path.join(outdir(a), "plausibility.csv"), index=False)
    print(S[["target_rpm", "N_Re", "N_Po", "lit_N_Po", "dev_pct"]].round(3).to_string(index=False)); print("points outside the literature N_Re range:", int((~ok).sum()))
    fig, ax = plt.subplots(figsize=(6.2, 4.4)); ax.loglog(L["N_Re"], L["N_Po"], "k-", label=a.label); ax.loglog(S["N_Re"], S["N_Po"], "o", label="MIXR-1")
    ax.set(xlabel="$N_{Re}$", ylabel="$N_{Po}$"); ax.legend(); ax.grid(True, which="both"); save(fig, a, "fig_plausibility")
    T = pd.DataFrame({"Setpoint (rpm)": S["target_rpm"].map(lambda v: f(v, 0)), "$N_{Re}$": S["N_Re"].map(lambda v: f(v, 0)), "$N_{Po}$": S["N_Po"].map(lambda v: f(v, 3)),
                      "Literature": S["lit_N_Po"].map(lambda v: f(v, 3)), "Deviation (\\%)": S["dev_pct"].map(lambda v: f(v, 1))})
    tex(T, a, "tab_plausibility", "Measured Power number against the literature curve at each sweep point.", "tab:plausibility")


# ============================================================== Fig 5.3
def cmd_rawfilt(a):
    df = pd.read_csv(a.csv); w = df[(df["elapsed_s"] >= a.t0) & (df["elapsed_s"] < a.t1)]
    sd = w["raw_rpm"].std(ddof=1); mean = w["raw_rpm"].mean(); pred = mean * 5e-6 / 0.01
    print(f"mean {mean:.2f} rpm, raw SD {sd:.3f} rpm, filtered SD {w['filtered_rpm'].std(ddof=1):.3f} rpm, M/T bound {pred:.3f} rpm, SD / bound = {sd / pred:.1f}")
    fig, ax = plt.subplots(figsize=(8, 3.6)); ax.plot(w["elapsed_s"], w["raw_rpm"], color="gray", lw=0.8, label=f"raw (SD {sd:.2f} rpm)"); ax.plot(w["elapsed_s"], w["filtered_rpm"], lw=1.4, label="filtered")
    ax.set(xlabel="Time (s)", ylabel="Speed (rpm)", title=f"Raw and filtered speed, mean {mean:.0f} rpm (M/T bound {pred:.2f} rpm)"); ax.legend(); save(fig, a, "fig_raw_vs_filtered")


# ============================================================== load disturbance (TC-S1)
def cmd_disturbance(a):
    df = pd.read_csv(a.csv); ev = pd.read_csv(a.events); ev["t"] = ev["unix_s"] - a.t0_unix
    sp = df["target_rpm"].median(); t = df["elapsed_s"].to_numpy(); y = df["filtered_rpm"].to_numpy(); rows = []
    for _, e in ev.iterrows():
        if not (0 < e["t"] < t[-1]): continue
        pre = y[(t >= e["t"] - 5) & (t < e["t"] - 0.2)]; base = pre.mean() if len(pre) else sp
        m = (t >= e["t"]) & (t < e["t"] + a.window); seg = y[m]; ts = t[m] - e["t"]
        dev = seg - base; k = np.argmax(np.abs(dev)); out = np.where(np.abs(seg - base) > a.band * abs(base))[0]
        rows.append(dict(event=e["label"], t_s=e["t"], pre_rpm=base, worst_dev_rpm=dev[k], worst_dev_pct=100 * dev[k] / base, t_worst_s=ts[k],
                         recover_s=(ts[out[-1]] if len(out) and out[-1] < len(seg) - 2 else (0.0 if not len(out) else np.nan)), iae_rpm_s=float(np.sum(np.abs(dev)) * 0.01)))
    R = pd.DataFrame(rows); R.to_csv(os.path.join(outdir(a), "disturbance_metrics.csv"), index=False); print(R.round(3).to_string(index=False))
    fig, ax = plt.subplots(figsize=(9, 3.8)); ax.plot(t, df["raw_rpm"], color="gray", lw=0.5, alpha=0.6, label="raw"); ax.plot(t, y, lw=1.2, label="filtered"); ax.axhline(sp, color="k", ls="--", lw=0.8, label="setpoint")
    for _, e in ev.iterrows(): ax.axvline(e["t"], color="C3", ls=":", lw=1); ax.text(e["t"], ax.get_ylim()[1], e["label"], rotation=90, va="top", fontsize=7)
    ax.set(xlabel="Time (s)", ylabel="Speed (rpm)"); ax.legend(loc="lower left"); save(fig, a, "fig_disturbance")


def cmd_torque_mean(a):
    rows = []
    for p in a.csv:
        d = pd.read_csv(p); tc = d.columns[0]; yc = next(c for c in d.columns if "torque" in c.lower()); d = d[d[tc] >= d[tc].min() + a.skip]
        if "ok" in d: d = d[d["ok"] == 1]
        rows.append(dict(file=os.path.basename(p), n=len(d), mean_nm=d[yc].mean(), sd_nm=d[yc].std(ddof=1), drift_nm=d[yc].iloc[len(d) // 2:].mean() - d[yc].iloc[:len(d) // 2].mean()))
    R = pd.DataFrame(rows); print(R.to_string(index=False)); R.to_csv(os.path.join(outdir(a), "torque_means.csv"), index=False)


# ============================================================== helpers for the rig
def cmd_mark(a):
    print("Press Enter to stamp an event (optionally type a label first). Ctrl-C to finish."); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    new = not os.path.exists(a.out)
    with open(a.out, "a") as fh:
        if new: fh.write("unix_s,label\n")
        try:
            while True:
                lab = input("event> ").strip() or "event"; t = time.time(); fh.write(f"{t:.3f},{lab.replace(',', ';')}\n"); fh.flush(); print(f"  stamped {lab} at {t:.3f}")
        except (KeyboardInterrupt, EOFError):
            pass


def cmd_platform_log(a):
    new = not os.path.exists(a.out)
    with open(a.out, "a") as fh:
        if new: fh.write("unix_s,temp_c,throttled,cpu_khz\n")
        try:
            while True:
                def sh(c):
                    try: return subprocess.run(c, shell=True, capture_output=True, text=True, timeout=5).stdout.strip()
                    except Exception: return ""
                tmp = re.search(r"[-\d.]+", sh("vcgencmd measure_temp") or "nan"); thr = sh("vcgencmd get_throttled").split("=")[-1] or "n/a"
                fh.write(f"{time.time():.1f},{tmp.group(0) if tmp else 'nan'},{thr},{sh('cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq')}\n"); fh.flush(); time.sleep(a.period)
        except KeyboardInterrupt:
            pass


# ============================================================== selftest
def cmd_selftest(a):
    rng = np.random.default_rng(1); d = os.path.join(a.out, "selftest_inputs"); os.makedirs(d, exist_ok=True)
    def rows(n, f):
        return pd.DataFrame([f(i) for i in range(n)])
    # duty sweep up and down, 12 s holds, deadband 6 %
    for lab, order in (("up", range(0, 22)), ("down", range(21, -1, -1))):
        recs = []
        for k, s in enumerate(order):
            duty = s * 4.0; rpm = 0 if duty < (6 if lab == "up" else 4) else 22 * (duty - 4)
            for i in range(1200):
                recs.append(dict(elapsed_s=k * 12 + i * 0.01 + 0.01, step_index=k, pwm=int(duty / 100 * 4095), raw_rpm=max(0, rpm + rng.normal(0, 1 + 0.002 * rpm)), power_w=2 + 0.0004 * rpm ** 1.5, current_ma=100 + rpm, bus_v=24))
        pd.DataFrame(recs).to_csv(os.path.join(d, f"duty_{lab}.csv"), index=False)
    # step response: 0 -> 500 -> 800 -> 400, first order tau 0.3 s + noise
    t = np.arange(0, 60, 0.01); tg = np.where(t < 5, 500, np.where(t < 25, 500, np.where(t < 45, 800, 400))); tg = np.where(t < 2, 0, tg); y = np.zeros_like(t)
    for i in range(1, len(t)): y[i] = y[i - 1] + (tg[i] - y[i - 1]) * 0.01 / 0.3 + rng.normal(0, 0.3)
    pd.DataFrame(dict(elapsed_s=t, target_rpm=tg, raw_rpm=y + rng.normal(0, 1.5, len(t)), filtered_rpm=y, power_w=0.00004 * y ** 2, loop_period_us=10000 + rng.normal(0, 40, len(t)), late_us=np.abs(rng.normal(0, 30, len(t))), torque_nm=0.0002 * y)).to_csv(os.path.join(d, "steps.csv"), index=False)
    # torque drift: 90 min, warm-up exponential
    tt = np.arange(0, 5400, 0.05); pd.DataFrame(dict(time_s=tt, torque_nm=0.01 * np.exp(-tt / 900) + 0.0015 + rng.normal(0, 0.0008, len(tt)), ok=1)).to_csv(os.path.join(d, "tdrift.csv"), index=False)
    # torque validation
    recs = []
    for dr, ms in (("up", [0, .05, .1, .2, .3, .5]), ("down", [.5, .3, .2, .1, .05, 0])):
        for m in ms: recs.append(dict(mass_kg=m, arm_m=0.10, measured_nm=m * 9.782 * 0.10 * 1.004 + 0.001 + rng.normal(0, 0.0003), dir=dr))
    pd.DataFrame(recs).to_csv(os.path.join(d, "loads.csv"), index=False)
    # integrity: dashboard drops 0.5 %
    n = 6000; el = np.arange(n) * 0.01 + 0.01; pd.DataFrame(dict(elapsed_s=el, raw_rpm=500 + rng.normal(0, 1, n), revs=(el * 500 / 60).astype(int), loop_period_us=10000, late_us=0)).to_csv(os.path.join(d, "daemon.csv"), index=False)
    keep = rng.random(n) > 0.005; pd.DataFrame({"t (s)": el[keep], "Raw RPM": 500 + rng.normal(0, 1, keep.sum()), "Revolutions": (el[keep] * 500 / 60).astype(int)}).to_csv(os.path.join(d, "dash.csv"), index=False)
    # plausibility
    pd.DataFrame(dict(N_Re=[1e3, 1e4, 1e5], N_Po=[5.0, 5.0, 5.0])).to_csv(os.path.join(d, "lit.csv"), index=False)
    pd.DataFrame(dict(target_rpm=[200, 400, 800], N_Re=[2e4, 4e4, 8e4], N_Po=[5.2, 4.9, 5.1])).to_csv(os.path.join(d, "seq.csv"), index=False)
    P = lambda x: os.path.join(d, x)
    ns = lambda **k: argparse.Namespace(out=a.out, **k)
    cmd_duty(ns(csv=[P("duty_up.csv"), P("duty_down.csv")], labels="up,down", hold=12.0, window=None, moving_rpm=5.0))
    cmd_steps(ns(csv=[P("steps.csv")], band=0.05))
    cmd_torque_drift(ns(csv=P("tdrift.csv"), window_n=40, steady_frac=0.5))
    cmd_torque_validate(ns(csv=P("loads.csv"), u_mass=0.0005, u_arm=0.0002, g=9.782, u_g=0.005))
    cmd_power_unc(ns(speeds="100,500,1000", torques="0.01,0.05,0.12", u_torque_abs=0.002, u_torque_pct=0.5, u_speed_pct=0.3, u_rho_pct=0.2, u_D_pct=0.1, u_mu_pct=1.0))
    cmd_endurance(ns(csv=P("steps.csv"), platform=None, stdout=None, expect_s=60.0))
    cmd_integrity(ns(daemon_csv=P("daemon.csv"), dash_csv=P("dash.csv"), mat=None))
    cmd_plausibility(ns(summary=P("seq.csv"), lit=P("lit.csv"), label="literature"))
    cmd_rawfilt(ns(csv=P("steps.csv"), t0=10, t1=20))
    pd.DataFrame(dict(unix_s=[1000.0 + 30, 1000.0 + 50], label=["water drop", "baffles in"])).to_csv(P("events.csv"), index=False)
    d2 = pd.read_csv(P("steps.csv")); d2["raw_rpm"] = d2["filtered_rpm"]; d2.loc[(d2.elapsed_s > 30) & (d2.elapsed_s < 31), "filtered_rpm"] -= 40; d2.to_csv(P("steps_d.csv"), index=False)
    cmd_disturbance(ns(csv=P("steps_d.csv"), events=P("events.csv"), t0_unix=1000.0, window=15.0, band=0.05))
    pd.DataFrame(dict(time_s=np.arange(0, 20, 0.05), torque_nm=0.05 + rng.normal(0, 0.0004, 400), ok=1)).to_csv(P("tq_load.csv"), index=False)
    cmd_torque_mean(ns(csv=[P("tq_load.csv")], skip=3.0))
    print("\nSELFTEST finished: outputs in", a.out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    def add(name, fn, *args):
        p = sp.add_parser(name); p.add_argument("--out", default="ch5_results"); p.set_defaults(fn=fn)
        for a_ in args: p.add_argument(a_[0], **a_[1])
        return p
    add("duty", cmd_duty, ("--csv", dict(nargs="+", required=True)), ("--labels", dict(default=None)), ("--hold", dict(type=float, required=True)), ("--window", dict(default=None)), ("--moving-rpm", dict(type=float, default=5.0)))
    add("steps", cmd_steps, ("--csv", dict(nargs="+", required=True)), ("--band", dict(type=float, default=0.05)))
    add("torque-drift", cmd_torque_drift, ("--csv", dict(required=True)), ("--window-n", dict(type=int, default=40)), ("--steady-frac", dict(type=float, default=0.5)))
    add("torque-validate", cmd_torque_validate, ("--csv", dict(required=True)), ("--u-mass", dict(type=float, required=True)), ("--u-arm", dict(type=float, required=True)), ("--g", dict(type=float, default=G_STD)), ("--u-g", dict(type=float, default=0.005)))
    add("power-unc", cmd_power_unc, ("--speeds", dict(required=True)), ("--torques", dict(required=True)), ("--u-torque-abs", dict(type=float, required=True)), ("--u-torque-pct", dict(type=float, default=0.0)),
        ("--u-speed-pct", dict(type=float, required=True)), ("--u-rho-pct", dict(type=float, default=0.2)), ("--u-D-pct", dict(type=float, default=0.1)), ("--u-mu-pct", dict(type=float, default=1.0)))
    add("endurance", cmd_endurance, ("--csv", dict(required=True)), ("--platform", dict(default=None)), ("--stdout", dict(default=None)), ("--expect-s", dict(type=float, default=None)))
    add("integrity", cmd_integrity, ("--daemon-csv", dict(required=True)), ("--dash-csv", dict(required=True)), ("--mat", dict(default=None)))
    add("plausibility", cmd_plausibility, ("--summary", dict(required=True)), ("--lit", dict(required=True)), ("--label", dict(default="Literature")))
    add("rawfilt", cmd_rawfilt, ("--csv", dict(required=True)), ("--t0", dict(type=float, required=True)), ("--t1", dict(type=float, required=True)))
    add("disturbance", cmd_disturbance, ("--csv", dict(required=True)), ("--events", dict(required=True)), ("--t0-unix", dict(type=float, required=True)), ("--window", dict(type=float, default=20.0)), ("--band", dict(type=float, default=0.05)))
    add("torque-mean", cmd_torque_mean, ("--csv", dict(nargs="+", required=True)), ("--skip", dict(type=float, default=3.0)))
    add("mark", cmd_mark); sp.choices["mark"].set_defaults(out="run.events.csv")
    add("platform-log", cmd_platform_log, ("--period", dict(type=float, default=10.0))); sp.choices["platform-log"].set_defaults(out="plat.csv")
    add("selftest", cmd_selftest)
    a = ap.parse_args(); a.fn(a)


if __name__ == "__main__":
    main()

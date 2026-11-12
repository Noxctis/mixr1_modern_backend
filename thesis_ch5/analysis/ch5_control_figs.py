#!/usr/bin/env python3
"""
ch5_control_figs.py - figures of Sections 5.3-5.5 (driver, PI control, torque sensor) from the logs of the project.

    python ch5_control_figs.py --root "<School Stuff>" --out ../Figs

--root holds: "Research SeBoCa Test" (dashboard logs, loadcell, disturbance logs) and "mixr1_modern_backend" (logs/ch5_new, matrix_ki_174mm_test).
Figures:  fig_duty_rpm  fig_plant_id  fig_bode_pm  fig_design_tradeoff  fig_pi_step  fig_gain_matrix  fig_disturbance  fig_loadcell
Needs numpy, pandas, scipy, matplotlib.
"""
import argparse, os, glob
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

C = {"96": "#0072B2", "174": "#D55E00", "ol": "#7A7A7A", "a": "#0072B2", "b": "#D55E00", "c": "#009E73", "d": "#CC79A7", "e": "#E69F00"}


def style():
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"], "mathtext.fontset": "stix",
                         "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5, "legend.fontsize": 7.8, "axes.grid": True, "grid.color": "#DDDDDD",
                         "grid.linewidth": 0.5, "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 300, "pdf.fonttype": 42})


def save(fig, out, name):
    fig.savefig(os.path.join(out, name + ".pdf")); fig.savefig(os.path.join(out, name + ".png"), dpi=170); plt.close(fig)


# ------------------------------------------------------------------ model helpers
def loop(w, Kp, Ki, K, A, delay):
    s = 1j * w
    return (Kp + Ki / s) * K * A / (s + A) * np.exp(-s * delay)


def margins(Kp, Ki, K, A, delay):
    w = np.logspace(-1, 3, 20000); L = loop(w, Kp, Ki, K, A, delay); mag = np.abs(L); ph = np.unwrap(np.angle(L))
    i = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    wc = np.interp(1, mag[i[0]:i[0] + 2][::-1], w[i[0]:i[0] + 2][::-1]) if len(i) else np.nan
    pm = 180 + np.degrees(np.interp(wc, w, ph)) if np.isfinite(wc) else np.nan
    j = np.where((ph[:-1] > -np.pi) & (ph[1:] <= -np.pi))[0]
    gm = -20 * np.log10(np.interp(-np.pi, [ph[j[0] + 1], ph[j[0]]], [mag[j[0] + 1], mag[j[0]]])) if len(j) else np.inf
    return wc, pm, gm


def simulate(t_target, target, Kp, Ki, K, A, theta, rpm0=0.0, u0=0.0, Ts=0.01, dt=0.001, u_max=4095.0):
    """PI at Ts on the output of the first-order plant with dead time theta (feedback = true speed, filter coefficient 1)."""
    n_steps = int(round((t_target[-1]) / dt)); nd = int(round(theta / dt)); y = rpm0; ubuf = [u0] * (nd + 1); I = 0.0; u = u0
    out = np.zeros(n_steps); k_ctrl = int(round(Ts / dt)); Tg = np.interp(np.arange(n_steps) * dt, t_target, target)
    for k in range(n_steps):
        if k % k_ctrl == 0:
            e = Tg[k] - y; I += Ki * Ts * e; I = np.clip(I, 0, u_max); u = np.clip(Kp * e + I, 0, u_max)
        ubuf.append(u); ud = ubuf[-nd - 1] if nd > 0 else u
        y += dt * A * (K * ud - y); out[k] = y
    return np.arange(n_steps) * dt, out


# ------------------------------------------------------------------ 1. duty against speed
def fig_duty_rpm(R, out):
    cet = pd.read_csv(os.path.join(R, "..", "mixr1_modern_backend", "cet_pwm_sweep.csv")) if False else pd.read_csv(os.path.join(R, "cet_pwm_sweep.csv"))
    free = []
    for k, g in cet.groupby("step_index"):
        r = g.raw_rpm.to_numpy()[100:900]; free.append((g.pwm_percent.iloc[0], r.mean()))
    free = np.array(free)
    runs = {"96": "20261107_180055", "174": "20261106_211730"}; S = {}
    for k, ts in runs.items(): S[k] = pd.read_csv(os.path.join(R, f"mixr1_{k}mm_{ts}_sequence_summary.csv"))
    fig, ax = plt.subplots(1, 3, figsize=(7.4, 2.8))
    ax[0].plot(free[:, 1], free[:, 0], "o-", color=C["ol"], ms=4, mec="white", mew=0.4, lw=1, label="free motor (no impeller)")
    for k in S: ax[0].plot(S[k].rpm_filtered_mean, S[k].pwm_pct_mean, "o-" if k == "96" else "s-", color=C[k], ms=3.8, mec="white", mew=0.4, lw=1, label=f"{k} mm impeller in water")
    ax[0].set_xlabel("Speed (rpm)"); ax[0].set_ylabel("Duty (%)"); ax[0].set_title("(a) Duty needed for a speed"); ax[0].legend(frameon=False, loc="upper left")
    for k in S:
        ax[1].plot(S[k].rpm_filtered_mean, S[k].elec_power_W_mean, "o-" if k == "96" else "s-", color=C[k], ms=3.8, mec="white", mew=0.4, lw=1, label=f"{k} mm, electrical")
        ax[1].plot(S[k].rpm_filtered_mean, S[k].shaft_power_W.clip(lower=0), "--", color=C[k], lw=1, label=f"{k} mm, shaft")
    ax[1].set_xlabel("Speed (rpm)"); ax[1].set_ylabel("Power (W)"); ax[1].set_title("(b) Electrical and shaft power"); ax[1].legend(frameon=False, fontsize=7)
    for k in S:
        d = S[k]; m = d.torque_Nm_mean > 0.01; ax[2].plot(d.rpm_filtered_mean[m], 100 * d.shaft_power_W[m] / d.elec_power_W_mean[m], "o-" if k == "96" else "s-", color=C[k], ms=3.8, mec="white", mew=0.4, lw=1, label=f"{k} mm")
    ax[2].set_xlabel("Speed (rpm)"); ax[2].set_ylabel("Shaft / electrical power (%)"); ax[2].set_title("(c) Drive efficiency"); ax[2].legend(frameon=False); ax[2].set_ylim(0, None)
    fig.tight_layout(); save(fig, out, "fig_duty_rpm")


# ------------------------------------------------------------------ 2. plant identification
def fit_step(t, n, du, n0):
    def model(p):
        K, A, th = p; x = np.clip(t - th, 0, None); return n0 + K * du * (1 - np.exp(-A * x))
    r = least_squares(lambda p: model(p) - n, [0.65, 30, 0.008], bounds=([0.1, 5, 0.0], [2, 100, 0.05])); return r.x, np.sqrt(np.mean(r.fun ** 2)), model(r.x)


def fig_plant_id(M, out):
    d = pd.read_csv(os.path.join(M, "logs", "ch5_new", "estimators", "plant_sweep.csv")); rows = []; traces = {}
    prev = None
    for k, g in d.groupby("step_index"):
        t = g.elapsed_s.to_numpy(); t = t - t[0] + 0.0; n = g.raw_rpm.to_numpy(); u = g.pwm.iloc[0]
        if prev is not None and k >= 2 and k <= 8:
            du = u - prev["u"]; n0 = prev["tail"]; sel = t <= 1.2
            (K, A, th), rms, mod = fit_step(t[sel], n[sel], du, n0); rows.append(dict(step=k, base=n0, K=K, A=A, tau=1000 / A, theta=1000 * th, rms=rms)); traces[k] = (t[sel], n[sel], mod)
        prev = dict(u=u, tail=np.mean(n[int(0.7 * len(n)):]))
    T = pd.DataFrame(rows); T.round(3).to_csv(os.path.join(out, "plant_fit_aligned.csv"), index=False)
    fig, ax = plt.subplots(1, 3, figsize=(7.4, 2.8)); k = 4; t, n, mod = traces[k]
    ax[0].plot(t * 1000, n, ".", color="#999", ms=3, label="measured (raw)"); ax[0].plot(t * 1000, mod, "-", color=C["a"], lw=1.4, label="first order + dead time")
    r = T[T.step == k].iloc[0]; ax[0].axvline(r.theta, color="k", ls=":", lw=0.9); ax[0].text(r.theta + 10, 0.97, rf"$\theta$ = {r.theta:.1f} ms", fontsize=7.5, transform=ax[0].get_xaxis_transform(), va="top")
    ax[0].axvline(r.theta + r.tau, color=C["b"], ls=":", lw=0.9); ax[0].text(r.theta + r.tau + 10, 0.80, rf"$\theta+\tau$ = {r.theta + r.tau:.0f} ms", fontsize=7.5, color=C["b"], transform=ax[0].get_xaxis_transform(), va="top")
    ax[0].set_xlim(0, 400); ax[0].set_xlabel("Time after the step (ms)"); ax[0].set_ylabel("Speed (rpm)"); ax[0].set_title("(a) One step and its fit"); ax[0].legend(frameon=False, loc="center right")
    ax[1].plot(T.base, T.K, "o-", color=C["a"], ms=4, mec="white", mew=0.4); ax[1].set_xlabel("Speed before the step (rpm)"); ax[1].set_ylabel("Gain $K$ (rpm/count)"); ax[1].set_ylim(0.5, 0.8); ax[1].set_title("(b) Steady-state gain")
    ax[2].plot(T.base, T.tau, "o-", color=C["a"], ms=4, mec="white", mew=0.4, label=r"time constant $\tau=1/A$"); ax[2].plot(T.base, T.theta, "s-", color=C["b"], ms=4, mec="white", mew=0.4, label=r"dead time $\theta$")
    ax[2].set_xlabel("Speed before the step (rpm)"); ax[2].set_ylabel("Time (ms)"); ax[2].set_ylim(0, 50); ax[2].set_title("(c) Dynamics"); ax[2].legend(frameon=False)
    fig.tight_layout(); save(fig, out, "fig_plant_id"); print(T.round(2).to_string())
    return T


# ------------------------------------------------------------------ 3. Bode and phase margin
def fig_bode_pm(out):
    K, A, th, hold = 0.640, 1 / 0.031, 0.0082, 0.005
    cases = [("Earlier gains, CET estimate", 1.5641, 41.2249, th + hold, "#0072B2", "-"), ("Earlier gains, DCT estimate (+19 ms)", 1.5641, 41.2249, th + hold + 0.019, "#D55E00", "-"),
             ("Present gains, CET estimate", 0.2329, 7.509, th + hold, "#009E73", "--"), ("Present gains, DCT estimate (+19 ms)", 0.2329, 7.509, th + hold + 0.019, "#CC79A7", "--")]
    w = np.logspace(-0.5, 2.3, 3000); fig, ax = plt.subplots(2, 1, figsize=(4.7, 5.2), sharex=True); rows = []
    for lab, Kp, Ki, dly, col, ls in cases:
        L = loop(w, Kp, Ki, K, A, dly); wc, pm, gm = margins(Kp, Ki, K, A, dly); ph = np.degrees(np.unwrap(np.angle(L)))
        ax[0].plot(w, 20 * np.log10(np.abs(L)), ls, color=col, lw=1.4, label=lab); ax[1].plot(w, ph, ls, color=col, lw=1.4)
        ax[0].plot(wc, 0, "o", color=col, mec="white", ms=6, zorder=5); ax[1].plot(wc, -180 + pm, "o", color=col, mec="white", ms=6, zorder=5)
        ax[1].annotate(f"PM {pm:.0f}$^{{\\circ}}$", (wc, -180 + pm), textcoords="offset points", xytext=(5, 6 if "CET" in lab else -12), fontsize=7.5, color=col)
        rows.append(dict(gains=lab, Kp=Kp, Ki=Ki, wc=wc, PM=pm, GM_dB=gm))
    ax[0].axhline(0, color="k", lw=0.8); ax[1].axhline(-180, color="k", lw=0.8); ax[1].text(0.35, -176, "$-180^{\\circ}$", fontsize=7.5, va="bottom")
    ax[0].set_xscale("log"); ax[0].set_ylabel("Magnitude of $L(j\\omega)$ (dB)"); ax[0].legend(frameon=False, fontsize=7); ax[1].set_ylabel("Phase of $L(j\\omega)$ (degrees)"); ax[1].set_xlabel("Frequency (rad/s)"); ax[1].set_ylim(-270, -60)
    ax[0].set_title("Open-loop frequency response and phase margin"); fig.tight_layout(); save(fig, out, "fig_bode_pm")
    pd.DataFrame(rows).round(2).to_csv(os.path.join(out, "margins_model.csv"), index=False); print(pd.DataFrame(rows).round(2).to_string())


# ------------------------------------------------------------------ 4. design trade-off (why 5 rad/s)
def fig_design_tradeoff(out):
    K, A, th, hold = 0.640, 1 / 0.031, 0.0082, 0.005; wcs = np.logspace(-0.3, 1.6, 120)
    fig, ax = plt.subplots(1, 3, figsize=(7.4, 2.8))
    for lab, extra, col in (("CET estimate (13 ms total delay)", 0.0, C["a"]), ("DCT estimate (32 ms)", 0.019, C["b"])):
        d = th + hold + extra; pm = [margins(w / (K * A), w / K, K, A, d)[1] for w in wcs]; ax[0].plot(wcs, pm, color=col, lw=1.5, label=lab)
    ax[0].axhline(60, color="k", ls=":", lw=1); ax[0].text(0.55, 62, "60$^{\\circ}$ target", fontsize=7.5)
    ax[0].set_xscale("log"); ax[0].set_xlabel(r"Designed crossover $\omega_c$ (rad/s)"); ax[0].set_ylabel("Phase margin (degrees)"); ax[0].set_title("(a) Phase margin vs. bandwidth"); ax[0].legend(frameon=False, fontsize=7); ax[0].set_ylim(0, 95)
    tt = np.array([0.0, 0.2, 5.0]); cols = [C["c"], C["a"], C["e"], C["b"], C["d"]]; ws = [1, 5, 10, 20, 29]
    for w, col in zip(ws, cols):
        t, y = simulate(np.array([0, 0.05, 3.0]), np.array([0, 100.0, 100.0]), w / (K * A), w / K, K, A, th); pm = margins(w / (K * A), w / K, K, A, th + hold)[1]
        ax[1].plot(t, y, color=col, lw=1.3, label=rf"$\omega_c$ = {w} rad/s (PM {pm:.0f}$^{{\circ}}$)")
    ax[1].set_xlim(0, 3.0); ax[1].set_ylim(0, 125); ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Speed (rpm), 100 rpm step"); ax[1].set_title("(b) Step response, CET estimate"); ax[1].legend(frameon=False, fontsize=6.8, loc="lower right")
    tt = []
    for w in wcs[::6]:
        t, y = simulate(np.array([0, 0.05, 12.0]), np.array([0, 100.0, 100.0]), w / (K * A), w / K, K, A, th); i = np.argmax(y >= 90); tt.append((w, t[i] - 0.05 if y.max() >= 90 else np.nan, max(0.0, y.max() - 100)))
    tt = np.array(tt); ax[2].plot(tt[:, 0], tt[:, 1], "o-", color=C["a"], ms=4, label="10--90 % rise time (s)")
    ax2 = ax[2].twinx(); kp_ = tt[:, 0] / (K * A)
    ax2.plot(tt[:, 0], 100 * kp_ * 3.8 / 4095, "s-", color=C["b"], ms=3.5, label="CET noise (3.8 rpm)"); ax2.plot(tt[:, 0], 100 * kp_ * 21.1 / 4095, "s--", color=C["b"], ms=3.5, mfc="white", label="DCT 20 ms noise (21 rpm)")
    ax2.set_yscale("log"); ax2.set_ylabel("Duty noise from the estimate (% of full scale)", color=C["b"], fontsize=8); ax2.grid(False); ax2.spines["right"].set_visible(True); ax2.legend(frameon=False, loc="lower left", fontsize=6.8)
    ax[2].set_xscale("log"); ax[2].set_yscale("log"); ax[2].set_xlabel(r"Designed crossover $\omega_c$ (rad/s)"); ax[2].set_ylabel("Time to 90 % (s)", color=C["a"]); ax[2].set_title("(c) Speed against duty noise")
    fig.tight_layout(); save(fig, out, "fig_design_tradeoff")


# ------------------------------------------------------------------ 5. measured against simulated step responses
def fig_pi_step(M, out):
    E = os.path.join(M, "logs", "ch5_new", "estimators"); K, A, th = 0.667, 30.0, 0.008
    cases = [("Earlier gains $K_p=1.564$, $K_i=41.2$", "est-aligned_Bold_r1.csv", 1.5641, 41.2249, C["b"]), ("Identified-plant gains $K_p=0.248$, $K_i=6.87$", "est-aligned_Bimc_r1.csv", 0.2482, 6.8691, C["a"])]
    fig, ax = plt.subplots(2, 2, figsize=(7.4, 4.9)); res = []
    for j, (lab, f, Kp, Ki, col) in enumerate(cases):
        d = pd.read_csv(os.path.join(E, f)); t = d.elapsed_s.to_numpy(); tg = d.target_rpm.to_numpy(); n = d.raw_rpm.to_numpy()
        i0 = np.where(tg >= 500)[0][0]; i1 = np.where(tg >= 1500)[0][0]; sel = slice(max(i0 - 100, 0), min(i1 + 1500, len(t)))
        ts, ns, tgs = t[sel] - t[sel][0], n[sel], tg[sel]; sub = d.iloc[sel]; r0 = d.filtered_rpm.iloc[sel.start]
        tm, ym = simulate(ts, tgs, Kp, Ki, K, A, th, rpm0=n[sel][0], u0=n[sel][0] / K)
        ax[0, j].plot(ts, ns, color="#BBBBBB", lw=0.6, label="measured (raw)"); ax[0, j].plot(ts, tgs, "k-", lw=1, label="setpoint"); ax[0, j].plot(tm, ym, color=col, lw=1.2, label="model of the identified plant")
        ax[0, j].set_title(lab, fontsize=9); ax[0, j].set_xlabel("Time (s)"); ax[0, j].set_ylabel("Speed (rpm)"); ax[0, j].legend(frameon=False, fontsize=7, loc="lower right")
        # zoom on the first step of 250 rpm in the 500-1500 range
        tstep = ts[np.where(np.diff(tgs) != 0)[0][2] + 1]; zs = (ts >= tstep - 0.3) & (ts <= tstep + 1.6); zm = (tm >= tstep - 0.3) & (tm <= tstep + 1.6)
        ax[1, j].plot(ts[zs], ns[zs], color="#BBBBBB", lw=0.8); ax[1, j].plot(ts[zs], tgs[zs], "k-", lw=1); ax[1, j].plot(tm[zm], ym[zm], color=col, lw=1.6)
        ax[1, j].set_xlabel("Time (s)"); ax[1, j].set_ylabel("Speed (rpm)"); ax[1, j].set_title("Zoom on one 250 rpm step", fontsize=9)
        m = ts > 0; yi = np.interp(ts, tm, ym); res.append(dict(case=lab, rmse_rpm=float(np.sqrt(np.mean((yi[m] - pd.Series(ns).rolling(5, center=True, min_periods=1).mean().to_numpy()[m]) ** 2)))))
    fig.tight_layout(); save(fig, out, "fig_pi_step"); print(res)


# ------------------------------------------------------------------ 6. Kp / Ki / alpha matrix (174 mm, baffled)
def fig_gain_matrix(M, out):
    D = os.path.join(M, "matrix_ki_174mm_test"); rows = {}
    for f in sorted(glob.glob(os.path.join(D, "G*_r1.csv"))) + [os.path.join(D, "A_baseline_r1.csv")]:
        name = os.path.basename(f).split("_r1")[0]; d = pd.read_csv(f); t = d.elapsed_s.to_numpy(); n = d.filtered_rpm.to_numpy(); u = d.pwm_percent.to_numpy(); tg = d.target_rpm.to_numpy()
        if name.startswith("A_"): kp, ki, al = 2.629, 1.454, 1.0
        else:
            p = name.split("_"); kp, ki, al = float(p[1][2:]), float(p[2][2:]), float(p[3][1:])
        m1 = t <= 9.9; ov = 100 * (n[m1].max() - 300) / 300; i10, i90 = np.argmax(n >= 30), np.argmax(n >= 270); rise = t[i90] - t[i10] if n[m1].max() >= 270 else np.nan
        out_band = np.where(np.abs(n[m1] - 300) > 15)[0]; settle = t[m1][out_band[-1]] if len(out_band) else 0
        late = (t > 7) & (t < 9.9); jit = np.std(np.diff(u[late])); sd = np.std(n[late])
        step = (t > 10.0) & (t < 13.9); t340 = np.argmax(n[step] >= 340 * 0.9); rise340 = t[step][t340] - 10.0 if n[step].max() >= 306 else np.nan
        rows[name] = dict(kp=kp, ki=ki, alpha=al, overshoot=ov, rise=rise, settle=settle, sd=sd, jitter=jit, rise340=rise340, t=t, n=n, u=u)
    T = pd.DataFrame({k: {x: v[x] for x in ("kp", "ki", "alpha", "overshoot", "rise", "settle", "sd", "jitter", "rise340")} for k, v in rows.items()}).T; T.round(3).to_csv(os.path.join(out, "gain_matrix_metrics.csv")); print(T.round(2).to_string())
    fig, ax = plt.subplots(2, 3, figsize=(7.4, 4.9))
    show = [("A_baseline", "baseline: $K_i$=1.45, $\\alpha$=1", C["ol"]), ("G007_kp2.629_ki12_a1.0", "$K_i$=12, $\\alpha$=1", C["b"]), ("G009_kp2.629_ki12_a0.15", "$K_i$=12, $\\alpha$=0.15 (chosen)", C["a"])]
    for nm, lab, col in show:
        v = rows[nm]; ax[0, 0].plot(v["t"], v["n"], color=col, lw=1.2, label=lab); ax[0, 1].plot(v["t"], v["u"], color=col, lw=0.8)
    ax[0, 0].axhline(300, color="k", ls=":", lw=0.8); ax[0, 0].set_xlabel("Time (s)"); ax[0, 0].set_ylabel("Speed (rpm)"); ax[0, 0].set_title("(a) Start-up to 300 rpm, $K_p$ = 2.629"); ax[0, 0].legend(frameon=False, fontsize=7, loc="lower right")
    ax[0, 1].set_xlabel("Time (s)"); ax[0, 1].set_ylabel("Duty (%)"); ax[0, 1].set_title("(b) Duty command")
    for ax_, key, ttl, fmt in ((ax[0, 2], "overshoot", "(c) Start-up overshoot (%)", "{:.0f}"), (ax[1, 0], "settle", "(d) Settling time to $\\pm$5 % (s)", "{:.1f}"), (ax[1, 1], "jitter", "(e) Duty jitter (% per loop)", "{:.3f}"), (ax[1, 2], "sd", "(f) Speed SD at 300 rpm (rpm)", "{:.1f}")):
        sub = T[(T.kp == 2.629)]; kis = sorted(sub.ki.unique()); als = [1.0, 0.2, 0.15]; Z = np.array([[sub[(sub.ki == k) & (sub.alpha == a)][key].astype(float).mean() for a in als] for k in kis])
        im = ax_.imshow(Z, cmap="viridis_r", aspect="auto"); ax_.set_xticks(range(3), [f"{a:g}" for a in als]); ax_.set_yticks(range(len(kis)), [f"{k:g}" for k in kis]); ax_.set_xlabel(r"Feedback filter coefficient $\alpha$"); ax_.set_ylabel("$K_i$"); ax_.set_title(ttl, fontsize=8.8); ax_.grid(False)
        for (i, j), z in np.ndenumerate(Z): ax_.text(j, i, fmt.format(z), ha="center", va="center", color="white" if z > np.nanmean(Z) else "black", fontsize=8)
    fig.tight_layout(); save(fig, out, "fig_gain_matrix")


# ------------------------------------------------------------------ 7. disturbance rejection: open loop against PI
def fig_disturbance(R, out):
    runs = [("Open loop (constant duty)", "mixr1_log_20260918_121626_256ppr_20ms_15pwm_around350rpm_openlooppwm_around_370_for_baffles.csv", [49, 68, 87, 112, 144], 371, 386, C["ol"]),
            ("PI, earlier gains, nominal 0.5 rad/s", "mixr1_log_20260915_120201_256ppr_20ms_350rpm05radsforrealcomplete.csv", [47, 78, 107, 133, 158], 209, 235, C["a"]),
            ("PI, earlier gains, nominal 1.0 rad/s", "mixr1_log_20260914_185949_350rpm_1rads_pi.csv", [11, 27, 74, 94, 117], 147, 179, C["b"])]
    fig = plt.figure(figsize=(7.4, 6.2)); gs = fig.add_gridspec(3, 2, width_ratios=[1.5, 1]); rows = []; axo = None
    for i, (lab, f, drops, bi, br, col) in enumerate(runs):
        d = pd.read_csv(os.path.join(R, f)); t, n = d["t (s)"].to_numpy(), d["Filtered RPM"].to_numpy()
        A = fig.add_subplot(gs[i, 0])
        A.plot(t, d["Raw RPM"], color="#CCCCCC", lw=0.6); A.plot(t, n, color=col, lw=1.1)
        for x in drops: A.axvline(x, color="teal", ls=":", lw=0.9)
        A.axvline(bi, color="#E69F00", ls="--", lw=1); A.axvline(br, color="green", ls="--", lw=1); A.set_ylim(0, 430); A.set_ylabel("Speed (rpm)"); A.set_title(lab, fontsize=9, loc="left")
        if i == 0: A.plot([], [], color="teal", ls=":", label="water drop"); A.plot([], [], color="#E69F00", ls="--", label="baffles in"); A.plot([], [], color="green", ls="--", label="baffles out"); A.legend(frameon=False, ncol=3, loc="lower left", fontsize=7)
        if i == 2: A.set_xlabel("Time (s)")
        A.set_xlim(0, t[n > 50].max() + 10)
        B = fig.add_subplot(gs[i, 1])
        for x in drops:
            m = (t >= x - 3) & (t <= x + 20); pre = np.median(n[(t >= x - 3) & (t < x - 0.2)]); B.plot(t[m] - x, 100 * (n[m] / pre - 1), color=col, lw=0.9, alpha=0.8)
            post = n[(t > x) & (t < x + 12)]; dip = pre - np.percentile(post, 2); tt_ = t[(t > x) & (t <= x + 20)]; nn_ = pd.Series(n[(t > x) & (t <= x + 20)]).rolling(25, min_periods=1).mean().to_numpy()
            out_ = np.where(np.abs(nn_ - pre) > 0.05 * pre)[0]; trec = (tt_[out_[-1]] - x) if len(out_) else 0.0
            noise = pre - np.percentile(n[(t >= x - 3) & (t < x - 0.2)], 2); rows.append(dict(mode=lab, event='drop', noise_pct=100 * noise / pre, pre_rpm=pre, dip_rpm=dip, dip_pct=100 * dip / pre, rec_s=trec, final_pct=100 * (nn_[-1] / pre - 1)))
        B.axvline(0, color="teal", ls=":", lw=0.9); B.axhline(0, color="k", lw=0.5); B.set_xlim(-3, 20); B.set_ylim(-40, 15); B.set_xlabel("Time from the water drop (s)" if i == 2 else ""); B.set_ylabel("Speed change (%)"); B.set_title("five drops overlaid, relative to the speed before", fontsize=8.5, loc="left")
        for ev, xt in (("baffles in", bi), ("baffles out", br)):
            pre = np.median(n[(t >= xt - 3) & (t < xt - 0.2)]); post = pd.Series(n[(t > xt) & (t <= xt + 15)]).rolling(25, min_periods=1).mean().to_numpy(); tt_ = t[(t > xt) & (t <= xt + 15)]
            out_ = np.where(np.abs(post - pre) > 0.05 * pre)[0]; rows.append(dict(mode=lab, event=ev, noise_pct=np.nan, pre_rpm=pre, dip_rpm=pre - post.min(), dip_pct=100 * (pre - post.min()) / pre, rec_s=(tt_[out_[-1]] - xt) if len(out_) else 0.0, final_pct=100 * (post[-1] / pre - 1)))
    fig.tight_layout(); save(fig, out, "fig_disturbance"); T = pd.DataFrame(rows); S = T.groupby(["event", "mode"], sort=False).agg(pre_rpm=("pre_rpm", "mean"), dip_rpm=("dip_rpm", "mean"), dip_pct=("dip_pct", "mean"), noise_pct=("noise_pct", "mean"), rec_s=("rec_s", "mean"), final_pct=("final_pct", "mean")).round(2)
    S.to_csv(os.path.join(out, "disturbance_summary.csv")); print(S.to_string())


# ------------------------------------------------------------------ 8. torque sensor against known weights
def fig_loadcell(R, out):
    d = pd.read_csv(os.path.join(R, "Torque_Sensor Test", "loadcell_test_20261012_210846.csv")); d["applied"] = d.expected_torque_Nm; d["meas"] = d.reading - d.baseline_torque_Nm
    d["err"] = d.meas - d.applied; slope, icpt = np.polyfit(d.applied, d.meas, 1); rm = float(np.sqrt(np.mean(d.err ** 2))); r2 = np.corrcoef(d.applied, d.meas)[0, 1] ** 2
    fig, ax = plt.subplots(1, 3, figsize=(7.4, 2.8)); lim = d.applied.max() * 1.05
    ax[0].plot([0, lim], [0, lim], "k-", lw=0.9, label="1:1"); ax[0].plot(d.applied, d.meas, "o", color=C["a"], mec="white", mew=0.4, ms=4.5, label="trials"); ax[0].plot([0, lim], [icpt, icpt + slope * lim], "--", color=C["b"], lw=1, label=f"fit: slope {slope:.3f}")
    ax[0].set_xlabel("Applied torque (N$\\cdot$m)"); ax[0].set_ylabel("Measured torque (N$\\cdot$m)"); ax[0].set_title("(a) Measured against applied"); ax[0].legend(frameon=False)
    ax[1].axhline(0, color="k", lw=0.8); ax[1].plot(d.applied, 1000 * d.err, "o", color=C["a"], mec="white", mew=0.4, ms=4.5); ax[1].set_xlabel("Applied torque (N$\\cdot$m)"); ax[1].set_ylabel("Error (mN$\\cdot$m)"); ax[1].set_title(f"(b) Residual, RMSE {1000 * rm:.1f} mN$\\cdot$m")
    g = d.groupby("weight_g").agg(mean=("meas", "mean"), sd=("meas", "std"), app=("applied", "first"), n=("meas", "size")).reset_index(); g["rel"] = 100 * (g["mean"] - g.app) / g.app
    ax[2].axhline(0, color="k", lw=0.8); ax[2].errorbar(g.app, g.rel, yerr=100 * g.sd / g.app, fmt="s", color=C["a"], mec="white", mew=0.4, ms=5, capsize=2); ax[2].set_ylim(-2, 1); ax[2].set_xlabel("Applied torque (N$\\cdot$m)"); ax[2].set_ylabel("Relative error (%)"); ax[2].set_title("(c) Relative error per load")
    fig.tight_layout(); save(fig, out, "fig_loadcell"); print(f"loadcell: n={len(d)} slope {slope:.4f} intercept {icpt:.4f} R2 {r2:.5f} RMSE {rm:.4f} Nm, max |err| {d.err.abs().max():.4f}"); g.round(4).to_csv(os.path.join(out, "loadcell_summary.csv"), index=False)
    pd.DataFrame([dict(n=len(d), slope=slope, intercept=icpt, r2=r2, rmse=rm, maxerr=d.err.abs().max(), lo=d.applied.min(), hi=d.applied.max())]).to_csv(os.path.join(out, "loadcell_fit.csv"), index=False)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--root", required=True); ap.add_argument("--out", required=True); ap.add_argument("--only", default=""); a = ap.parse_args(); style(); os.makedirs(a.out, exist_ok=True)
    R, M = os.path.join(a.root, "Research SeBoCa Test"), os.path.join(a.root, "mixr1_modern_backend"); only = a.only.split(",") if a.only else []
    jobs = dict(duty=lambda: fig_duty_rpm(R, a.out), plant=lambda: fig_plant_id(M, a.out), bode=lambda: fig_bode_pm(a.out), design=lambda: fig_design_tradeoff(a.out),
                step=lambda: fig_pi_step(M, a.out), matrix=lambda: fig_gain_matrix(M, a.out), dist=lambda: fig_disturbance(R, a.out), load=lambda: fig_loadcell(R, a.out))
    for k, f in jobs.items():
        if not only or k in only:
            print("==", k)
            try: f()
            except Exception as e: import traceback; traceback.print_exc()

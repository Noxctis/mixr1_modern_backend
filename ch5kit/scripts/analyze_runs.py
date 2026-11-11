#!/usr/bin/env python3
"""
analyze_runs.py -- turns the logs of run_estimators.sh into every table and figure of the estimator comparison.

    python3 ch5kit/scripts/analyze_runs.py --dir logs/ch5_new/estimators

Reads   est-<variant>_<A|Bold|Bimc>_r<k>.csv   (variant = dct5, dct10, dct20, aligned, newest)
  A     open-loop PWM staircase (0..90 %, 10 s per step)
  Bold  closed loop, setpoints 10..1500 rpm (15 s each), the OLD gains Kp 1.5641 Ki 41.2249
  Bimc  closed loop, same setpoints, the gains computed from your own plant (IMC, 5 rad/s)
Writes to <dir>/results:  *.csv tables, *.tex table bodies, *.pdf figures and RESULTS.txt (the key numbers; send me this file).
"""
import argparse, glob, os, re, sys
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

FS, TS = 100.0, 0.010
NAME = re.compile(r"est-(?P<est>[a-z0-9]+)_(?P<test>A|Bold|Bimc)_r(?P<rep>\d+)\.csv$")
ORDER = ["dct5", "dct10", "dct20", "aligned", "newest"]
LABEL = {"dct5": "DCT 5 ms", "dct10": "DCT 10 ms", "dct20": "DCT 20 ms", "aligned": "CET aligned", "newest": "CET newest"}
COLOR = {"dct5": "#f4a582", "dct10": "#d6604d", "dct20": "#b2182b", "aligned": "#4393c3", "newest": "#053061"}
plt.rcParams.update({"font.size": 8.5, "font.family": "serif", "axes.grid": True, "grid.alpha": .3})


# ------------------------------------------------------------------ signal helpers
def fold(ft):
    return abs(ft - round(ft / FS) * FS)


def sinc_att(f):
    x = np.pi * f * TS
    return abs(np.sin(x) / x) if x else 1.0


def spectrum(x):
    x = x - x.mean(); w = np.hanning(len(x)); X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), 1 / FS)
    amp = 2 * np.abs(X) / np.sum(w)
    S = 2 * np.abs(X) ** 2 / (FS * np.sum(w ** 2)); S[0] /= 2
    return f, amp, S


def band_amp(f, S, fc, hw=0.5):
    return float(np.sqrt(2 * S[np.abs(f - fc) <= hw].sum() * (f[1] - f[0])))


def order_line(f, S, n, h):
    ft = h * n / 60.0; fa = fold(ft)
    if (ft > FS / 2 and fa < 4.0) or fa > 48.0:
        return np.nan
    return band_amp(f, S, fa)


def broadband(f, S, n):
    m = (f >= 1) & (f <= 48)
    for h in (1, 2, 3, 4):
        m &= ~(np.abs(f - fold(h * n / 60.0)) <= 1.0)
    return float(np.sqrt(S[m].sum() * (f[1] - f[0])))


def tex_table(df, path, fmt="{:.2f}"):
    cols = list(df.columns)
    with open(path, "w") as f:
        f.write("\\begin{tabular}{@{}" + "r" * len(cols) + "@{}}\n\\toprule\n" + " & ".join(str(c).replace("_", " ") for c in cols) + " \\\\\n\\midrule\n")
        for _, r in df.iterrows():
            f.write(" & ".join(("--" if (isinstance(v, float) and np.isnan(v)) else (fmt.format(v) if isinstance(v, (float, np.floating)) else str(v))) for v in r) + " \\\\\n")
        f.write("\\bottomrule\n\\end{tabular}\n")



# ------------------------------------------------------------------ independent reference: the index pulses
def load_index(csv_path):
    """<run>_index.csv is written by the daemon: one row per accepted index pulse (tick_us on the pigpio clock, A/B count)."""
    ip = csv_path[:-4] + "_index.csv"
    if not os.path.exists(ip): return None
    try:
        x = pd.read_csv(ip)
        return x if len(x) >= 3 else None
    except Exception:
        return None


def clock_rel(d):
    """(elapsed_s array, relative pigpio clock in us, origin tick) of the loop rows. Uses tick_now_us (--log-age) or, for the CET estimators,
    snap_tick. Returns None if neither is usable."""
    for col in ("tick_now_us", "snap_tick"):
        if col in d:
            v = d[col].to_numpy(float)
            m = np.isfinite(v) & (v > 0)
            if m.sum() >= 10:
                raw = v[m]; el = d.elapsed_s.to_numpy(float)[m]; t0 = int(raw[0])
                rel = np.cumsum(np.diff(((raw.astype(np.int64) - t0) & 0xFFFFFFFF), prepend=0) % (2 ** 32)).astype(float)
                return el, rel, t0
    return None


def idx_rel(idx, t0):
    ri = ((idx.tick_us.to_numpy(np.int64) - t0) & 0xFFFFFFFF).astype(float)
    return np.where(ri > 2 ** 31, ri - 2 ** 32, ri)


def idx_speed(d, idx, ta, tb):
    """Mean speed over [ta, tb] (s of the log) from the index pulses: (N-1) revolutions between the first and last pulse in the window.
    It does not depend on the quadrature decoding, on the resolution setting or on any estimator. NaN if the clock or the index log is missing."""
    if idx is None: return np.nan
    cm = clock_rel(d)
    if cm is None: return np.nan
    el, rel, t0 = cm
    ia, ib = np.interp(ta, el, rel), np.interp(tb, el, rel)
    ri = idx_rel(idx, t0); sel = ri[(ri >= ia) & (ri <= ib)]
    if len(sel) < 3: return np.nan
    return float((len(sel) - 1) * 60e6 / (sel[-1] - sel[0]))


def fold_ripple(d, idx, s, nmax_harm=2):
    """Speed ripple as a function of SHAFT ANGLE. Each loop sample is placed at its angle inside the revolution (0..1) using the index pulses
    before and after it; the mean is removed and cos/sin terms of the 1st and 2nd harmonic of the shaft angle are fitted. A ripple caused by the
    shaft or the mounting is locked to the angle: same amplitude and the same phase against the index, revolution after revolution. Returns
    dict(A1, ph1, A2, ph2, nrev) or None."""
    if idx is None: return None
    cm = clock_rel(d)
    if cm is None: return None
    el, rel, t0 = cm
    ts = np.interp(s.elapsed_s.to_numpy(float), el, rel); ri = np.sort(idx_rel(idx, t0))
    k = np.searchsorted(ri, ts) - 1
    ok = (k >= 0) & (k < len(ri) - 1)
    if ok.sum() < 200: return None
    t_prev, t_next = ri[k[ok]], ri[k[ok] + 1]; per = t_next - t_prev
    good = np.abs(per - np.median(per)) < 0.15 * np.median(per)
    if good.sum() < 200: return None
    phi = ((ts[ok] - t_prev) / per)[good]; y = np.abs(s.raw_rpm.to_numpy(float))[ok][good]; y = y - y.mean()
    cols = [np.ones_like(phi)]
    for h in range(1, nmax_harm + 1): cols += [np.cos(2 * np.pi * h * phi), np.sin(2 * np.pi * h * phi)]
    X = np.column_stack(cols); c, *_ = np.linalg.lstsq(X, y, rcond=None)
    out = {"nrev": int(len(np.unique(k[ok][good])))}
    for h in range(1, nmax_harm + 1):
        a, b = c[2 * h - 1], c[2 * h]; out[f"A{h}"] = float(np.hypot(a, b)); out[f"ph{h}"] = float(np.degrees(np.arctan2(b, a)))
    out["phi"], out["y"] = phi, y
    return out


def spike_stats(d):
    """Samples of the raw speed that are far from the local (1 s rolling) median. Returns (samples, off20, off50, max_ratio). Only where the speed is above 100 rpm."""
    x = np.abs(d.raw_rpm.to_numpy(float)); med = pd.Series(x).rolling(101, center=True, min_periods=50).median().to_numpy()
    m = np.isfinite(med) & (med > 100)
    if m.sum() < 100: return 0, 0, 0, np.nan
    r = x[m] / med[m]
    return int(m.sum()), int((np.abs(r - 1) > 0.2).sum()), int((np.abs(r - 1) > 0.5).sum()), float(r.max())

FOLD = {}


# ------------------------------------------------------------------ per-run analysis
def load(path):
    d = pd.read_csv(path)
    d.columns = [c.strip() for c in d.columns]
    return d


def analyse_run(path, est, test, rep):
    d = load(path)
    idx = load_index(path)
    steps, run_rows = [], []
    for i, g in d.groupby("step_index"):
        t = g.elapsed_s.to_numpy()
        hold = t[-1] - t[0]
        if hold < 4: continue
        m = (t >= t[0] + max(1.0, 0.3 * hold)) & (t <= t[0] + 0.97 * hold)
        s = g[m]
        if len(s) < 100: continue
        raw = np.abs(s.raw_rpm.to_numpy(float)); n = raw.mean()
        ref = idx_speed(d, idx, s.elapsed_s.iloc[0], s.elapsed_s.iloc[-1])
        row = dict(est=est, idx_rpm=ref, bias=(n - ref) if np.isfinite(ref) else np.nan, bias_pct=100 * (n - ref) / ref if np.isfinite(ref) and ref > 0 else np.nan, test=test, rep=rep, step=int(i), pwm_pct=float(g.pwm_percent.mean()), target=float(s.target_rpm.mean()), n=n,
                   sd=float(raw.std(ddof=1)), err_rms=float(np.sqrt(np.nanmean(s.error_rpm.to_numpy(float) ** 2))) if "error_rpm" in s else np.nan)
        if "new_snap" in s and s.new_snap.notna().any() and est not in ("dct5", "dct10", "dct20"):
            row["new_frac"] = float(s.new_snap.astype(float).mean())
        elif "new_snap" in s and s.new_snap.notna().any():
            row["new_frac"] = float(s.new_snap.astype(float).mean())
        else:
            row["new_frac"] = float(np.mean(np.diff(raw) != 0))
        if test == "A" and n > 150 and len(raw) >= 400:
            f, amp, S = spectrum(raw)
            a1, a2, a3 = order_line(f, S, n, 1), order_line(f, S, n, 2), order_line(f, S, n, 3)
            fa = [fold(h * n / 60.0) for h in (1, 2, 3)]                       # a higher order that aliases onto a lower one is not counted twice
            if abs(fa[1] - fa[0]) < 1.0: a2 = np.nan
            if abs(fa[2] - fa[0]) < 1.0 or abs(fa[2] - fa[1]) < 1.0: a3 = np.nan
            sdv = raw.std(ddof=1)
            row.update(A1=a1, A2=a2, A3=a3, noise=broadband(f, S, n), rel1_pct=100 * a1 / n,
                       eps1_deg=np.degrees((a1 / max(sinc_att(n / 60.0), 1e-3)) / n) if np.isfinite(a1) else np.nan,
                       share_pct=100 * np.nansum([a1 ** 2 / 2, a2 ** 2 / 2, a3 ** 2 / 2]) / sdv ** 2)
            if 100 < n <= 800:
                fr = fold_ripple(d, idx, s)
                if fr: row.update(f1=fr["A1"], fph1=fr["ph1"], f2=fr["A2"], fph2=fr["ph2"], fnrev=fr["nrev"])
                if fr: FOLD[(est, rep, int(i))] = (n, fr["phi"], fr["y"])
        if "age_us" in s and s.age_us.notna().any() and est in ("aligned", "newest"):
            a_raw = s.age_us.to_numpy(float)
            a = a_raw - (0.5 * s.tick_rtt_us.to_numpy(float) if "tick_rtt_us" in s and s.tick_rtt_us.notna().any() else 0.0)   # remove half of the clock-request round trip
            a = a[np.isfinite(a)]
            if len(a) > 20:
                row.update(age_med=float(np.median(a)), age_p95=float(np.percentile(a, 95)), age_max=float(a.max()), age_mean=float(a.mean()),
                           age_raw_med=float(np.nanmedian(a_raw)), rtt_med=float(np.nanmedian(s.tick_rtt_us)) if "tick_rtt_us" in s else np.nan)
        steps.append(row)
    lp = d.loop_period_us.to_numpy(float)[1:] if "loop_period_us" in d else np.array([np.nan])
    la = d.late_us.to_numpy(float) if "late_us" in d else np.array([np.nan])
    timing = dict(est=est, test=test, rep=rep, period_p99=float(np.nanpercentile(lp, 99)), period_max=float(np.nanmax(lp)),
                  late_p99=float(np.nanpercentile(la, 99)), late_max=float(np.nanmax(la)), quad_err=float(d.quad_err.iloc[-1]) if "quad_err" in d else np.nan)
    ns, o20, o50, mx = spike_stats(d)
    timing.update(spike_n=ns, spike_off20=o20, spike_off50=o50, spike_maxratio=mx)
    return steps, timing, d


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", required=True); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(a.dir, "results"); os.makedirs(out, exist_ok=True)
    files = sorted(glob.glob(os.path.join(a.dir, "est-*.csv")))
    runs = []
    for f in files:
        m = NAME.search(os.path.basename(f))
        if m: runs.append((f, m["est"], m["test"], int(m["rep"])))
    if not runs:
        sys.exit("No files named est-<variant>_<A|Bold|Bimc>_r<k>.csv found in " + a.dir)
    S, T, RAW = [], [], {}
    for f, est, test, rep in runs:
        try:
            st, tm, d = analyse_run(f, est, test, rep)
        except Exception as e:
            print(f"skipped {os.path.basename(f)}: {e}"); continue
        S += st; T.append(tm); RAW[(est, test, rep)] = d
    S = pd.DataFrame(S); T = pd.DataFrame(T)
    ests = [e for e in ORDER if e in set(S.est)]
    lines = [f"Files analysed: {len(runs)}   estimators: {', '.join(ests)}   repeats: {sorted(set(r[3] for r in runs))}", ""]

    # ---------------- runs that did not finish, and stalls
    exp = {}
    for (est, test, rep), d in RAW.items(): exp[test] = max(exp.get(test, 0), int(d.step_index.max()))
    st = []
    for f, est, test, rep in runs:
        if (est, test, rep) not in RAW: continue
        d = RAW[(est, test, rep)]; last = int(d.step_index.max()); why = ""
        sp = f[:-4] + ".stdout.txt"
        if os.path.exists(sp):
            txt = open(sp, errors="ignore").read()
            hit = [ln.strip() for ln in txt.splitlines() if re.search(r"ABORT|TRIP|STALL|SAFETY|CRITICAL|Cannot", ln)]
            why = hit[0][:110] if hit else ""
        lp = d.loop_period_us.to_numpy(float)[1:] if "loop_period_us" in d else np.array([0.0])
        big = np.flatnonzero(lp > 100000)
        stall = f"stall {lp[big].max() / 1000:.0f} ms at t={d.elapsed_s.to_numpy()[big[np.argmax(lp[big])] + 1]:.1f} s" if len(big) else ""
        if last < exp[test] or why or stall:
            st.append(dict(run=os.path.basename(f)[:-4], last_step=f"{last} of {exp[test]}", message=why, loop_stall=stall))
    if st:
        lines += ["RUNS THAT ENDED EARLY OR HAD A STALL (steps missing from the tables below come from these runs)", pd.DataFrame(st).to_string(index=False), ""]
    else:
        lines += ["All runs finished all their steps with no abort and no stall.", ""]

    # ---------------- open loop
    A = S[(S.test == "A") & (S.n > 100)]
    if len(A):
        g = A.groupby(["est", "pwm_pct"]).agg(n=("n", "mean"), sd=("sd", "mean"), sd_min=("sd", "min"), sd_max=("sd", "max"), new_frac=("new_frac", "mean"),
                                              A1=("A1", "mean"), A2=("A2", "mean"), noise=("noise", "mean")).reset_index()
        g.to_csv(os.path.join(out, "open_loop_by_estimator.csv"), index=False)
        piv = g.pivot(index="pwm_pct", columns="est", values="sd")[ests]
        piv.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
        piv = piv.reset_index().rename(columns={"pwm_pct": "PWM %"}); piv.to_csv(os.path.join(out, "table_open_loop_sd.csv"), index=False)
        tex_table(piv.rename(columns=LABEL), os.path.join(out, "table_open_loop_sd.tex"))
        lines += ["OPEN LOOP: SD of the raw speed (rpm), mean over repeats", piv.round(2).to_string(index=False), ""]
        lines += ["  mean SD over the steps from 30 % PWM: " + ", ".join(f"{LABEL[e]} {g[(g.est == e) & (g.pwm_pct >= 30)].sd.mean():.2f}" for e in ests), ""]
        fig, ax = plt.subplots(figsize=(4.8, 3.2))
        for e in ests:
            q = g[g.est == e]; ax.plot(q.n, q.sd, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e]); ax.fill_between(q.n, q.sd_min, q.sd_max, color=COLOR[e], alpha=.15)
        ax.set_yscale("log"); ax.set_xlabel("Speed (rpm)"); ax.set_ylabel("SD of the raw speed (rpm)"); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_openloop_sd.pdf")); plt.close(fig)
        fig, ax = plt.subplots(figsize=(4.8, 3.0))
        for e in ests:
            q = g[g.est == e]; ax.plot(q.n, q.new_frac, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e])
        ax.set_xlabel("Speed (rpm)"); ax.set_ylabel("Fraction of loops with a new estimate"); ax.set_ylim(0, 1.05); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_refresh_fraction.pdf")); plt.close(fig)
        h = g.pivot(index="pwm_pct", columns="est", values="A1")[ests]; h.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
        h.reset_index().to_csv(os.path.join(out, "table_line_1x.csv"), index=False)
        lines += ["AMPLITUDE OF THE LINE AT THE ROTATION FREQUENCY (rpm), open loop", h.round(2).reset_index().to_string(index=False), ""]
        # spectra at four steps
        picks = [p for p in (20, 40, 70, 90) if p in set(A.pwm_pct.round())]
        if picks:
            fig, axs = plt.subplots(2, 2, figsize=(6.4, 4.4), sharey=True)
            for ax, p in zip(axs.ravel(), picks):
                for e in ests:
                    specs = []
                    for (ee, tt, rr), d in RAW.items():
                        if ee != e or tt != "A": continue
                        for i, gg in d.groupby("step_index"):
                            if abs(gg.pwm_percent.mean() - p) < 1:
                                t = gg.elapsed_s.to_numpy(); hold = t[-1] - t[0]; m = (t >= t[0] + max(1, .3 * hold)) & (t <= t[0] + .97 * hold)
                                x = gg.raw_rpm.to_numpy(float)[m][:600]
                                if len(x) >= 600: f, amp, _ = spectrum(x); specs.append(amp)
                    if specs: ax.semilogy(f, np.clip(np.mean(specs, 0), 1e-3, None), color=COLOR[e], lw=.7, label=LABEL[e])
                ax.set_title(f"{p} % PWM", fontsize=8); ax.set_xlim(0, 50); ax.set_ylim(1e-2, 20)
            for ax in axs[1]: ax.set_xlabel("Frequency (Hz)")
            for ax in axs[:, 0]: ax.set_ylabel("Amplitude (rpm)")
            axs[0, 0].legend(fontsize=6.5); fig.tight_layout(); fig.savefig(os.path.join(out, "fig_openloop_spectra.pdf")); plt.close(fig)

    # ---------------- the shaft: does a once-per-revolution error ride on the encoder reading?
    if len(A) and "eps1_deg" in A and A.eps1_deg.notna().any():
        sh = A.groupby(["est", "pwm_pct"]).agg(n=("n", "mean"), A1=("A1", "mean"), A2=("A2", "mean"), rel1_pct=("rel1_pct", "mean"), eps1_deg=("eps1_deg", "mean"),
                                               share_pct=("share_pct", "mean")).reset_index()
        sh.to_csv(os.path.join(out, "shaft_ripple_fft.csv"), index=False)
        L1 = sh[sh.est == ("aligned" if "aligned" in set(sh.est) else sh.est.iloc[0])][["n", "A1", "A2", "rel1_pct", "eps1_deg", "share_pct"]]
        lines += ["SHAFT / ECCENTRICITY (FFT of the raw speed, open loop). A1, A2 = amplitude (rpm) of the lines at 1x and 2x the rotation frequency;",
                  "  rel1 = A1/n in %; eps1 = the equivalent angular error of the encoder in degrees if the 1x line were all encoder error (datasheet accuracy 0.25 deg);",
                  "  share = % of the variance of the raw speed carried by the lines at 1x, 2x, 3x. First estimator shown; the others are in shaft_ripple_fft.csv.",
                  "  HOW TO READ IT: an ANGLE error of the encoder (shaft not concentric) gives a relative amplitude rel1 that stays the SAME at every speed.",
                  "  A true speed ripple of the shaft (a load that repeats each turn) gives an amplitude in rpm that FALLS as the speed rises.", L1.round(3).to_string(index=False), ""]
        fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.9))
        for e in ests:
            q = sh[sh.est == e]
            ax[0].plot(q.n, q.A1, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e]); ax[1].plot(q.n, q.eps1_deg, "o-", ms=3, lw=1, color=COLOR[e])
        ax[0].set_xlabel("Speed (rpm)"); ax[0].set_ylabel("Line at 1x (rpm)"); ax[0].set_yscale("log"); ax[0].legend(fontsize=6.5)
        ax[1].axhline(0.25, color="k", ls="--", lw=.8); ax[1].text(ax[1].get_xlim()[1] * .5, 0.27, "datasheet 0.25 deg", fontsize=7)
        ax[1].set_yscale("log"); ax[1].set_xlabel("Speed (rpm)"); ax[1].set_ylabel("Equivalent encoder angle error (deg)")
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_shaft_ripple.pdf")); plt.close(fig)
    if len(A) and "f1" in A and A.f1.notna().any():
        ft = A.groupby(["est", "pwm_pct"]).agg(n=("n", "mean"), fold_A1=("f1", "mean"), fold_phase1_deg=("fph1", "mean"), fold_A2=("f2", "mean"), fold_phase2_deg=("fph2", "mean"),
                                               revs=("fnrev", "mean")).reset_index()
        ft.to_csv(os.path.join(out, "shaft_ripple_folded.csv"), index=False)
        lines += ["RIPPLE VERSUS SHAFT ANGLE (samples folded onto one revolution with the index pulses; speeds 100 to 800 rpm).",
                  "  A ripple that belongs to the shaft or to the mounting has the SAME phase against the index at every speed and every repeat.",
                  ft[ft.est == ft.est.iloc[0]].round(2).to_string(index=False), ""]
        for e in sorted(set(ft.est)):
            q = ft[(ft.est == e) & ft.fold_phase1_deg.notna()].sort_values("n")
            if len(q) >= 3:
                fr_ = q.n.to_numpy() / 60.0; ph = np.degrees(np.unwrap(np.radians(q.fold_phase1_deg.to_numpy())))
                sl, ic = np.polyfit(fr_, ph, 1)
                lines.append(f"  {LABEL.get(e, e)}: the phase of the 1x ripple advances {sl:.2f} deg per Hz of rotation, i.e. the speed estimate lags the shaft angle by {sl / 360 * 1000:.1f} ms"
                             f" (CET is expected near T_s/2 + age = 5 to 6 ms, DCT 12 to 25 ms); phase extrapolated to zero speed {ic:.0f} deg (the angle at which the shaft makes the speed highest).")
        lines.append("")
        picks = sorted(set(int(k[2]) for k in FOLD))[:3]
        if FOLD:
            keys = sorted(FOLD, key=lambda k: FOLD[k][0])
            fig, axs = plt.subplots(1, 3, figsize=(6.8, 2.4), sharey=False)
            sel = [keys[0], keys[len(keys) // 2], keys[-1]] if len(keys) >= 3 else keys
            for ax, k in zip(axs, sel):
                n, phi, y = FOLD[k]; b = np.linspace(0, 1, 17); c = 0.5 * (b[1:] + b[:-1]); m = [y[(phi >= b[j]) & (phi < b[j + 1])].mean() if ((phi >= b[j]) & (phi < b[j + 1])).any() else np.nan for j in range(16)]
                ax.plot(phi, y, ".", ms=1, alpha=.25, color="gray"); ax.plot(c, m, "o-", ms=3, color="tab:red"); ax.set_title(f"{n:.0f} rpm ({LABEL.get(k[0], k[0])})", fontsize=7.5); ax.set_xlabel("Shaft angle (turns, 0 = index)")
            axs[0].set_ylabel("Speed minus its mean (rpm)"); fig.tight_layout(); fig.savefig(os.path.join(out, "fig_shaft_folded.pdf")); plt.close(fig)

    # ---------------- closed loop
    for test, title in (("Bold", "old gains (Kp 1.5641, Ki 41.2249)"), ("Bimc", "gains from your plant (IMC)")):
        B = S[S.test == test]
        if not len(B): continue
        g = B.groupby(["est", "target"]).agg(sd=("sd", "mean"), sd_min=("sd", "min"), sd_max=("sd", "max"), err_rms=("err_rms", "mean"), new_frac=("new_frac", "mean")).reset_index()
        g.to_csv(os.path.join(out, f"closed_loop_{test}.csv"), index=False)
        piv = g.pivot(index="target", columns="est", values="sd")[[e for e in ests if e in set(g.est)]].reset_index().rename(columns={"target": "setpoint rpm"})
        piv.to_csv(os.path.join(out, f"table_closed_loop_{test}.csv"), index=False); tex_table(piv.rename(columns=LABEL), os.path.join(out, f"table_closed_loop_{test}.tex"))
        lines += [f"CLOSED LOOP, {title}: SD of the raw speed (rpm), mean over repeats", piv.round(2).to_string(index=False), ""]
        if "aligned" in set(g.est):
            for e in [x for x in ests if x.startswith("dct")]:
                r = g[g.est == e].set_index("target").sd / g[g.est == "aligned"].set_index("target").sd
                lines.append(f"  {LABEL[e]} / CET aligned (SD ratio): min {r.min():.1f}, max {r.max():.1f}, at 1500 rpm {r.get(1500.0, float('nan')):.1f}")
            lines.append("")
    if any(S.test.isin(["Bold", "Bimc"])):
        fig, axs = plt.subplots(2, 2, figsize=(6.6, 5.0))
        for row, test in enumerate(("Bold", "Bimc")):
            B = S[S.test == test]
            if not len(B): continue
            g = B.groupby(["est", "target"]).agg(sd=("sd", "mean"), lo=("sd", "min"), hi=("sd", "max")).reset_index()
            for e in ests:
                q = g[g.est == e]
                if len(q): axs[row, 0].semilogy(q.target, q.sd, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e]); axs[row, 0].fill_between(q.target, q.lo, q.hi, color=COLOR[e], alpha=.15)
            axs[row, 0].set_xlabel("Setpoint (rpm)"); axs[row, 0].set_ylabel("SD of the raw speed (rpm)"); axs[row, 0].set_title("old gains" if test == "Bold" else "IMC gains", fontsize=8)
            for e in ests:
                specs = []
                for (ee, tt, rr), d in RAW.items():
                    if ee != e or tt != test: continue
                    top = d.step_index.max(); gg = d[d.step_index == top]; t = gg.elapsed_s.to_numpy(); hold = t[-1] - t[0]
                    x = gg.raw_rpm.to_numpy(float)[(t >= t[0] + 4) & (t <= t[0] + .97 * hold)][:900]
                    if len(x) >= 600: f, amp, _ = spectrum(x[:600]); specs.append(amp)
                if specs: axs[row, 1].semilogy(f, np.clip(np.mean(specs, 0), 1e-3, None), color=COLOR[e], lw=.7, label=LABEL[e])
            axs[row, 1].set_xlim(0, 50); axs[row, 1].set_xlabel("Frequency (Hz)"); axs[row, 1].set_ylabel("Amplitude (rpm)"); axs[row, 1].set_title("spectrum at the highest setpoint", fontsize=8)
        axs[0, 0].legend(fontsize=6.5); fig.tight_layout(); fig.savefig(os.path.join(out, "fig_closedloop.pdf")); plt.close(fig)

    # ---------------- accuracy against the index reference
    if "bias_pct" in S and S.bias_pct.notna().any():
        Q = S[(S.test.isin(["A", "Bold", "Bimc"])) & S.idx_rpm.notna() & (S.idx_rpm > 100)].copy()
        Q["abs_bias_pct"] = Q.bias_pct.abs()
        qa = Q.groupby(["est", "test"]).agg(steps=("n", "count"), mean_bias_pct=("bias_pct", "mean"), mean_abs_bias_pct=("abs_bias_pct", "mean"),
                                            max_abs_bias_pct=("abs_bias_pct", "max"), mean_bias_rpm=("bias", "mean")).reset_index()
        qa.to_csv(os.path.join(out, "accuracy_vs_index.csv"), index=False); tex_table(qa.round(3), os.path.join(out, "table_accuracy.tex"), "{:.3f}")
        lines += ["ACCURACY: the mean of each estimator over a step minus the speed from the index pulses (independent of every estimator), speeds above 100 rpm",
                  "  bias_pct = 100*(estimator - index)/index.  An error of x % in speed gives about 2x % in the Power number (N_Po ~ torque / n^2).",
                  qa.round(3).to_string(index=False), ""]
        z = Q[Q.test == "A"].groupby(["est", "pwm_pct"]).bias_pct.mean().reset_index().pivot(index="pwm_pct", columns="est", values="bias_pct")
        z = z[[e for e in ests if e in z.columns]]; z.insert(0, "speed_rpm", Q[Q.test == "A"].groupby("pwm_pct").n.mean())
        z.reset_index().to_csv(os.path.join(out, "table_bias_open_loop.csv"), index=False)
        lines += ["BIAS (%) vs the index reference, open loop, by PWM step", z.round(3).reset_index().to_string(index=False), ""]
        fig, ax = plt.subplots(figsize=(4.8, 3.2))
        for e in ests:
            q = Q[(Q.test == "A") & (Q.est == e)].groupby("pwm_pct").agg(n=("n", "mean"), b=("bias_pct", "mean"), lo=("bias_pct", "min"), hi=("bias_pct", "max"))
            if len(q): ax.plot(q.n, q.b, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e]); ax.fill_between(q.n, q.lo, q.hi, color=COLOR[e], alpha=.15)
        ax.axhline(0, color="k", lw=.6); ax.set_xlabel("Speed (rpm)"); ax.set_ylabel("Bias against the index speed (%)"); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_accuracy_bias.pdf")); plt.close(fig)
    else:
        lines += ["ACCURACY: no index reference available (needs tick_now_us in the log, i.e. --log-age, and the *_index.csv files).", ""]

    # ---------------- age of the estimate (CET only)
    G = S[S.est.isin(["aligned", "newest"]) & S.test.isin(["Bold", "Bimc"]) & S.get("age_med", pd.Series(dtype=float)).notna()] if "age_med" in S else pd.DataFrame()
    if len(G):
        G = G[G.target.isin([100, 250, 350, 500, 1000, 1500])]
        g = G.groupby(["est", "target"]).agg(age_med_us=("age_med", "mean"), age_p95_us=("age_p95", "mean"), age_max_us=("age_max", "max"),
                                             age_raw_med_us=("age_raw_med", "mean"), clock_rtt_us=("rtt_med", "mean")).reset_index()
        g["Ta_us"] = 60e6 / (g.target * 256.0)
        g["predicted_max_us"] = np.where(g.est == "aligned", g.Ta_us, g.Ta_us / 4)
        g.to_csv(os.path.join(out, "age_of_estimate.csv"), index=False); tex_table(g.round(1), os.path.join(out, "table_age.tex"), "{:.1f}")
        lines += ["AGE OF THE ESTIMATE (us): the daemon clock minus the time stamp of the snapshot in use, minus half of the clock-request round trip (tick_rtt_us).",
                  "  age_raw_med_us is before that correction. If the corrected age is still far above the prediction at high speed, the rest is the delay of the pigpio notification stream.",
                  "  predicted maximum: aligned = Ta, newest = Ta/4, with Ta = 60e6/(speed*256) us", g.round(1).to_string(index=False), ""]
    else:
        lines += ["AGE OF THE ESTIMATE: no age_us data found (the CET runs need --log-age; run_estimators.sh adds it).", ""]

    # ---------------- outliers
    if len(T) and "spike_n" in T:
        sp = T.groupby("est").agg(samples=("spike_n", "sum"), off_20pct=("spike_off20", "sum"), off_50pct=("spike_off50", "sum"), max_ratio_to_median=("spike_maxratio", "max")).reindex(ests).reset_index()
        sp["off_50pct_per_million"] = (1e6 * sp.off_50pct / sp.samples.replace(0, np.nan)).round(0)
        sp.to_csv(os.path.join(out, "outliers_by_estimator.csv"), index=False)
        lines += ["OUTLIERS: samples of the raw speed far from the local median (speeds above 100 rpm). max_ratio = the largest sample divided by the local median.", sp.round(2).to_string(index=False), ""]

    # ---------------- timing
    if len(T):
        tt = T.groupby("est")[["period_p99", "period_max", "late_p99", "late_max", "quad_err"]].agg({"period_p99": "mean", "period_max": "max", "late_p99": "mean", "late_max": "max", "quad_err": "max"}).reindex(ests).reset_index()
        tt.to_csv(os.path.join(out, "timing_by_estimator.csv"), index=False)
        lines += ["LOOP TIMING per estimator (us) and lost edges (quad_err, must be 0)", tt.round(1).to_string(index=False), ""]
    # ---------------- which CET to use
    try:
        sd = {e: S[(S.est == e) & (S.test == "A") & (S.pwm_pct >= 30)].sd.mean() for e in ("aligned", "newest")}
        ag = {e: S[(S.est == e) & S.test.isin(["Bold", "Bimc"]) & (S.target == 500)].age_p95.mean() for e in ("aligned", "newest")} if "age_p95" in S else {}
        qe = {e: T[T.est == e].quad_err.sum() for e in ("aligned", "newest")}
        reps = len(set(r[3] for r in runs))
        if all(np.isfinite(list(sd.values()))) and ag and all(np.isfinite(list(ag.values()))):
            lost_ok = (not np.isfinite(qe["newest"])) or qe["newest"] <= qe["aligned"] + 2
            use_new = (ag["newest"] < 0.7 * ag["aligned"]) and (sd["newest"] <= 1.1 * sd["aligned"]) and lost_ok
            lines += [f"RECOMMENDED ESTIMATOR: {'newest' if use_new else 'aligned'}",
                      f"  SD aligned {sd['aligned']:.2f} / newest {sd['newest']:.2f} rpm; age p95 at 500 rpm aligned {ag['aligned']:.0f} / newest {ag['newest']:.0f} us; "
                      f"lost edges over all runs: aligned {qe['aligned']:g}, newest {qe['newest']:g}",
                      "  rule: newest only if its age is clearly smaller (under 70 %), its scatter is not worse, and it lost no more edges than aligned (+2). A lost edge is a property of the",
                      "  encoder callbacks and of system stalls, not of the estimator, so a single one is not held against either.",
                      ("  WARNING: only one repeat: differences of 10 to 20 % are inside the run-to-run scatter, so treat both CET versions as equivalent." if reps < 2 else ""),
                      "  to use it:   bash /home/pi/mixr1_modern_backend/ch5kit/ch5.sh use " + ("newest" if use_new else "aligned"), ""]
    except Exception as ex:
        lines += [f"(no recommendation: {ex})", ""]
    txt = re.sub(r"\bNaN\b", "n/a", "\n".join(lines))
    open(os.path.join(out, "RESULTS.txt"), "w").write(txt)
    print(txt); print(f"\nAll tables (.csv, .tex) and figures (.pdf) are in {out}\nSend me {os.path.join(out, 'RESULTS.txt')} and the figures.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
analyze_ppr.py -- tables and figures for the PPR (encoder resolution) study, from the logs of ppr_campaign.py.
Use this if analyze_ppr_study.py is not in your folder (or as a second opinion).

    python3 ch5kit/scripts/analyze_ppr.py --dir logs/ppr_YYYYMMDD

Reads  mixr1_log_*_<PPR>ppr_<cet|cetn>_r<k>_test<A|B>.csv  (A = open-loop staircase, B = closed-loop staircase) and the matching
.pidstat.txt files, and writes to <dir>/results_simple: tables (.csv/.tex), figures (.pdf) and RESULTS.txt (send me that file).
"""
import argparse, glob, os, re, sys
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_runs import spectrum, order_line, broadband, tex_table, load, load_index, idx_speed   # noqa: E402

NAME = re.compile(r"(?P<ppr>\d+)ppr_(?P<tok>cet|cetn)_r(?P<rep>\d+)_test(?P<test>[AB])\.csv$")
plt.rcParams.update({"font.size": 8.5, "font.family": "serif", "axes.grid": True, "grid.alpha": .3})


def cpu_from_pidstat(path):
    """Best effort: mean %CPU per command name from a pidstat file (sum over its threads). Returns {} if the format is not recognised."""
    if not os.path.exists(path): return {}
    hdr, acc = None, {}
    for ln in open(path, errors="ignore"):
        p = ln.split()
        if "%CPU" in p and "Command" in p:
            hdr = p; continue
        if hdr and len(p) == len(hdr):
            try:
                cpu = float(p[hdr.index("%CPU")])
            except ValueError:
                continue
            acc.setdefault(p[-1], []).append(cpu)
    return {k: float(np.mean(v)) for k, v in acc.items()}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", required=True); a = ap.parse_args()
    out = os.path.join(a.dir, "results_simple"); os.makedirs(out, exist_ok=True)
    rows, runs, timing, cpu = [], [], [], []
    for f in sorted(glob.glob(os.path.join(a.dir, "*.csv"))):
        m = NAME.search(os.path.basename(f))
        if not m: continue
        ppr, rep, test = int(m["ppr"]), int(m["rep"]), m["test"]
        d = load(f); idx = load_index(f)
        for i, g in d.groupby("step_index"):
            t = g.elapsed_s.to_numpy(); hold = t[-1] - t[0]
            if hold < 4: continue
            s = g[(t >= t[0] + max(1.0, 0.3 * hold)) & (t <= t[0] + 0.97 * hold)]
            if len(s) < 100: continue
            raw = np.abs(s.raw_rpm.to_numpy(float)); n = raw.mean()
            ref = idx_speed(d, idx, s.elapsed_s.iloc[0], s.elapsed_s.iloc[-1])
            r = dict(ppr=ppr, idx_rpm=ref, bias_pct=100 * (n - ref) / ref if np.isfinite(ref) and ref > 0 else np.nan, rep=rep, test=test, step=int(i), pwm_pct=float(g.pwm_percent.mean()), target=float(s.target_rpm.mean()), n=n, sd=float(raw.std(ddof=1)),
                     err_rms=float(np.sqrt(np.nanmean(s.error_rpm.to_numpy(float) ** 2))) if "error_rpm" in s else np.nan,
                     new_frac=float(s.new_snap.astype(float).mean()) if "new_snap" in s and s.new_snap.notna().any() else float(np.mean(np.diff(raw) != 0)),
                     zero_frac=float(np.mean(raw == 0)))
            if test == "A" and n > 150 and len(raw) >= 600:
                f_, amp, S = spectrum(raw); r.update(A1=order_line(f_, S, n, 1), noise=broadband(f_, S, n))
            rows.append(r)
        revs = abs(float(d.revs.iloc[-1])) if "revs" in d else np.nan
        runs.append(dict(ppr=ppr, rep=rep, test=test, edge_total=float(d.edge_total.iloc[-1]) if "edge_total" in d else np.nan, revs=revs,
                         quad_err=float(d.quad_err.iloc[-1]) if "quad_err" in d else np.nan, idx_rej=float(d.idx_rej.iloc[-1]) if "idx_rej" in d else np.nan,
                         idx_miss=float(d.idx_miss.iloc[-1]) if "idx_miss" in d else np.nan,
                         period_p99=float(np.nanpercentile(d.loop_period_us.to_numpy(float)[1:], 99)), late_max=float(d.late_us.max())))
        c = cpu_from_pidstat(f[:-4] + ".pidstat.txt")
        if c: cpu.append(dict(ppr=ppr, rep=rep, test=test, **{k: v for k, v in c.items() if "mixr" in k or "pigpio" in k}))
    if not rows: sys.exit("No campaign files found in " + a.dir)
    S, R = pd.DataFrame(rows), pd.DataFrame(runs)
    pprs = [int(p) for p in sorted(S.ppr.unique())]; col = dict(zip(pprs, plt.cm.viridis(np.linspace(0, .95, len(pprs)))))
    L = [f"PPR settings found: {pprs}", ""]

    # edge loss and counts per revolution
    R["edges_per_rev"] = R.edge_total / R.revs
    e = R.groupby("ppr").agg(runs=("rep", "count"), edges_per_rev=("edges_per_rev", "mean"), quad_err_max=("quad_err", "max"), idx_rej=("idx_rej", "max"), idx_miss=("idx_miss", "max"),
                             period_p99=("period_p99", "mean"), late_max=("late_max", "max")).reset_index()
    e["expected"] = 4.0 * e.ppr
    e.to_csv(os.path.join(out, "table_edge_loss.csv"), index=False); tex_table(e, os.path.join(out, "table_edge_loss.tex"), "{:.1f}")
    L += ["EDGES PER REVOLUTION (must equal 4 x PPR) AND LOST EDGES (quad_err must be 0)", e.round(1).to_string(index=False), ""]

    # open loop
    A = S[(S.test == "A") & (S.n > 100)]
    if len(A):
        g = A.groupby(["ppr", "pwm_pct"]).agg(n=("n", "mean"), sd=("sd", "mean"), lo=("sd", "min"), hi=("sd", "max"), A1=("A1", "mean"), noise=("noise", "mean"), new_frac=("new_frac", "mean")).reset_index()
        g.to_csv(os.path.join(out, "open_loop_by_ppr.csv"), index=False)
        piv = g.pivot(index="pwm_pct", columns="ppr", values="sd"); piv.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
        piv.reset_index().to_csv(os.path.join(out, "table_sd_by_ppr.csv"), index=False); tex_table(piv.reset_index(), os.path.join(out, "table_sd_by_ppr.tex"))
        L += ["OPEN LOOP: SD of the raw speed (rpm) by PPR, mean over repeats", piv.round(2).reset_index().to_string(index=False), ""]
        a1t = g.pivot(index="pwm_pct", columns="ppr", values="A1"); a1t.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
        L += ["LINE AT THE ROTATION FREQUENCY, A1 (rpm), by PPR: it should NOT depend on the PPR if it belongs to the shaft", a1t.round(2).reset_index().to_string(index=False), ""]
        if A.bias_pct.notna().any():
            bt = A.groupby(["ppr", "pwm_pct"]).bias_pct.mean().reset_index().pivot(index="pwm_pct", columns="ppr", values="bias_pct"); bt.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
            bt.reset_index().to_csv(os.path.join(out, "table_bias_by_ppr.csv"), index=False)
            L += ["ACCURACY BY PPR: bias (%) of the speed estimate against the speed from the index pulses (independent of the PPR)", bt.round(3).reset_index().to_string(index=False), ""]
        nz = g.pivot(index="pwm_pct", columns="ppr", values="noise"); nz.insert(0, "speed_rpm", g.groupby("pwm_pct").n.mean())
        L += ["BROADBAND NOISE (rpm), the part that is not the line at the rotation frequency", nz.round(2).reset_index().to_string(index=False), ""]
        fig, ax = plt.subplots(1, 2, figsize=(6.8, 3.0))
        for p in pprs:
            q = g[g.ppr == p]; ax[0].plot(q.n, q.sd, "o-", ms=3, lw=1, color=col[p], label=str(p)); ax[0].fill_between(q.n, q.lo, q.hi, color=col[p], alpha=.12)
        ax[0].set_yscale("log"); ax[0].set_xlabel("Speed (rpm)"); ax[0].set_ylabel("SD of the raw speed (rpm)"); ax[0].legend(title="PPR", fontsize=6.5, ncol=2)
        lo = g[g.pwm_pct == g.pwm_pct[g.pwm_pct > 0].min()]
        ax[1].plot(lo.ppr, lo.noise, "o-", ms=3, label="broadband noise"); ax[1].plot(lo.ppr, lo.A1, "s--", ms=3, label="line at 1x")
        ax[1].set_xscale("log"); ax[1].set_xlabel("PPR"); ax[1].set_ylabel("at the lowest step (rpm)"); ax[1].legend(fontsize=7)
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_ppr_sd_noise.pdf")); plt.close(fig)

    # closed loop and the update floor
    B = S[S.test == "B"]
    if len(B):
        g = B.groupby(["ppr", "target"]).agg(n=("n", "mean"), sd=("sd", "mean"), err_rms=("err_rms", "mean"), new_frac=("new_frac", "mean"), zero_frac=("zero_frac", "mean")).reset_index()
        g.to_csv(os.path.join(out, "closed_loop_by_ppr.csv"), index=False)
        piv = g.pivot(index="target", columns="ppr", values="sd").reset_index().rename(columns={"target": "setpoint rpm"}); tex_table(piv, os.path.join(out, "table_closedloop_sd.tex"))
        L += ["CLOSED LOOP: SD of the raw speed (rpm) by PPR and setpoint", piv.round(2).to_string(index=False), ""]
        nf = g.pivot(index="target", columns="ppr", values="new_frac").reset_index().rename(columns={"target": "setpoint rpm"})
        L += ["FRACTION OF LOOPS WITH A NEW ESTIMATE (prediction: min(1, speed*PPR*0.010/60))", nf.round(2).to_string(index=False), ""]
        zf = g.pivot(index="target", columns="ppr", values="zero_frac").reset_index().rename(columns={"target": "setpoint rpm"})
        L += ["FRACTION OF SAMPLES REPORTED AS ZERO SPEED (the stopped-shaft timeout; matters at the lowest setpoints)", zf.round(2).to_string(index=False), ""]
        fig, ax = plt.subplots(figsize=(4.8, 3.2))
        for p in pprs:
            q = g[g.ppr == p]
            if len(q):
                ax.plot(q.n.clip(lower=1), q.new_frac, "o", ms=3, color=col[p], label=str(p)); v = np.linspace(2, 1600, 300); ax.plot(v, np.minimum(1, v * p * 0.010 / 60), "-", lw=.8, color=col[p])
        ax.set_xscale("log"); ax.set_xlabel("Speed (rpm)"); ax.set_ylabel("Fraction of loops with a new estimate"); ax.legend(title="PPR", fontsize=6.5, ncol=2)
        fig.tight_layout(); fig.savefig(os.path.join(out, "fig_update_floor.pdf")); plt.close(fig)

    if cpu:
        C = pd.DataFrame(cpu).groupby("ppr").mean(numeric_only=True).reset_index().drop(columns=["rep"], errors="ignore")
        C["edge_rate_kHz_at_1500rpm"] = C.ppr * 4 * 1500 / 60 / 1e3
        C.to_csv(os.path.join(out, "table_cpu.csv"), index=False)
        L += ["CPU LOAD (%CPU from pidstat, best effort: mean over all runs of that PPR) AND EDGE RATE", C.round(1).to_string(index=False), ""]
    open(os.path.join(out, "RESULTS.txt"), "w").write("\n".join(L)); print("\n".join(L)); print(f"\nEverything is in {out}\nSend me {os.path.join(out, 'RESULTS.txt')} and the figures.")


if __name__ == "__main__":
    main()

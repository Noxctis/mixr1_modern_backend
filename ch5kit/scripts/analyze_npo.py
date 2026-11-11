#!/usr/bin/env python3
"""
analyze_npo.py -- Reynolds and Power numbers from the runs of run_npo.sh, and what the speed estimator does to them.

    python3 ch5kit/scripts/analyze_npo.py --dir logs/ch5_new/npo [--tq-acc-nm 0.004]

For every setpoint (interior of each hold):  n from each estimator, n from the index pulses (the reference), torque, shaft power
    P = 2*pi*n_s*tau          N_Re = rho*n_s*D^2/mu          N_Po = P/(rho*n_s^3*D^5) = 2*pi*tau/(rho*n_s^2*D^5)      (n_s in rev/s)
Because the power is torque times speed, N_Po depends on the speed as n^-2 (not n^-3): a speed error of x % gives about 2x % in N_Po.
Writes tables (.csv/.tex), figures (.pdf) and RESULTS.txt into <dir>/results.  --tq-acc-nm is the absolute torque accuracy from the datasheet
(N m); without it the torque uncertainty contains only the measured scatter and the budget says so.
"""
import argparse, glob, json, os, re, sys
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_runs import load, load_index, idx_speed, tex_table, ORDER, LABEL, COLOR   # noqa: E402
plt.rcParams.update({"font.size": 8.5, "font.family": "serif", "axes.grid": True, "grid.alpha": .3})
NAME = re.compile(r"npo-(?P<est>[a-z0-9]+)_r(?P<rep>\d+)\.csv$")


def water(T):
    """Density (kg/m3, Tanaka 1975) and dynamic viscosity (Pa s, Vogel) of water at T deg C; valid 0 to 40 deg C."""
    rho = 999.974950 * (1 - (T - 3.983035) ** 2 * (T + 301.797) / (522528.9 * (T + 69.34881)))
    mu = 2.414e-5 * 10 ** (247.8 / ((T + 273.15) - 140.0))
    return rho, mu


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", required=True); ap.add_argument("--tq-acc-nm", type=float, default=None)
    ap.add_argument("--rho", type=float, default=None); ap.add_argument("--mu", type=float, default=None); a = ap.parse_args()
    meta = json.load(open(os.path.join(a.dir, "meta.json"))); D = float(meta["D_m"]); hold = float(meta["hold_s"])
    T = np.nanmean([meta.get("T_start", np.nan), meta.get("T_end", np.nan)])
    if not np.isfinite(T): sys.exit("meta.json has no fluid temperature: edit T_start in " + os.path.join(a.dir, "meta.json"))
    rho, mu = water(T); rho = a.rho or rho; mu = a.mu or mu
    out = os.path.join(a.dir, "results"); os.makedirs(out, exist_ok=True)
    rows = []
    for f in sorted(glob.glob(os.path.join(a.dir, "npo-*.csv"))):
        m = NAME.search(os.path.basename(f))
        if not m: continue
        d = load(f); idx = load_index(f)
        if "torque_nm" not in d or d.torque_nm.notna().sum() < 100:
            print("no torque in", os.path.basename(f), "(torque flags wrong or sensor off)"); continue
        for i, g in d.groupby("step_index"):
            t = g.elapsed_s.to_numpy()
            if t[-1] - t[0] < 10: continue
            s = g[(t >= t[0] + 0.4 * (t[-1] - t[0])) & (t <= t[0] + 0.97 * (t[-1] - t[0]))]
            n_est = float(np.abs(s.raw_rpm.to_numpy(float)).mean()); n_sd = float(np.abs(s.raw_rpm.to_numpy(float)).std(ddof=1))
            n_ref = idx_speed(d, idx, s.elapsed_s.iloc[0], s.elapsed_s.iloc[-1])
            tq = s.torque_nm.dropna().to_numpy(float)
            if len(tq) < 50 or n_est < 20: continue
            rows.append(dict(est=m["est"], rep=int(m["rep"]), step=int(i), target=float(s.target_rpm.mean()), n_est=n_est, n_sd=n_sd, n_ref=n_ref, N=len(s),
                             tau=float(tq.mean()), tau_sd=float(tq.std(ddof=1)), p_el=float(s.power_w.mean()) if "power_w" in s else np.nan))
    if not rows: sys.exit("No usable runs (with torque) found in " + a.dir)
    R = pd.DataFrame(rows)

    def npo(tau, n): ns = n / 60.0; return 2 * np.pi * tau / (rho * ns ** 2 * D ** 5)
    R["N_Re_est"] = rho * (R.n_est / 60) * D ** 2 / mu
    R["N_Re_ref"] = rho * (R.n_ref / 60) * D ** 2 / mu
    R["P_W"] = R.tau * 2 * np.pi * R.n_est / 60
    R["N_Po_est"] = npo(R.tau, R.n_est); R["N_Po_ref"] = npo(R.tau, R.n_ref)
    R["dNPo_pct"] = 100 * (R.N_Po_est - R.N_Po_ref) / R.N_Po_ref
    R["bias_pct"] = 100 * (R.n_est - R.n_ref) / R.n_ref
    # uncertainty budget (relative): speed (bias + scatter of the mean), torque (accuracy + scatter of the mean)
    R["u_n"] = np.sqrt((R.bias_pct / 100) ** 2 + (R.n_sd / R.n_est) ** 2 / R.N)
    tq_acc = a.tq_acc_nm if a.tq_acc_nm is not None else 0.0
    R["u_tau"] = np.sqrt((tq_acc / R.tau) ** 2 + (R.tau_sd / R.tau) ** 2 / R.N)
    R["u_NPo"] = np.sqrt(R.u_tau ** 2 + (2 * R.u_n) ** 2)
    R["share_speed"] = (2 * R.u_n) ** 2 / R.u_NPo ** 2
    R.to_csv(os.path.join(out, "npo_all_steps.csv"), index=False)
    g = R.groupby(["est", "target"]).agg(n_est=("n_est", "mean"), n_ref=("n_ref", "mean"), bias_pct=("bias_pct", "mean"), tau=("tau", "mean"), N_Re=("N_Re_ref", "mean"),
                                        N_Po_est=("N_Po_est", "mean"), N_Po_ref=("N_Po_ref", "mean"), dNPo_pct=("dNPo_pct", "mean"), N_Po_rep_sd=("N_Po_est", "std"),
                                        u_NPo_pct=("u_NPo", lambda x: 100 * x.mean()), share_speed=("share_speed", "mean")).reset_index()
    g.to_csv(os.path.join(out, "npo_by_estimator.csv"), index=False)
    ests = [e for e in ORDER if e in set(g.est)]
    L = [f"D = {D*1000:.0f} mm; fluid temperature {T:.1f} deg C -> rho = {rho:.1f} kg/m3, mu = {mu*1000:.4f} mPa s" + ("" if a.tq_acc_nm is not None else "   [torque accuracy NOT given: --tq-acc-nm]"), ""]
    for e in ests:
        q = g[g.est == e][["target", "n_est", "bias_pct", "tau", "N_Re", "N_Po_est", "dNPo_pct", "N_Po_rep_sd", "u_NPo_pct", "share_speed"]]
        L += [f"{LABEL[e]}", q.round(3).to_string(index=False), ""]
        tex_table(q.round(3), os.path.join(out, f"table_npo_{e}.tex"), "{:.3f}")
    L += ["EFFECT OF THE SPEED ESTIMATOR ON THE POWER NUMBER (N_Po from the estimator minus N_Po from the index speed), mean over setpoints:"]
    for e in ests:
        q = g[g.est == e]; L.append(f"  {LABEL[e]:12s} mean {q.dNPo_pct.mean():+.2f} %, largest |.| {q.dNPo_pct.abs().max():.2f} %   (speed bias mean {q.bias_pct.mean():+.2f} %)")
    L += ["", "DOMINANT UNCERTAINTY IN N_Po (share of the variance that comes from the speed), mean over setpoints:"]
    for e in ests: L.append(f"  {LABEL[e]:12s} speed {100*g[g.est == e].share_speed.mean():.0f} %   torque {100*(1-g[g.est == e].share_speed.mean()):.0f} %")
    fig, ax = plt.subplots(1, 2, figsize=(6.8, 3.0))
    for e in ests:
        q = g[g.est == e]; ax[0].semilogx(q.N_Re, q.N_Po_est, "o-", ms=3, lw=1, color=COLOR[e], label=LABEL[e]); ax[1].semilogx(q.N_Re, q.dNPo_pct, "o-", ms=3, lw=1, color=COLOR[e])
    ax[0].set_xlabel("$N_{Re}$"); ax[0].set_ylabel("$N_{Po}$"); ax[0].legend(fontsize=7); ax[1].axhline(0, color="k", lw=.6)
    ax[1].set_xlabel("$N_{Re}$"); ax[1].set_ylabel("$N_{Po}$ error caused by the speed estimate (%)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_npo.pdf")); plt.close(fig)
    open(os.path.join(out, "RESULTS.txt"), "w").write("\n".join(L)); print("\n".join(L)); print(f"\nSend me {os.path.join(out, 'RESULTS.txt')} and fig_npo.pdf")


if __name__ == "__main__":
    main()

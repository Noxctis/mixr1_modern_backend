#!/usr/bin/env python3
"""
confirm_analysis.py - are the differences between PI settings REAL, and which setting is best?

Reads one or more run_matrix.sh output folders (their results_raw.txt: one line per run, "name key=value ...") and, for every setting:
  * mean, standard deviation and a 95 % confidence interval over the repeats (Student t),
  * a Welch t-test against the baseline with the Holm correction for multiple comparisons,
  * the MINIMUM DETECTABLE DIFFERENCE of the experiment (differences smaller than this cannot be told apart from noise),
  * glitch counting (a run whose speed leaves the +-5 % band late in the run is a glitch, reported separately and excluded from time metrics),
  * validity of the reference-step runs (a step that hit the duty cap measures the cap, not the gains -> excluded),
then ranks the settings with explicit, printed rules.

  python3 tools/confirm_analysis.py matrix_confirm                       # startup runs
  python3 tools/confirm_analysis.py matrix_confirm matrix_step           # startup + reference-step runs (merged by setting name)
  python3 tools/confirm_analysis.py matrix_confirm --baseline A_baseline --out confirm_out
With only ONE run per setting there are no confidence intervals: the baseline's own repeat-to-repeat spread is used as the error bar.
"""
import argparse, collections, math, os, sys
import numpy as np

try:
    from scipy import stats
except ImportError:                                                # scipy is normally installed with matplotlib
    stats = None
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

METRICS = [  # key, label, unit, lower_is_better, kind
    ("pwm_jitter_pct", "PWM jitter", "% duty", True, "steady"),
    ("rpm_std", "Speed ripple (rpm_std)", "RPM", True, "steady"),
    ("overshoot_pct", "Startup overshoot", "%", True, "startup"),
    ("rs_overshoot_pct", "Step overshoot", "% of step", True, "step"),
    ("rs_rise_s", "Step rise time 10-90 %", "s", True, "step"),
    ("rs_settle_s", "Step settling time", "s", True, "step"),
    ("rs_rpm_std", "Speed ripple after the step", "RPM", True, "step"),
    ("p_mean_W", "Electrical power", "W", True, "info"),
]


def load(folders):
    runs = collections.defaultdict(list)
    for folder in folders:
        path = os.path.join(folder, "results_raw.txt") if os.path.isdir(folder) else folder
        for ln in open(path):
            p = ln.split()
            if len(p) < 2 or p[0].startswith("#"):
                continue
            kv = {}
            for x in p[1:]:
                k, _, v = x.partition("=")
                try:
                    kv[k] = float(v)
                except ValueError:
                    pass
            kv["_src"] = folder
            runs[p[0]].append(kv)
    return runs


def t95(df):
    return stats.t.ppf(0.975, df) if stats else 1.96 + 2.4 / max(df, 1)


def usable(key, runs):
    """Values of one metric over the repeats, with the rules that keep bad runs from polluting the statistics."""
    out = []
    for r in runs:
        if key not in r:
            continue
        v = r[key]
        if key.startswith("rs_") and r.get("rs_sat_s", 0.0) > 0.05:        # the step hit the duty cap: it measured the cap
            continue
        if key in ("rs_settle_s", "rs_rise_s") and v < 0:                    # never settled / never reached
            continue
        out.append(v)
    return np.array(out, float)


def glitches(runs):
    g = 0
    for r in runs:
        if r.get("settle5_s", 0.0) > 10.0 or r.get("rs_settle_s", 0.0) < 0.0:
            g += 1
    return g


def holm(pvals):
    idx = sorted(range(len(pvals)), key=lambda i: pvals[i])
    m = len(pvals)
    adj = [0.0] * m
    run = 0.0
    for rank, i in enumerate(idx):
        run = max(run, min(1.0, (m - rank) * pvals[i]))
        adj[i] = run
    return adj


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--baseline", default="A_baseline")
    ap.add_argument("--out", default=None, help="output folder (default: <first folder>/confirm)")
    ap.add_argument("--max-step-overshoot-rise", type=float, default=5.0, help="a setting is rejected if its step overshoot exceeds the baseline's by more than this many points")
    ap.add_argument("--max-settle-factor", type=float, default=1.5, help="a setting is rejected if its step settling time exceeds the baseline's by more than this factor")
    ap.add_argument("--max-ripple-rise-pct", type=float, default=15.0, help="a setting is rejected if its speed ripple exceeds the baseline's by more than this percentage")
    ap.add_argument("--abs-step-overshoot", type=float, default=None, help="ABSOLUTE limit on the step overshoot (%% of the step); replaces the 'relative to baseline' overshoot rule")
    ap.add_argument("--abs-step-settle", type=float, default=None, help="ABSOLUTE limit on the step settling time (s, +-5 %% of the step); replaces the 'relative to baseline' settling rule. A setting that never settles fails it")
    args = ap.parse_args()
    out = args.out or os.path.join(args.folders[0] if os.path.isdir(args.folders[0]) else os.path.dirname(args.folders[0]) or ".", "confirm")
    os.makedirs(out, exist_ok=True)

    runs = load(args.folders)
    if args.baseline not in runs:
        sys.exit(f"baseline '{args.baseline}' not found in the results (found: {', '.join(list(runs)[:6])} ...)")
    names = list(runs)
    have = [m for m in METRICS if any(m[0] in r for rr in runs.values() for r in rr)]
    nmax = max(len(v) for v in runs.values())
    base_runs = runs[args.baseline]
    L = []
    P = L.append
    P(f"CONFIRMATION ANALYSIS   folders: {', '.join(args.folders)}")
    P(f"{len(names)} settings, {sum(len(v) for v in runs.values())} runs; repeats per setting: " + ", ".join(f"{n}={len(runs[n])}" for n in names[:12]) + (" ..." if len(names) > 12 else ""))
    P(f"baseline = {args.baseline} ({len(base_runs)} repeats)")
    if len(base_runs) < 2:
        P("WARNING: the baseline has fewer than 2 repeats: no noise estimate is possible.")
    P("")

    # ---------------------------------------------------------------- noise floor from the baseline
    P("=== NOISE FLOOR: the baseline repeated with identical settings ===")
    base_sd = {}
    for key, label, unit, low, kind in have:
        v = usable(key, base_runs)
        if len(v) >= 2:
            base_sd[key] = float(np.std(v, ddof=1))
            P(f"  {label:34s} mean {v.mean():9.4f} {unit:9s} sd {base_sd[key]:8.4f}  (relative {base_sd[key] / abs(v.mean()) * 100:5.1f} %)   n={len(v)}")
    P("")

    # ---------------------------------------------------------------- per-setting statistics
    rows = []
    for name in names:
        rr = runs[name]
        row = dict(setting=name, n=len(rr), kp=rr[0].get("kp", np.nan), ki=rr[0].get("ki", np.nan), alpha=rr[0].get("alpha", np.nan), glitch_runs=glitches(rr))
        for key, label, unit, low, kind in have:
            v = usable(key, rr)
            row[key + "_n"] = len(v)
            row[key] = float(v.mean()) if len(v) else np.nan
            if len(v) >= 2:
                sd = float(np.std(v, ddof=1))
                row[key + "_sd"] = sd
                row[key + "_ci"] = t95(len(v) - 1) * sd / math.sqrt(len(v))
            else:
                row[key + "_sd"] = np.nan
                row[key + "_ci"] = (t95(len(base_runs) - 1) * base_sd[key] / math.sqrt(max(len(v), 1))) if (key in base_sd and len(v) == 1) else np.nan
            row[key + "_proxy"] = bool(len(v) == 1 and key in base_sd)
        rows.append(row)
    byname = {r["setting"]: r for r in rows}
    base = byname[args.baseline]

    # ---------------------------------------------------------------- tests against the baseline
    for key, label, unit, low, kind in have:
        pv = []
        who = []
        for r in rows:
            if r["setting"] == args.baseline:
                continue
            a, b = usable(key, runs[r["setting"]]), usable(key, base_runs)
            if len(a) >= 2 and len(b) >= 2 and stats:
                pv.append(float(stats.ttest_ind(a, b, equal_var=False)[1]))
                who.append(r["setting"])
        adj = holm(pv) if pv else []
        for s_, p_, pa in zip(who, pv, adj):
            byname[s_][key + "_p"] = p_
            byname[s_][key + "_padj"] = pa

    # minimum detectable difference of THIS experiment (two-sided 95 %)
    def mdd(key, n_set):
        nb = len(usable(key, base_runs))
        if key not in base_sd or nb < 2:
            return float("nan")
        if n_set <= 1:                                   # one run against the baseline mean
            return t95(nb - 1) * base_sd[key] * math.sqrt(1.0 + 1.0 / nb)
        return t95(n_set + nb - 2) * base_sd[key] * math.sqrt(1.0 / n_set + 1.0 / nb)
    P("=== WHAT THIS EXPERIMENT CAN DETECT (smallest difference to the baseline that is more than noise, 95 %) ===")
    for key, label, unit, low, kind in have:
        if key not in base_sd:
            continue
        ns = [r[key + "_n"] for r in rows if r["setting"] != args.baseline and r[key + "_n"] > 0]
        nn = int(np.median(ns)) if ns else 1
        P(f"  {label:34s} with your n={nn} per setting: {mdd(key, nn):8.4f} {unit:9s}   with n=5 per setting: {mdd(key, 5):8.4f} {unit}")
    P("")

    # ---------------------------------------------------------------- the table
    def fmt(r, key, d=3):
        v = r.get(key, np.nan)
        if not (v == v):
            return "    -    "
        ci = r.get(key + "_ci", np.nan)
        mark = "~" if r.get(key + "_proxy") else " "
        return f"{v:.{d}f}" + (f"{mark}+-{ci:.{d}f}" if ci == ci else "")

    P("=== RESULTS: mean +- 95 % CI over the repeats   ('~' = only 1 run: the +- is the BASELINE's noise, not this setting's) ===")
    hdr = f"{'setting':<24}{'n':>2} {'kp':>6} {'alpha':>6} | {'jitter %duty':>17} {'vs base':>8} {'p(adj)':>7} | {'rpm_std':>16} {'p(adj)':>7}"
    if any(m[0] == "rs_overshoot_pct" for m in have):
        hdr += f" | {'step os %':>16} {'settle s':>14} {'rise s':>13}"
    hdr += " | glitches"
    P(hdr)
    P("-" * len(hdr))
    for r in sorted(rows, key=lambda r: (r["setting"] != args.baseline, r.get("pwm_jitter_pct", 9))):
        jb = base.get("pwm_jitter_pct", np.nan)
        rel = f"{(r['pwm_jitter_pct'] / jb - 1) * 100:+6.0f}%" if (jb == jb and r["setting"] != args.baseline and r.get("pwm_jitter_pct", np.nan) == r.get("pwm_jitter_pct", np.nan)) else "   base" if r["setting"] == args.baseline else "      -"
        pj = r.get("pwm_jitter_pct_padj", np.nan)
        pr = r.get("rpm_std_padj", np.nan)
        line = (f"{r['setting']:<24}{r['n']:>2} {r['kp']:>6.2f} {r['alpha']:>6.2f} | {fmt(r, 'pwm_jitter_pct'):>17} {rel:>8} {('%.3f' % pj) if pj == pj else '   -  ':>7} | {fmt(r, 'rpm_std', 2):>16} {('%.3f' % pr) if pr == pr else '   -  ':>7}")
        if any(m[0] == "rs_overshoot_pct" for m in have):
            line += f" | {fmt(r, 'rs_overshoot_pct', 1):>16} {fmt(r, 'rs_settle_s', 2):>14} {fmt(r, 'rs_rise_s', 2):>13}"
        line += f" | {r['glitch_runs']}/{r['n']}"
        P(line)
    P("")

    # ---------------------------------------------------------------- ranking rules
    P("=== RANKING RULES (all printed so you can defend them) ===")
    P("  1. REJECT a setting with a glitch in more than 20 % of its runs, or whose reference step hit the duty cap.")
    P(f"  2. REJECT if its speed ripple (rpm_std) is more than {args.max_ripple_rise_pct:g} % above the baseline's.")
    if any(m[0] == "rs_overshoot_pct" for m in have):
        if args.abs_step_overshoot is not None or args.abs_step_settle is not None:
            lim = []
            if args.abs_step_overshoot is not None: lim.append(f"step overshoot above {args.abs_step_overshoot:g} % of the step")
            if args.abs_step_settle is not None: lim.append(f"step settling (+-5 %) longer than {args.abs_step_settle:g} s, or never settling")
            P("  3. REJECT (YOUR targets, not relative to the baseline): " + "; ".join(lim) + ".")
        else:
            P(f"  3. REJECT if its step overshoot is more than {args.max_step_overshoot_rise:g} points above the baseline's, or its step settling is more than {args.max_settle_factor:g}x the baseline's.")
    P("  4. Among the rest, prefer the LOWEST PWM jitter; settings whose jitter differs by less than the minimum detectable difference are TIED,")
    P("     and a tie is broken by the lower step overshoot (if measured), then the lower speed ripple.")
    P("")
    keep, rejected = [], []
    jb = base["pwm_jitter_pct"]
    mdd_j = np.nan
    if "pwm_jitter_pct" in base_sd:
        ns = [r["pwm_jitter_pct_n"] for r in rows if r["setting"] != args.baseline and r["pwm_jitter_pct_n"] > 0]
        mdd_j = mdd("pwm_jitter_pct", int(np.median(ns)) if ns else 1)
    for r in rows:
        if r["setting"] == args.baseline:
            continue
        why = []
        if r["n"] and r["glitch_runs"] / r["n"] > 0.20:
            why.append(f"glitches in {r['glitch_runs']}/{r['n']} runs")
        if "rs_overshoot_pct" in have_keys(have) and r.get("rs_overshoot_pct_n", 1) == 0 and r["n"] > 0 and any("rs_sat_s" in x for x in runs[r["setting"]]):
            why.append("every step run hit the duty cap (invalid)")
        if base.get("rpm_std") == base.get("rpm_std") and r.get("rpm_std", np.nan) == r.get("rpm_std", np.nan) and r["rpm_std"] > base["rpm_std"] * (1 + args.max_ripple_rise_pct / 100):
            why.append(f"speed ripple {r['rpm_std']:.2f} > baseline {base['rpm_std']:.2f} +{args.max_ripple_rise_pct:g} %")
        use_abs = args.abs_step_overshoot is not None or args.abs_step_settle is not None
        if use_abs:
            if args.abs_step_overshoot is not None and r.get("rs_overshoot_pct", np.nan) == r.get("rs_overshoot_pct", np.nan) and r["rs_overshoot_pct"] > args.abs_step_overshoot:
                why.append(f"step overshoot {r['rs_overshoot_pct']:.1f} % > your limit {args.abs_step_overshoot:g} %")
            if args.abs_step_settle is not None:
                st_ = r.get("rs_settle_s", np.nan)
                if st_ != st_:
                    why.append("never settled within the run (or the step hit the duty cap)")
                elif st_ > args.abs_step_settle:
                    why.append(f"step settling {st_:.2f} s > your limit {args.abs_step_settle:g} s")
        else:
            if r.get("rs_overshoot_pct", np.nan) == r.get("rs_overshoot_pct", np.nan) and base.get("rs_overshoot_pct", np.nan) == base.get("rs_overshoot_pct", np.nan):
                if r["rs_overshoot_pct"] > base["rs_overshoot_pct"] + args.max_step_overshoot_rise:
                    why.append(f"step overshoot {r['rs_overshoot_pct']:.1f} % vs baseline {base['rs_overshoot_pct']:.1f} %")
            if r.get("rs_settle_s", np.nan) == r.get("rs_settle_s", np.nan) and base.get("rs_settle_s", np.nan) == base.get("rs_settle_s", np.nan) and base["rs_settle_s"] > 0:
                if r["rs_settle_s"] > base["rs_settle_s"] * args.max_settle_factor:
                    why.append(f"step settling {r['rs_settle_s']:.2f} s vs baseline {base['rs_settle_s']:.2f} s")
        if r.get("pwm_jitter_pct", np.nan) == r.get("pwm_jitter_pct", np.nan) and jb == jb:
            need = mdd_j if mdd_j == mdd_j else 0.0
            if jb - r["pwm_jitter_pct"] <= need:
                why.append(f"no significant jitter reduction ({(r['pwm_jitter_pct'] / jb - 1) * 100:+.0f} %; the experiment can only detect > {need:.3f} % duty)")
        (rejected if why else keep).append((r, why))
    keep.sort(key=lambda x: x[0]["pwm_jitter_pct"])
    ranked = []
    if keep:
        best = keep[0][0]["pwm_jitter_pct"]
        tied = [x for x in keep if (x[0]["pwm_jitter_pct"] - best) < (mdd_j if mdd_j == mdd_j else 0.0)]
        rest = [x for x in keep if x not in tied]
        key2 = lambda x: (x[0].get("rs_overshoot_pct", 0.0) if x[0].get("rs_overshoot_pct", np.nan) == x[0].get("rs_overshoot_pct", np.nan) else 0.0, x[0].get("rpm_std", 9.0))
        ranked = sorted(tied, key=key2) + rest
    P("=== RECOMMENDATION ===")
    if not ranked:
        P("  No setting passes all rules. See the rejected list below, or relax the thresholds.")
    for i, (r, _) in enumerate(ranked[:6], 1):
        tie = ""
        if mdd_j == mdd_j and (r["pwm_jitter_pct"] - ranked[0][0]["pwm_jitter_pct"]) < mdd_j and i > 1:
            tie = "  (tied with #1 on jitter)"
        P(f"  #{i}  {r['setting']:<24} Kp {r['kp']:.3f}  alpha {r['alpha']:.2f}   jitter {r['pwm_jitter_pct']:.3f} ({(r['pwm_jitter_pct'] / jb - 1) * 100:+.0f} % vs baseline)   rpm_std {r['rpm_std']:.2f}" + tie)
    if ranked and mdd_j == mdd_j:
        P(f"  (jitter differences below {mdd_j:.3f} % duty are within the noise of this experiment: such settings are TIED on jitter)")
    P("")
    if rejected:
        P("=== REJECTED ===")
        for r, why in rejected:
            P(f"  {r['setting']:<24} " + "; ".join(why))
        P("")
    open(os.path.join(out, "confirm_report.txt"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))

    # ---------------------------------------------------------------- csv + figure
    import csv
    keys = sorted({k for r in rows for k in r})
    first = ["setting", "n", "kp", "ki", "alpha", "glitch_runs"]
    with open(os.path.join(out, "confirm_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=first + [k for k in keys if k not in first])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    panels = [m for m in have if m[0] in ("pwm_jitter_pct", "rpm_std", "rs_overshoot_pct", "rs_settle_s")]
    if panels:
        fig, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 5.2), squeeze=False)
        order = [args.baseline] + [r["setting"] for r in sorted(rows, key=lambda r: r.get("pwm_jitter_pct", 9)) if r["setting"] != args.baseline]
        for ax, (key, label, unit, low, kind) in zip(axes[0], panels):
            ys = [byname[s].get(key, np.nan) for s in order]
            es = [byname[s].get(key + "_ci", np.nan) for s in order]
            cols = ["#C0392B" if s == args.baseline else ("#1E8449" if s in [x[0]["setting"] for x in ranked[:3]] else "#7F8C8D") for s in order]
            ax.barh(range(len(order)), ys, xerr=[0 if e != e else e for e in es], color=cols, ecolor="k", capsize=3)
            ax.set_yticks(range(len(order)))
            short = [x.split("_")[0] for x in order]
            labels = [("baseline" if x == args.baseline else (sh if short.count(sh) == 1 else x[:18])) for x, sh in zip(order, short)]
            ax.set_yticklabels(labels, fontsize=9)
            ax.invert_yaxis()
            ax.set_xlabel(f"{label} ({unit})")
            ax.set_title(label + "\nmean +- 95 % CI")
            ax.grid(alpha=0.3, axis="x")
        fig.suptitle("Red = baseline, green = recommended (top 3), grey = others.  Bars with ~ are single runs (error bar = baseline noise).", fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "confirm_ci.png"), dpi=130)
    print(f"\nwritten: {out}/confirm_report.txt, confirm_summary.csv, confirm_ci.png")


def have_keys(have):
    return [h[0] for h in have]


if __name__ == "__main__":
    main()
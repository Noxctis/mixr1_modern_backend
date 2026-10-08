#!/usr/bin/env python3
"""
mixr_figures.py - automatic thesis-quality AND PowerPoint-ready figures for MIXR-1 test data.

Put the CSVs of ONE test session in a folder (plus results_raw.txt if you have it) and run:

    python mixr_figures.py <folder>                  # -> <folder>/figures  and  <folder>/figures/individual
    python mixr_figures.py <folder> --list           # only show how each file was understood
    python mixr_figures.py <folder> --show C3,F3     # force which configs get time-series graphs
    python mixr_figures.py <folder> --no-title       # individual graphs without titles (your slide has its own title)
    python mixr_figures.py <folder> --slide-size 13.33 7.5   # individual graph size in inches (default 10 x 5.625 = 16:9)

TWO kinds of output:
  figures/             multi-panel composites for the thesis document   (PNG 300 dpi + PDF vector)
  figures/individual/  every panel as its own graph for PowerPoint       (PNG 200 dpi + SVG), 16:9, large fonts

Recognised automatically from the CSV contents (file names do not matter for the type):
  PI step runs (daemon test mode) | open-loop runs (constant PWM) | dashboard exports | results_raw.txt (metrics only)
For PI runs the gains come from results_raw.txt when present, otherwise from the file name (C3_kp0.8_r1.csv, dots or underscores).
Compare numbers only WITHIN one session folder.  Also written: captions.md and metrics_*.csv.
Needs: numpy, pandas, matplotlib.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from dataclasses import dataclass

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FixedFormatter, NullFormatter

MINUS = "\u2212"
FS = 100.0
NYQ = FS / 2.0
DEF_KP, DEF_KI = 1.5641, 41.2249
PALETTE = ["#C0392B", "#2471A3", "#1E8449", "#8E44AD", "#E08A00", "#117A8B", "#7F8C8D", "#B7950B"]
SLIDE_RC = {"font.size": 14, "axes.labelsize": 15, "axes.titlesize": 15, "legend.fontsize": 12,
            "xtick.labelsize": 13, "ytick.labelsize": 13, "lines.linewidth": 2.0}


def set_style():
    plt.rcParams.update({
        "font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"],
        "font.size": 10.5, "axes.labelsize": 11, "axes.titlesize": 11, "legend.fontsize": 9,
        "mathtext.fontset": "stix", "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 100, "svg.fonttype": "path", "axes.axisbelow": True,
    })


# ============================================================================================ output manager
class Out:
    """Writes composite figures (thesis) and individual graphs (PowerPoint)."""

    def __init__(self, outdir, args):
        self.dir = outdir
        self.ind = os.path.join(outdir, "individual")
        self.args = args
        self.caps = []
        self.n_ind = 0
        os.makedirs(self.dir, exist_ok=True)
        if not args.no_individual:
            os.makedirs(self.ind, exist_ok=True)

    def comp(self, fig, name, caption):
        for ext in ("png", "pdf"):
            fig.savefig(os.path.join(self.dir, f"{name}.{ext}"), dpi=300, bbox_inches="tight")
        plt.close(fig)
        self.caps.append((name, caption))
        print(f"  wrote {name}.png / .pdf")

    def single(self, name, draw, title=None, caption=None, size=None):
        """draw(ax) fills one axes. Saved at slide size with large fonts."""
        if self.args.no_individual:
            return
        with plt.rc_context(SLIDE_RC):
            fig, ax = plt.subplots(figsize=size or tuple(self.args.slide_size))
            draw(ax)
            if title and not self.args.no_title:
                ax.set_title(title, fontsize=15, fontweight="bold", loc="left")
            fig.tight_layout()
            fig.savefig(os.path.join(self.ind, f"{name}.png"), dpi=200)
            fig.savefig(os.path.join(self.ind, f"{name}.svg"))
            plt.close(fig)
        self.n_ind += 1
        if caption:
            self.caps.append((f"individual/{name}", caption))

    def single2(self, name, draw, title=None, caption=None, size=None, ratios=(3, 2)):
        """Two stacked panels sharing the time axis (speed on top, PWM below). draw(ax_top, ax_bottom)."""
        if self.args.no_individual:
            return
        rc = dict(SLIDE_RC)
        rc.update({"font.size": 13, "xtick.labelsize": 12, "ytick.labelsize": 12, "axes.labelsize": 14, "legend.fontsize": 11})
        with plt.rc_context(rc):
            fig, (a1, a2) = plt.subplots(2, 1, figsize=size or tuple(self.args.slide_size), sharex=True,
                                         gridspec_kw={"height_ratios": list(ratios)})
            draw(a1, a2)
            if title and not self.args.no_title:
                a1.set_title(title, fontsize=14, fontweight="bold", loc="left")
            fig.tight_layout(h_pad=0.5)
            fig.savefig(os.path.join(self.ind, f"{name}.png"), dpi=200)
            fig.savefig(os.path.join(self.ind, f"{name}.svg"))
            plt.close(fig)
        self.n_ind += 1
        if caption:
            self.caps.append((f"individual/{name}", caption))

    def single_fig(self, name, fig, caption=None):
        """Save an already-built single-axes figure (e.g. a table) to the individual folder too."""
        if self.args.no_individual:
            return
        fig.savefig(os.path.join(self.ind, f"{name}.png"), dpi=200, bbox_inches="tight")
        fig.savefig(os.path.join(self.ind, f"{name}.svg"), bbox_inches="tight")
        self.n_ind += 1
        if caption:
            self.caps.append((f"individual/{name}", caption))


# ============================================================================================ data model
@dataclass
class Run:
    path: str
    stem: str
    kind: str
    cfg: str
    rep: int
    t: np.ndarray
    rpm: np.ndarray
    pwm: np.ndarray
    target: float = float("nan")
    tgt_series: np.ndarray | None = None
    fb: np.ndarray | None = None
    filt: np.ndarray | None = None
    torque_v: np.ndarray | None = None
    torque_nm: np.ndarray | None = None
    pwm_cmd: int = 0


def load_run(path: str):
    stem = os.path.splitext(os.path.basename(path))[0]
    try:
        df = pd.read_csv(path)
    except Exception as e:                                           # noqa: BLE001
        print(f"  skip {os.path.basename(path)}: cannot read ({e})")
        return None
    cols = set(df.columns)
    m = re.search(r"_r(\d+)$", stem)
    rep = int(m.group(1)) if m else 1
    cfg = re.sub(r"_r\d+$", "", stem)

    if {"elapsed_s", "raw_rpm", "pwm"} <= cols:
        t = df.elapsed_s.to_numpy(float)
        rpm = df.raw_rpm.to_numpy(float)
        pwm_raw = df.pwm.to_numpy(float)
        mode = str(df.intended_mode.iloc[0]) if "intended_mode" in cols else ""
        tgt = df.target_rpm.to_numpy(float) if "target_rpm" in cols else np.full(len(t), np.nan)
        step = df.step_index.to_numpy() if "step_index" in cols else -np.ones(len(t))
        const_pwm = np.ptp(pwm_raw) == 0
        const_tgt = np.isfinite(tgt).all() and np.ptp(tgt) < 1e-9
        if mode.lower().startswith("open") or (not mode and const_pwm):
            if not const_pwm:
                print(f"  skip {stem}: open-loop but PWM changes (sweep) - not handled")
                return None
            kind = "OL"
        elif mode.upper().startswith("PI") and const_tgt and (step == -1).all():
            kind = "PI"
        else:
            print(f"  skip {stem}: sweep / sine / non-constant target")
            return None
        r = Run(path, stem, kind, cfg, rep, t, rpm, pwm_raw * 100.0 / 4095.0, target=float(tgt[0]) if const_tgt else np.nan,
                fb=df.fb_rpm.to_numpy(float) if "fb_rpm" in cols else None, pwm_cmd=int(pwm_raw[0]))
        if "torque_nm" in cols and df.torque_nm.notna().any():
            r.torque_nm = df.torque_nm.to_numpy(float)
            r.torque_v = df.torque_v.to_numpy(float) if "torque_v" in cols else None
        return r

    if {"t (s)", "Raw RPM"} <= cols:
        t = df["t (s)"].to_numpy(float)
        r = Run(path, stem, "DASH", cfg, rep, t, df["Raw RPM"].to_numpy(float),
                df["PWM (%)"].to_numpy(float) if "PWM (%)" in cols else np.full(len(t), np.nan))
        if "Filtered RPM" in cols:
            r.filt = df["Filtered RPM"].to_numpy(float)
        if "Target RPM" in cols:
            r.tgt_series = df["Target RPM"].to_numpy(float)
            pos = r.tgt_series[r.tgt_series > 0]
            r.target = float(np.median(pos)) if len(pos) else float("nan")
        if "Torque" in cols and np.nanmax(np.abs(df["Torque"].to_numpy(float))) > 0:
            r.torque_nm = df["Torque"].to_numpy(float)
        return r

    print(f"  skip {stem}: unknown CSV layout")
    return None


# ============================================================================================ signal helpers
def steady_arr(r: Run, arr, t0=5.0, n=1500):
    start = t0 if r.t[-1] >= 12.0 else 0.4 * r.t[-1]
    x = np.asarray(arr)[r.t >= start][:n]
    return x if len(x) >= 300 else None


def spectrum(x, fs=FS):
    x = x - np.nanmean(x)
    w = np.hanning(len(x))
    fr = np.fft.rfftfreq(len(x), 1.0 / fs)
    return fr, np.abs(np.fft.rfft(x * w)) * 2.0 / w.sum()


def order_peak(fr, a, f0):
    if f0 > NYQ - 1.0:
        return float("nan")
    sel = np.abs(fr - f0) < 0.12
    return float(a[sel].max()) if sel.any() else float("nan")


def order_spectrum(r: Run, arr, xmax=4.5):
    x_rpm = steady_arr(r, r.rpm)
    x = steady_arr(r, arr)
    if x is None or x_rpm is None:
        return None
    sh = float(np.mean(x_rpm)) / 60.0
    fr, a = spectrum(x)
    o = fr / sh
    return o[o <= xmax], a[o <= xmax], sh


def shaft_hz(r: Run):
    x = steady_arr(r, r.rpm)
    return float(np.mean(x)) / 60.0 if x is not None else float("nan")


def pwm_jitter(p):
    return float(np.sqrt(np.mean(np.diff(p) ** 2)))


def pi_metrics(r: Run):
    ss = r.t >= 0.6 * r.t[-1]
    tgt = r.target
    out = np.where(np.abs(r.rpm - tgt) > 0.05 * tgt)[0]
    if len(out) == 0:
        settle = 0.0
    elif out[-1] == len(r.rpm) - 1:
        settle = -1.0
    else:
        settle = float(r.t[out[-1]])
    return dict(jitter=pwm_jitter(r.pwm[ss]), pwm_std=float(np.std(r.pwm[ss])), rpm_std=float(np.std(r.rpm[ss])),
                sse=float(tgt - np.mean(r.rpm[ss])), overshoot=float(max(0.0, (r.rpm.max() - tgt) / tgt * 100.0)),
                settle=settle, pwm_mean=float(np.mean(r.pwm[ss])), rpm_mean=float(np.mean(r.rpm[ss])))


def loglog_fit(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y) & (x > 0) & (y > 0)
    x, y = np.log(x[ok]), np.log(y[ok])
    n = len(x)
    if n < 3:
        return None
    A = np.vstack([x, np.ones(n)]).T
    coef = np.linalg.lstsq(A, y, rcond=None)[0]
    res = y - A @ coef
    s2 = res @ res / (n - 2)
    se = float(np.sqrt(s2 / np.sum((x - x.mean()) ** 2)))
    r2 = float(1 - (res @ res) / np.sum((y - y.mean()) ** 2)) if np.ptp(y) > 0 else 1.0
    return float(coef[0]), se, r2, n


def log_axis(ax, which, lo, hi, cand):
    ticks = [c for c in cand if lo * 0.85 <= c <= hi * 1.2]
    ticks = ticks if len(ticks) >= 2 else cand[:3]
    axis = ax.xaxis if which == "x" else ax.yaxis
    axis.set_major_locator(FixedLocator(ticks))
    axis.set_major_formatter(FixedFormatter([f"{c:g}" for c in ticks]))
    axis.set_minor_formatter(NullFormatter())


def trace_window(r: Run, win):
    lo, hi = win
    if r.t[-1] < hi + 0.5:
        hi = r.t[-1]
        lo = max(0.0, hi - (win[1] - win[0]))
    return (r.t >= lo) & (r.t <= hi), (lo, hi)


def fg(x):
    """Gain/alpha label: 1.0, 0.8, 1.56, 0.15 (at most 2 decimals, at least 1)."""
    t = f"{x:.2f}".rstrip("0")
    return t + "0" if t.endswith(".") else t


def fk(x):
    return f"{x:.1f}"


def mn(s):
    return s.replace("-", MINUS)


# ============================================================================================ PI study: model
def parse_gains(name: str):
    n = name.lower()
    num = lambda a, b: float(f"{a}.{b}") if b is not None else float(a)          # noqa: E731
    mk = re.search(r"kp(\d+)(?:[._](\d+))?", n)
    mi = re.search(r"ki(\d+)(?:[._](\d+))?", n)
    ma = re.search(r"(?:alpha|(?<![a-z])a)(\d+)(?:[._](\d+))?", n)
    kp = num(*mk.groups()) if mk else DEF_KP
    ki = num(*mi.groups()) if mi else DEF_KI
    al = num(*ma.groups()) if ma else 1.0
    return kp, ki, al, bool(mk or mi or ma or "baseline" in n)


def read_raw(path: str):
    rows = []
    for ln in open(path):
        p = ln.split()
        if len(p) < 3 or "=" not in p[1]:
            continue
        try:
            kv = {k: float(v) for k, v in (x.split("=") for x in p[1:])}
        except ValueError:
            continue
        kv["name"] = p[0]
        rows.append(kv)
    return rows


def norm(name):
    return name.replace(".", "_")


@dataclass
class Cfg:
    name: str
    id: str
    kp: float
    ki: float
    alpha: float
    known: bool
    runs: list
    rows: list
    source: str


def short_id(name):
    return name.split("_")[0] if re.match(r"^[A-Za-z]\d*_", name) else name


def build_cfgs(pi_runs, raw_rows):
    cfgs = {}
    for r in sorted(pi_runs, key=lambda x: (x.cfg, x.rep)):
        c = cfgs.get(r.cfg)
        if c is None:
            kp, ki, al, known = parse_gains(r.cfg)
            c = cfgs[r.cfg] = Cfg(r.cfg, short_id(r.cfg), kp, ki, al, known, [], [], "csv")
        c.runs.append(r)
        c.rows.append(pi_metrics(r))
    by_norm = {norm(c.name): c for c in cfgs.values()}
    for rw in raw_rows:
        key = norm(rw["name"])
        m = dict(jitter=rw["pwm_jitter_pct"], pwm_std=rw["pwm_std_pct"], rpm_std=rw["rpm_std"], sse=rw["sse"],
                 overshoot=rw["overshoot_pct"], settle=rw["settle5_s"], pwm_mean=rw["pwm_mean_pct"], rpm_mean=rw["rpm_mean"])
        if key in by_norm:
            c = by_norm[key]
            c.kp, c.ki, c.alpha, c.known = rw["kp"], rw["ki"], rw["alpha"], True
            continue
        c = cfgs.get(rw["name"])
        if c is None:
            c = cfgs[rw["name"]] = Cfg(rw["name"], short_id(rw["name"]), rw["kp"], rw["ki"], rw["alpha"], True, [], [], "raw")
        c.rows.append(m)
    return cfgs


def mean_m(c: Cfg, k):
    v = [r[k] for r in c.rows]
    return float(np.mean(v)) if v else float("nan")


def find_baseline(cfgs):
    for c in cfgs.values():
        if abs(c.kp - DEF_KP) < 0.01 and abs(c.ki - DEF_KI) < 0.5 and abs(c.alpha - 1.0) < 1e-6 and c.known:
            return c
    for c in cfgs.values():
        if "baseline" in c.name.lower():
            return c
    return None


def label_of(c: Cfg, base, long=True):
    if base is not None and c is base:
        return f"{c.id}: baseline (Kp {fg(c.kp)}, Ki {fk(c.ki)})" if long else c.id
    if not c.known:
        return c.id
    parts = []
    if base is None or abs(c.kp - base.kp) > 1e-3:
        parts.append(f"Kp {fg(c.kp)}")
    if base is None or abs(c.ki - base.ki) > 0.05:
        parts.append(f"Ki {fk(c.ki)}")
    if abs(c.alpha - 1.0) > 1e-6:
        parts.append(f"\u03b1 {fg(c.alpha)}")
    return f"{c.id}: " + ", ".join(parts) if long else c.id


def pareto_front(items):
    return [(c, j, o) for c, j, o in items
            if not any((j2 <= j and o2 <= o) and (j2 < j or o2 < o) for _, j2, o2 in items)]


NF_COLORS = ["#2471A3", "#5DADE2", "#154360", "#85C1E9", "#1F618D"]            # no-filter family: blues
FL_COLORS = ["#E08A00", "#8E44AD", "#1E8449", "#B7950B", "#117A8B", "#D81B60", "#6D4C41", "#7F8C8D", "#00897B", "#3949AB"]  # filtered family


def fam(c, base):
    """0 = baseline, 1 = no feedback filter (Kp / Ki changes), 2 = with feedback filter."""
    if base is not None and c is base:
        return 0
    return 1 if abs(c.alpha - 1) < 1e-6 else 2


def fam_key(c, base):
    return (fam(c, base), -c.kp, -c.ki, c.id)


def assign_colors(cfgs, base):
    """Stable colour per setting (same colour in every figure, whichever subset is shown). Baseline red, no-filter blues, filtered warm/green."""
    cols, i1, i2 = {}, 0, 0
    for c in sorted(cfgs.values(), key=lambda c: fam_key(c, base)):
        f = fam(c, base)
        if f == 0:
            cols[c.name] = PALETTE[0]
        elif f == 1:
            cols[c.name] = NF_COLORS[i1 % len(NF_COLORS)]
            i1 += 1
        else:
            cols[c.name] = FL_COLORS[i2 % len(FL_COLORS)]
            i2 += 1
    return cols


def family_sets(shown, base):
    """Overlay groups so no single graph has more than ~4 lines."""
    if base is None or base not in shown or len(shown) < 5:
        return []
    nf = [c for c in shown if fam(c, base) == 1]
    fl = [c for c in shown if fam(c, base) == 2]
    same = [c for c in fl if abs(c.kp - base.kp) < 1e-3 and abs(c.ki - base.ki) < 0.05]      # filter only, baseline gains
    both = [c for c in fl if c not in same]                                                  # lower gains AND filter
    sets = []
    if len(nf) >= 2:
        sets.append(("no_filter", "Baseline vs lower Kp / Ki (no filter)", [base] + nf))
    if len(same) >= 2:
        sets.append(("filter_only", "Baseline vs feedback filter alone (baseline Kp, Ki)", [base] + same))
    if len(both) >= 2:
        sets.append(("gains_and_filter", "Baseline vs lower gains plus filter", [base] + both))
    return sets


def choose_shown(cfgs, base, args):
    with_runs = [c for c in cfgs.values() if c.runs]
    show_all = (args.show and args.show.strip().lower() == "all") or (not args.show and len([c for c in with_runs if c.known]) <= 12)
    if show_all:
        out = [c for c in with_runs if c.known or c is base]
    elif args.show:
        wanted = [s.strip() for s in args.show.split(",") if s.strip()]
        out = [c for c in with_runs if c.id in wanted or c.name in wanted]
    else:
        cand = [(c, mean_m(c, "jitter"), mean_m(c, "overshoot")) for c in with_runs
                if c is not base and c.known and mean_m(c, "overshoot") <= args.max_overshoot]
        front = sorted(pareto_front(cand), key=lambda x: x[1])
        rest = sorted([x for x in cand if x[0] not in [f[0] for f in front]], key=lambda x: x[1])
        out = [x[0] for x in (front + rest)[: args.n_show]]
    rest = sorted([c for c in out if c is not base], key=lambda c: fam_key(c, base))
    return ([base] if (base is not None and base.runs) else []) + rest


# ============================================================================================ PI study: draw functions
def d_trace(ax, c, color, win, ylim):
    r = c.runs[0]
    mask, _ = trace_window(r, win)
    ax.plot(r.t[mask], r.pwm[mask], color=color, lw=None)
    ax.set_ylim(*ylim)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("PWM duty (%)")


EMPH = {"other_alpha": 0.3}          # opacity of every non-baseline line in overlay graphs (1.0 = no emphasis)


def emph(c, base, first=True, lw=None):
    """Line kwargs that make the baseline stand out: baseline thick, opaque and on top; every other setting faded."""
    lw0 = lw if lw is not None else plt.rcParams["lines.linewidth"]
    if base is not None and c is base:
        return dict(lw=lw0 * 1.7, alpha=1.0 if first else 0.7, zorder=6)
    a = EMPH["other_alpha"]
    return dict(lw=lw0, alpha=a if first else a * 0.6, zorder=2)


def leg_opaque(leg):
    """Legend swatches stay fully readable even though the plotted lines are faded."""
    for h in getattr(leg, "legend_handles", None) or getattr(leg, "legendHandles", []):
        try:
            h.set_alpha(1.0)
        except Exception:                                                        # noqa: BLE001
            pass


def lc1(t):
    """Lower-case only the first letter (keeps Kp, Ki capitalised)."""
    return t[0].lower() + t[1:] if t else t


def d_overlay(ax, shown, cols, base, win, ylim):
    for c in shown:
        r = c.runs[0]
        mask, _ = trace_window(r, win)
        ax.plot(r.t[mask], r.pwm[mask], color=cols[c.name], label=label_of(c, base), **emph(c, base))
    ax.set_ylim(*ylim)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("PWM duty (%)")
    leg_opaque(ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=2 if len(shown) > 3 else 1, frameon=False))


def d_startup(ax, shown, cols, base, tgt, legend_loc="lower right"):
    for c in shown:
        for r in c.runs[:2]:
            first = r.rep == c.runs[0].rep
            m = r.t <= 1.2
            ax.plot(r.t[m], r.rpm[m], color=cols[c.name], ls="-" if first else "--", label=label_of(c, base) if first else None, **emph(c, base, first))
    ax.axhline(tgt, color="k", ls=":", lw=1)
    ax.axhspan(tgt * 0.95, tgt * 1.05, color="k", alpha=0.06)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw speed (RPM)")
    leg_opaque(ax.legend(loc=legend_loc))


def d_steady(ax, shown, cols, tgt, win, base=None):
    for c in shown:
        for r in c.runs[:2]:
            first = r.rep == c.runs[0].rep
            mask, _ = trace_window(r, win)
            ax.plot(r.t[mask], r.rpm[mask], color=cols[c.name], ls="-" if first else "--", **emph(c, base, first))
    ax.axhline(tgt, color="k", ls=":", lw=1)
    ax.set_ylim(tgt * 0.955, tgt * 1.045)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw speed (RPM)")


def d_bars(ax, items, labf, xl, base, bj):
    vals = [mean_m(c, "jitter") for c in items]
    bars = ax.bar([labf(c) for c in items], vals, color=[PALETTE[0] if c is base else PALETTE[1] for c in items], width=0.6)
    for b, c, v in zip(bars, items, vals):
        top = max([v] + [r["jitter"] for r in c.rows])
        ax.text(b.get_x() + b.get_width() / 2, top * 1.03, f"{v:.3f}", ha="center")
        if c is not base and base:
            ax.text(b.get_x() + b.get_width() / 2, v / 2, f"{MINUS}{(1 - v / bj) * 100:.0f}%", ha="center", color="white", fontweight="bold")
        ax.plot([b.get_x() + b.get_width() / 2] * len(c.rows), [r["jitter"] for r in c.rows], "k.", ms=5)
    ax.set_ylim(0, max(max(r["jitter"] for r in c.rows) for c in items) * 1.25)
    ax.set_ylabel("PWM jitter (% duty)")
    ax.set_xlabel(xl)
    ax.grid(axis="x", alpha=0)


def filter_pairs(known):
    """[( (kp, ki), [cfgs with/without the filter] )] for gain pairs that exist both with alpha = 1 and with alpha < 1."""
    g = {}
    for c in known:
        g.setdefault((round(c.kp, 3), round(c.ki, 2)), []).append(c)
    out = [(k, sorted(v, key=lambda c: -c.alpha)) for k, v in g.items()
           if any(abs(c.alpha - 1) < 1e-6 for c in v) and any(abs(c.alpha - 1) >= 1e-6 for c in v)]
    return sorted(out, key=lambda x: (-x[0][0], -x[0][1]))


def d_bars_grouped(ax, pairs, base, bj):
    xs, vals, cols_, labs, cfgs_ = [], [], [], [], []
    x = 0.0
    for (kp, ki), cs in pairs:
        for c in cs:
            xs.append(x)
            vals.append(mean_m(c, "jitter"))
            cols_.append("#5B7C99" if abs(c.alpha - 1) < 1e-6 else "#1E8449")
            labs.append(("no filter" if abs(c.alpha - 1) < 1e-6 else f"\u03b1 {fg(c.alpha)}") + f"\nKp {fg(kp)}, Ki {fk(ki)}")
            cfgs_.append(c)
            x += 1.0
        x += 0.6
    bars = ax.bar(xs, vals, color=cols_, width=0.75)
    for b, c, v in zip(bars, cfgs_, vals):
        top = max([v] + [r["jitter"] for r in c.rows])
        ax.text(b.get_x() + b.get_width() / 2, top * 1.03, f"{v:.3f}\novershoot {mean_m(c, 'overshoot'):.1f}%", ha="center", fontsize=9)
        ax.plot([b.get_x() + b.get_width() / 2] * len(c.rows), [r["jitter"] for r in c.rows], "k.", ms=5)
    ax.set_xticks(xs)
    ax.set_xticklabels(labs, fontsize=9)
    ax.set_ylim(0, max(max(r["jitter"] for r in c.rows) for c in cfgs_) * 1.4)
    ax.set_ylabel("PWM jitter (% duty)")
    ax.grid(axis="x", alpha=0)


def place_labels(ax, pts, labels):
    """Greedy label placement: for each point try 8 positions and keep the first that overlaps no marker or earlier label. Offsets in points."""
    ax.figure.canvas.draw()
    fs = plt.rcParams["font.size"]
    k = ax.figure.dpi / 72.0
    P = [ax.transData.transform(p) for p in pts]
    markers = [(x - 9 * k, y - 9 * k, 18 * k, 18 * k) for x, y in P]
    placed, out = [], []

    def hit(a, b):
        return not (a[0] + a[2] < b[0] or b[0] + b[2] < a[0] or a[1] + a[3] < b[1] or b[1] + b[3] < a[1])
    for (x, y), lab in zip(P, labels):
        w, h = (0.62 * fs * len(lab) + 2) * k, 1.25 * fs * k
        gap = 7 * k
        g2 = 2.3 * gap
        cands = [(gap, gap), (gap, -gap - h), (-gap - w, gap), (-gap - w, -gap - h), (-w / 2, gap + 3 * k), (-w / 2, -gap - h - 3 * k),
                 (gap + 4 * k, -h / 2), (-gap - w - 4 * k, -h / 2),
                 (g2, g2), (g2, -g2 - h), (-g2 - w, g2), (-g2 - w, -g2 - h), (-w / 2, g2 + 6 * k), (-w / 2, -g2 - h - 6 * k)]
        choice = cands[0]
        for dx, dy in cands:
            r = (x + dx, y + dy, w, h)
            if not any(hit(r, m) for m in markers) and not any(hit(r, q) for q in placed):
                choice = (dx, dy)
                break
        placed.append((x + choice[0], y + choice[1], w, h))
        out.append((choice[0] / k, choice[1] / k))
    return out


def d_tradeoff(ax, known, shown, base, args, big=False):
    cand = [(c, mean_m(c, "jitter"), mean_m(c, "overshoot")) for c in known if c is not base and mean_m(c, "overshoot") <= args.max_overshoot]
    front = {c.name for c, _, _ in pareto_front(cand)}
    for c in known:
        sel = c in shown
        col = PALETTE[0] if c is base else ("#1E8449" if c.name in front else "#7F8C8D")
        ax.scatter(mean_m(c, "overshoot"), mean_m(c, "jitter"), s=(170 if big else 110) if sel else (90 if big else 55), color=col,
                   edgecolor="k" if sel else "none", zorder=3)
    for lab, col in (("baseline", PALETTE[0]), (f"best trade-off (overshoot \u2264 {args.max_overshoot:g} %)", "#1E8449"), ("other", "#7F8C8D")):
        ax.scatter([], [], color=col, label=lab)
    ax.legend(loc="upper right")
    ax.set_xlabel("Startup overshoot (%)  \u2192 worse")
    ax.set_ylabel("PWM jitter (% duty)  \u2193 better")
    ax.set_xlim(left=-max(1.0, 0.03 * max(mean_m(c, "overshoot") for c in known)))
    ax.set_ylim(0, max(mean_m(c, "jitter") for c in known) * 1.16)
    ordered = sorted(known, key=lambda c: (mean_m(c, "overshoot"), mean_m(c, "jitter")))
    offs = place_labels(ax, [(mean_m(c, "overshoot"), mean_m(c, "jitter")) for c in ordered], [c.id for c in ordered])
    for c, off in zip(ordered, offs):
        ax.annotate(c.id, (mean_m(c, "overshoot"), mean_m(c, "jitter")), textcoords="offset points", xytext=off,
                    fontweight="bold" if c in shown else "normal")


def d_ranking(ax, known, base):
    order = sorted(known, key=lambda c: mean_m(c, "jitter"), reverse=True)
    ys = np.arange(len(order))
    cols = ["#1E8449" if mean_m(c, "overshoot") <= 5 else ("#E08A00" if mean_m(c, "overshoot") <= 15 else "#C0392B") for c in order]
    ax.barh(ys, [mean_m(c, "jitter") for c in order], color=cols, height=0.65)
    ax.set_yticks(ys)
    ax.set_yticklabels([label_of(c, base) for c in order])
    for y, c in zip(ys, order):
        ax.text(mean_m(c, "jitter") * 1.02, y, f"{mean_m(c, 'jitter'):.3f}   (overshoot {mean_m(c, 'overshoot'):.1f} %)", va="center")
    ax.set_xlim(0, max(mean_m(c, "jitter") for c in order) * 1.65)
    ax.set_xlabel("PWM jitter (% duty), lower is flatter")
    ax.grid(axis="y", alpha=0)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=col, label=lab) for lab, col in (("overshoot \u2264 5 %", "#1E8449"), ("5\u201315 %", "#E08A00"), ("> 15 %", "#C0392B"))],
              loc="upper right", title="Startup overshoot")


def d_peaks(ax, shown, base, pk, ylabel="PWM ripple amplitude (% duty, zero-to-peak)", fmt="{:.3f}"):
    xs = np.arange(len(shown))
    w = 0.38
    p1 = [pk[c.name][0] for c in shown]
    p2 = [pk[c.name][1] for c in shown]
    b1 = ax.bar(xs - w / 2, p1, w, color="#8FB3D9", label="1\u00d7 shaft frequency")
    b2 = ax.bar(xs + w / 2, p2, w, color="#1B4F72", label="2\u00d7 shaft frequency")
    for b, v in zip(list(b1) + list(b2), p1 + p2):
        ax.text(b.get_x() + b.get_width() / 2, v * 1.02, fmt.format(v), ha="center", fontsize=8.5)
    if base is not None and base in shown:
        ax.axhline(pk[base.name][0], color="#8FB3D9", ls="--", lw=1)
        ax.axhline(pk[base.name][1], color="#1B4F72", ls="--", lw=1)
    ax.set_xticks(xs)
    ax.set_xticklabels([c.id for c in shown])
    ax.set_xlabel("Setting (dashed lines = baseline)")
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, max(p1 + p2) * 1.2)
    ax.legend(loc="upper right")
    ax.grid(axis="x", alpha=0)


# ---- speed (RPM) comparison drawers
def d_rtrace(ax, c, color, win, ylim, tgt):
    r = c.runs[0]
    mask, _ = trace_window(r, win)
    ax.plot(r.t[mask], r.rpm[mask], color=color, lw=None)
    ax.axhline(tgt, color="k", ls=":", lw=1)
    ax.set_ylim(*ylim)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw speed (RPM)")


def d_roverlay(ax, shown, cols, base, win, ylim, tgt):
    for c in shown:
        r = c.runs[0]
        mask, _ = trace_window(r, win)
        ax.plot(r.t[mask], r.rpm[mask], color=cols[c.name], label=label_of(c, base), **emph(c, base, lw=1.4))
    ax.axhline(tgt, color="k", ls=":", lw=1)
    ax.set_ylim(*ylim)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw speed (RPM)")
    leg_opaque(ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=2 if len(shown) > 3 else 1, frameon=False))


def d_step(axr, axp, c, color, tgt, rlim, plim):
    """One setting's step response: speed (both repeats, +-5 % band, overshoot and settling marked) and, optionally, the PWM command below."""
    for r in c.runs[:2]:
        first = r is c.runs[0]
        m = r.t <= 1.2
        axr.plot(r.t[m], r.rpm[m], color=color, lw=None, ls="-" if first else "--", alpha=1 if first else 0.55, label=f"repeat {r.rep}")
        if axp is not None:
            axp.plot(r.t[m], r.pwm[m], color=color, lw=None, ls="-" if first else "--", alpha=1 if first else 0.55)
    axr.axhline(tgt, color="k", ls=":", lw=1)
    axr.axhspan(tgt * 0.95, tgt * 1.05, color="k", alpha=0.06)
    r0 = c.runs[0]
    m = r0.t <= 1.2
    i = int(np.argmax(r0.rpm[m]))
    t_pk, v_pk = float(r0.t[m][i]), float(r0.rpm[m][i])
    ov, st = mean_m(c, "overshoot"), mean_m(c, "settle")
    if ov > 5.0:                       # a real overshoot feature: point at the peak
        axr.annotate(f"peak {v_pk:.0f} RPM\n(overshoot {ov:.1f} %)", xy=(t_pk, v_pk), xytext=(t_pk + 0.13, min(v_pk + 0.03 * rlim[1], rlim[1] * 0.80)),
                     arrowprops=dict(arrowstyle="-", color="gray", lw=1), va="bottom")
    else:                              # no overshoot beyond the ripple: do not point at a noise peak
        axr.text(0.97, 0.62, f"no overshoot beyond the ripple\n(max {v_pk:.0f} RPM, +{ov:.1f} %)", transform=axr.transAxes, ha="right", va="center",
                 fontsize=plt.rcParams["font.size"] * 0.85)
    if st >= 0:
        for a in ([axr] + ([axp] if axp is not None else [])):
            a.axvline(st, color="gray", ls="--", lw=1.2)
        axr.text(st + 0.02, rlim[1] * 0.12, f"settles within \u00b15 %\nat {st:.2f} s", va="bottom", fontsize=plt.rcParams["font.size"] * 0.85)
    axr.set_ylim(*rlim)
    axr.set_ylabel("Speed (RPM)")
    axr.legend(loc="lower right")
    if axp is not None:
        axp.set_ylim(*plim)
        axp.set_ylabel("PWM duty (%)")
        axp.set_xlabel("Time (s)")
    else:
        axr.set_xlabel("Time (s)")


def d_rpm_pwm(axr, axp, c, color, win, rlim, plim, tgt):
    """One setting at steady state: speed on top, PWM command below, shared time axis."""
    r = c.runs[0]
    mask, _ = trace_window(r, win)
    axr.plot(r.t[mask], r.rpm[mask], color=color, lw=1.4)
    axr.axhline(tgt, color="k", ls=":", lw=1)
    axp.plot(r.t[mask], r.pwm[mask], color=color, lw=1.6)
    axr.set_ylim(*rlim)
    axp.set_ylim(*plim)
    axr.set_ylabel("Speed (RPM)")
    axp.set_ylabel("PWM duty (%)")
    axp.set_xlabel("Time (s)")


def d_rbars(ax, items, cols, base, bs):
    vals = [mean_m(c, "rpm_std") for c in items]
    bars = ax.bar([c.id for c in items], vals, color=[cols.get(c.name, "#7F8C8D") for c in items], width=0.65)
    for b, c, v in zip(bars, items, vals):
        top = max([v] + [r["rpm_std"] for r in c.rows])
        ax.text(b.get_x() + b.get_width() / 2, top * 1.03, f"{v:.2f}", ha="center", fontsize=9)
        ax.plot([b.get_x() + b.get_width() / 2] * len(c.rows), [r["rpm_std"] for r in c.rows], "k.", ms=4)
    if base is not None and base in items:
        ax.axhline(mean_m(base, "rpm_std"), color=PALETTE[0], ls="--", lw=1)
    ax.set_ylim(0, max(max(r["rpm_std"] for r in c.rows) for c in items) * 1.25)
    ax.set_ylabel("Speed ripple, std at steady state (RPM)")
    ax.set_xlabel("Setting (dashed line = baseline)")
    ax.grid(axis="x", alpha=0)


def d_pwm_vs_rpm(ax, known, shown, cols, base):
    for c in known:
        ax.scatter(mean_m(c, "jitter"), mean_m(c, "rpm_std"), s=100, color=cols.get(c.name, "#7F8C8D"), edgecolor="k" if c in shown else "none", zorder=3)
    ax.set_xlabel("PWM jitter (% duty)  \u2190 flatter command")
    ax.set_ylabel("Speed ripple, std (RPM)  \u2193 steadier speed")
    ax.set_xlim(0, max(mean_m(c, "jitter") for c in known) * 1.18)
    ax.set_ylim(0, max(mean_m(c, "rpm_std") for c in known) * 1.25)
    ordered = sorted(known, key=lambda c: (mean_m(c, "jitter"), mean_m(c, "rpm_std")))
    offs = place_labels(ax, [(mean_m(c, "jitter"), mean_m(c, "rpm_std")) for c in ordered], [c.id for c in ordered])
    for c, off in zip(ordered, offs):
        ax.annotate(c.id, (mean_m(c, "jitter"), mean_m(c, "rpm_std")), textcoords="offset points", xytext=off, fontweight="bold" if c in shown else "normal")


def d_spec(ax, grid, shown, cols, base, data, which, ol_ref):
    if which == "speed" and ol_ref is not None:
        o, a, _ = ol_ref
        ax.plot(o, a, color="k", lw=2.4, label="No controller (open-loop, constant PWM)")
    for c in shown:
        if c.name not in data:
            continue
        y = data[c.name][0 if which == "speed" else 1]
        ax.plot(grid, y, color=cols[c.name], label=("PI: " if which == "speed" else "") + label_of(c, base), **emph(c, base))
    ax.set_xlim(0, 4.5)
    ax.set_xticks([0, 1, 2, 3, 4])
    ax.set_xticklabels(["0", "1\u00d7", "2\u00d7", "3\u00d7", "4\u00d7"])
    ax.set_xlabel("Frequency in multiples of shaft rotation frequency")
    ax.set_ylabel("Speed-ripple amplitude (RPM)" if which == "speed" else "PWM-command ripple amplitude (% duty)")
    ax.set_ylim(bottom=0)
    leg_opaque(ax.legend(loc="upper right"))


# ============================================================================================ PI study: section
OL_RUNS: list = []


def pi_section(runs, raw_rows, args, out):
    cfgs = build_cfgs(runs, raw_rows)
    if not cfgs:
        return
    base = find_baseline(cfgs)
    shown = choose_shown(cfgs, base, args)
    cols = assign_colors(cfgs, base)
    print(f"\n[PI study] {len(cfgs)} configs ({sum(1 for c in cfgs.values() if c.runs)} with CSVs). "
          f"Baseline: {base.name if base else 'none found'}. Time-series graphs for: {[c.id for c in shown]}")
    bj = mean_m(base, "jitter") if base else float("nan")
    tgts = sorted({r.target for c in cfgs.values() for r in c.runs})
    tgt = tgts[0] if tgts else 460.0
    known = [c for c in cfgs.values() if c.known and c.rows]

    bs = mean_m(base, "rpm_std") if base else float("nan")

    def cfg_peaks(c):
        rr1, rr2, pp1, pp2 = [], [], [], []
        for r in c.runs:
            xr, xp = steady_arr(r, r.rpm), steady_arr(r, r.pwm)
            if xr is None or xp is None:
                continue
            sh = float(np.mean(xr)) / 60.0
            fr, a = spectrum(xr)
            rr1.append(order_peak(fr, a, sh)); rr2.append(order_peak(fr, a, 2 * sh))
            fr, a = spectrum(xp)
            pp1.append(order_peak(fr, a, sh)); pp2.append(order_peak(fr, a, 2 * sh))
        f = lambda v: float(np.mean(v)) if v else float("nan")                      # noqa: E731
        return dict(rpm_amp_1x=f(rr1), rpm_amp_2x=f(rr2), pwm_amp_1x_pct=f(pp1), pwm_amp_2x_pct=f(pp2))
    allpk = {c.name: cfg_peaks(c) for c in cfgs.values()}

    pd.DataFrame([dict(id=c.id, config=c.name, kp=c.kp, ki=c.ki, alpha=c.alpha, runs=len(c.rows), source=c.source,
                       jitter_pct=mean_m(c, "jitter"), pwm_std_pct=mean_m(c, "pwm_std"), rpm_std=mean_m(c, "rpm_std"),
                       rpm_mean=mean_m(c, "rpm_mean"), mean_error_rpm=mean_m(c, "sse"),
                       overshoot_pct=mean_m(c, "overshoot"), settle_s=mean_m(c, "settle"),
                       jitter_vs_baseline_pct=(mean_m(c, "jitter") / bj - 1) * 100 if (base and c.known) else np.nan,
                       rpm_std_vs_baseline_pct=(mean_m(c, "rpm_std") / bs - 1) * 100 if (base and c.known) else np.nan, **allpk[c.name])
                  for c in cfgs.values()]).sort_values("jitter_pct").to_csv(os.path.join(out.dir, "metrics_pi_configs.csv"), index=False)

    def red_txt(c):
        j = mean_m(c, "jitter")
        return "" if (c is base or not base) else f"  ({MINUS}{(1 - j / bj) * 100:.0f} %)"

    # ---------------------------------------------------------------- PWM traces
    if shown:
        wins = [trace_window(c.runs[0], args.window) for c in shown]
        ymin = min(np.nanmin(c.runs[0].pwm[w[0]]) for c, w in zip(shown, wins))
        ymax = max(np.nanmax(c.runs[0].pwm[w[0]]) for c, w in zip(shown, wins))
        pad = 0.12 * (ymax - ymin)
        ylim = (ymin - pad, ymax + pad)
        fig, axs = plt.subplots(len(shown), 1, figsize=(10, (2.2 if len(shown) <= 5 else (1.75 if len(shown) <= 8 else 1.5)) * len(shown) + 1.2), sharex=True, sharey=True, squeeze=False)
        for ax, c in zip(axs[:, 0], shown):
            d_trace(ax, c, cols[c.name], args.window, ylim)
            ax.set_xlabel("")
            ax.lines[0].set_linewidth(1.5)
            ax.set_title(f"{label_of(c, base)}   \u2192  PWM jitter {mean_m(c, 'jitter'):.3f} %{red_txt(c)}", loc="left", fontsize=10.5,
                         color=cols[c.name], fontweight="bold")
        axs[-1, 0].set_xlabel("Time (s)")
        fig.suptitle(f"Motor PWM command at steady state, {tgt:g} RPM target (same vertical scale on all panels)", y=1.0, fontsize=12)
        fig.tight_layout()
        txt = "; ".join(f"{c.id} {mean_m(c, 'jitter'):.3f} %" for c in shown)
        cap = (f"Motor PWM command at steady state for a {tgt:g} RPM step. Same vertical scale in all panels; repeat 1 shown. "
               f"PWM jitter (RMS sample-to-sample change over the last 40 % of the run, mean of repeats): {txt}.")
        out.comp(fig, "01_pwm_traces", cap)
        for c in shown:
            out.single(f"01_pwm_trace_{c.id}", lambda ax, c=c: d_trace(ax, c, cols[c.name], args.window, ylim),
                       title=f"{label_of(c, base)}  \u2192  PWM jitter {mean_m(c, 'jitter'):.3f} %{red_txt(c)}",
                       caption=f"PWM command at steady state, {label_of(c, base)}: jitter {mean_m(c, 'jitter'):.3f} % duty{red_txt(c).strip()}. Same vertical scale as the other 01_pwm_trace graphs.")
        out.single("01_pwm_overlay", lambda ax: d_overlay(ax, shown, cols, base, args.window, ylim), title="PWM command at steady state: all settings overlaid",
                   caption="All shown settings overlaid on one axis (repeat 1).")
        for suf, ttl, cs in family_sets(shown, base):
            out.single(f"01_pwm_overlay_{suf}", lambda ax, cs=cs: d_overlay(ax, cs, cols, base, args.window, ylim), title=f"PWM command: {lc1(ttl)}",
                       caption=f"{ttl}: PWM command at steady state, repeat 1, same vertical scale as the other PWM graphs.")

    # ---------------------------------------------------------------- step response
    if shown:
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
        d_startup(ax[0], shown, cols, base, tgt)
        d_steady(ax[1], shown, cols, tgt, args.window, base)
        ax[0].set_title(f"(a) Startup step 0 \u2192 {tgt:g} RPM (grey band = \u00b15 %)")
        ax[1].set_title("(b) Steady state (solid = run 1, dashed = run 2)")
        fig.tight_layout()
        txt = "; ".join(f"{c.id} {mean_m(c, 'overshoot'):.1f} %" for c in shown)
        out.comp(fig, "02_step_response", f"Speed response to a step from rest to {tgt:g} RPM. (a) Startup; (b) steady state. Overshoot (mean of repeats): {txt}.")
        out.single("02a_step_startup", lambda ax: d_startup(ax, shown, cols, base, tgt), title=f"Startup step 0 \u2192 {tgt:g} RPM (grey band = \u00b15 %)",
                   caption=f"Startup response to the {tgt:g} RPM step; overshoot {txt}.")
        out.single("02b_step_steady", lambda ax: d_steady(ax, shown, cols, tgt, args.window, base), title="Steady state (solid = run 1, dashed = run 2)",
                   caption="Steady-state speed, same shaft-order ripple for all settings.")
        for suf, ttl, cs in family_sets(shown, base):
            out.single(f"02a_step_startup_{suf}", lambda ax, cs=cs: d_startup(ax, cs, cols, base, tgt), title=f"Startup step: {lc1(ttl)}",
                       caption=f"{ttl}: startup response to the {tgt:g} RPM step (solid = run 1, dashed = run 2).")

    # ---------------------------------------------------------------- tuning summary: Kp / alpha / Ki / trade-off
    panels = []          # (key, short name, subtitle, draw(ax))
    if base:
        kp_s = sorted([c for c in known if abs(c.alpha - 1) < 1e-6 and abs(c.ki - base.ki) < 0.05], key=lambda c: -c.kp)
        if len(kp_s) >= 2:
            panels.append(("kp", "Kp", f"(Ki = {fk(base.ki)}, no filter)",
                           lambda ax, it=kp_s: d_bars(ax, it, lambda c: fg(c.kp), "Proportional gain Kp", base, bj)))
        al_s = sorted([c for c in known if abs(c.kp - base.kp) < 1e-3 and abs(c.ki - base.ki) < 0.05], key=lambda c: -c.alpha)
        if len(al_s) >= 2:
            panels.append(("alpha", "filter \u03b1", f"(Kp = {fg(base.kp)}, Ki = {fk(base.ki)})",
                           lambda ax, it=al_s: d_bars(ax, it, lambda c: fg(c.alpha), "Feedback filter \u03b1 (smaller = stronger)", base, bj)))
        groups = {}
        for c in known:
            if abs(c.alpha - 1) < 1e-6 and abs(c.kp - base.kp) > 1e-3:
                groups.setdefault(round(c.kp, 3), []).append(c)
        best = max(groups.values(), key=lambda g: len({round(x.ki, 2) for x in g}), default=[])
        if len({round(x.ki, 2) for x in best}) >= 2:
            ks = sorted(best, key=lambda c: -c.ki)
            panels.append(("ki", "Ki", f"(Kp = {fg(ks[0].kp)}, no filter)",
                           lambda ax, it=ks: d_bars(ax, it, lambda c: fk(c.ki), "Integral gain Ki", base, bj)))
        if len(al_s) < 2:            # no alpha series at the baseline gains: compare with / without filter at each fixed (Kp, Ki)
            pairs = filter_pairs(known)
            if pairs:
                panels.append(("filter", "the feedback filter", "(same Kp and Ki, with vs without filter)",
                               lambda ax, pr=pairs: d_bars_grouped(ax, pr, base, bj)))
    npan = len(panels) + 1
    fig, axs = plt.subplots(1, npan, figsize=(4.9 * npan, 4.6), squeeze=False)
    axs = axs[0]
    for i, (key, nm, sub, drawf) in enumerate(panels):
        drawf(axs[i])
        axs[i].set_title(f"({'abcd'[i]}) Effect of {nm}\n{sub}")
    d_tradeoff(axs[-1], known, shown, base, args)
    axs[-1].set_title(f"({'abcd'[len(panels)]}) Trade-off, all {len(known)} settings")
    fig.tight_layout()
    best_c = min(known, key=lambda c: mean_m(c, "jitter") if mean_m(c, "overshoot") <= args.max_overshoot else 9e9)
    out.comp(fig, "03_tuning_summary",
             f"Effect of the PI settings on PWM jitter and startup overshoot (bars: mean of repeats, dots: individual repeats). "
             f"Lowest jitter with overshoot \u2264 {args.max_overshoot:g} %: {best_c.id} ({mean_m(best_c, 'jitter'):.3f} % duty, {mean_m(best_c, 'overshoot'):.1f} % overshoot).")
    for key, nm, sub, drawf in panels:
        out.single(f"03_effect_{key}", drawf, title=f"Effect of {nm} on PWM jitter {sub}",
                   caption=f"PWM jitter versus {nm} {sub}; bars = mean of repeats, dots = repeats.")
    out.single("03_tradeoff", lambda ax: d_tradeoff(ax, known, shown, base, args, big=True), title="Trade-off: PWM jitter vs startup overshoot",
               caption=f"Every setting tested: jitter versus overshoot. Best trade-off (overshoot \u2264 {args.max_overshoot:g} %): {best_c.id}.")
    out.single("03_ranking", lambda ax: d_ranking(ax, known, base), title="All settings ranked by PWM jitter",
               caption="All settings ranked by PWM jitter, bar colour = startup overshoot.", size=(max(10, args.slide_size[0]), max(5.625, 0.5 * len(known) + 1.8)))

    # ---------------------------------------------------------------- ripple orders
    if shown:
        grid = np.linspace(0, 4.5, 1800)
        ol_ref = None
        ol_near = [r for r in OL_RUNS if steady_arr(r, r.rpm) is not None and abs(np.mean(steady_arr(r, r.rpm)) - tgt) < 0.06 * tgt]
        if ol_near:
            ol_ref = order_spectrum(ol_near[0], ol_near[0].rpm)
        data, pk = {}, {}
        for c in shown:
            sr, sp, p1, p2 = [], [], [], []
            for r in c.runs:
                s1, s2 = order_spectrum(r, r.rpm), order_spectrum(r, r.pwm)
                if s1 is None or s2 is None:
                    continue
                sr.append(np.interp(grid, s1[0], s1[1]))
                sp.append(np.interp(grid, s2[0], s2[1]))
                fr, a2 = spectrum(steady_arr(r, r.pwm))
                sh = shaft_hz(r)
                p1.append(order_peak(fr, a2, sh))
                p2.append(order_peak(fr, a2, 2 * sh))
            if sr:
                data[c.name] = (np.mean(sr, axis=0), np.mean(sp, axis=0))
                pk[c.name] = (float(np.mean(p1)), float(np.mean(p2)))
        if data:
            fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
            d_spec(ax[0], grid, shown, cols, base, data, "speed", ol_ref)
            d_spec(ax[1], grid, shown, cols, base, data, "pwm", None)
            ax[0].set_title("(a) Speed ripple")
            ax[1].set_title("(b) PWM ripple reaching the motor")
            fig.tight_layout()
            byname = {c.name: c for c in shown}
            txt = "; ".join(f"{byname[k].id}: 1\u00d7 {v[0]:.3f}, 2\u00d7 {v[1]:.3f} % duty" for k, v in pk.items())
            out.comp(fig, "04_ripple_orders", f"Amplitude spectra (zero-to-peak, Hann window, 15 s steady state) versus shaft order. (a) Speed; (b) PWM command. PWM ripple peaks: {txt}.")
            out.single("04a_spectrum_speed", lambda ax: d_spec(ax, grid, shown, cols, base, data, "speed", ol_ref), title="Speed ripple by shaft order",
                       caption="Speed-ripple spectrum versus shaft order" + (" with the no-controller reference." if ol_ref else "."))
            out.single("04b_spectrum_pwm", lambda ax: d_spec(ax, grid, shown, cols, base, data, "pwm", None), title="PWM ripple by shaft order",
                       caption=f"PWM-command ripple versus shaft order. Peaks: {txt}.")
            sh_pk = [c for c in shown if c.name in pk]
            fig, ax = plt.subplots(figsize=(max(8, 1.35 * len(sh_pk) + 3), 4.8))
            d_peaks(ax, sh_pk, base, pk)
            ax.set_title("PWM ripple at the first two shaft orders, per setting")
            fig.tight_layout()
            pcap = f"PWM-command ripple amplitude at 1\u00d7 and 2\u00d7 the shaft frequency for each setting (mean of repeats). {txt}."
            out.comp(fig, "04c_ripple_peaks", pcap)
            out.single("04c_ripple_peaks", lambda ax: d_peaks(ax, sh_pk, base, pk), title="PWM ripple at 1\u00d7 and 2\u00d7 shaft frequency, per setting", caption=pcap,
                       size=(max(10, 1.2 * len(sh_pk) + 3), 5.625))

    # ---------------------------------------------------------------- speed (RPM) comparison
    def rdelta(c):
        v = mean_m(c, "rpm_std")
        if c is base or not base or not c.known:
            return ""
        if abs(v / bs - 1) < 0.005:
            return "  (\u2248 0 %)"
        return f"  ({MINUS if v < bs else '+'}{abs(v / bs - 1) * 100:.0f} %)"
    if shown:
        wins = [trace_window(c.runs[0], args.window) for c in shown]
        rmin = min(np.nanmin(c.runs[0].rpm[w[0]]) for c, w in zip(shown, wins))
        rmax = max(np.nanmax(c.runs[0].rpm[w[0]]) for c, w in zip(shown, wins))
        rpad = 0.12 * (rmax - rmin)
        rlim = (rmin - rpad, rmax + rpad)

        def rtitle(c):
            v = mean_m(c, "rpm_std")
            return f"{label_of(c, base)}   \u2192  speed ripple (std) {v:.2f} RPM = {v / tgt * 100:.2f} % of target{rdelta(c)}"
        fig, axs = plt.subplots(len(shown), 1, figsize=(10, (2.2 if len(shown) <= 5 else (1.75 if len(shown) <= 8 else 1.5)) * len(shown) + 1.2),
                                sharex=True, sharey=True, squeeze=False)
        for ax, c in zip(axs[:, 0], shown):
            d_rtrace(ax, c, cols[c.name], args.window, rlim, tgt)
            ax.set_xlabel("")
            ax.lines[0].set_linewidth(1.2)
            ax.set_title(rtitle(c), loc="left", fontsize=10.5, color=cols[c.name], fontweight="bold")
        axs[-1, 0].set_xlabel("Time (s)")
        fig.suptitle(f"Raw motor speed at steady state, {tgt:g} RPM target (same vertical scale on all panels)", y=1.0, fontsize=12)
        fig.tight_layout()
        rtxt = "; ".join(f"{c.id} {mean_m(c, 'rpm_std'):.2f} RPM" for c in shown)
        out.comp(fig, "06_rpm_traces", f"Raw motor speed at steady state for a {tgt:g} RPM step. Same vertical scale in all panels; repeat 1 shown. "
                 f"Speed ripple (std over the last 40 % of the run, mean of repeats): {rtxt}.")
        for c in shown:
            out.single(f"06_rpm_trace_{c.id}", lambda ax, c=c: d_rtrace(ax, c, cols[c.name], args.window, rlim, tgt), title=rtitle(c),
                       caption=f"Raw speed at steady state, {label_of(c, base)}: std {mean_m(c, 'rpm_std'):.2f} RPM{rdelta(c).strip()}. Same vertical scale as the other 06_rpm_trace graphs.")
        out.single("06_rpm_overlay", lambda ax: d_roverlay(ax, shown, cols, base, args.window, rlim, tgt), title="Raw motor speed at steady state: all settings overlaid",
                   caption="All shown settings overlaid on one axis (repeat 1).")
        for suf, ttl, cs in family_sets(shown, base):
            out.single(f"06_rpm_overlay_{suf}", lambda ax, cs=cs: d_roverlay(ax, cs, cols, base, args.window, rlim, tgt), title=f"Raw speed: {lc1(ttl)}",
                       caption=f"{ttl}: raw speed at steady state, repeat 1, same vertical scale as the other speed graphs.")

    # speed summary: ripple size, shaft-order content, and PWM vs speed
    allc = sorted([c for c in known if c.rows], key=lambda c: fam_key(c, base))
    rpk = {c.name: (allpk[c.name]["rpm_amp_1x"], allpk[c.name]["rpm_amp_2x"]) for c in shown if np.isfinite(allpk[c.name]["rpm_amp_1x"])}
    sh_r = [c for c in shown if c.name in rpk]
    if allc:
        npan = 3 if sh_r else 2
        fig, axs = plt.subplots(1, npan, figsize=(max(14, 0.5 * len(allc) + 11), 4.8), squeeze=False)
        axs = axs[0]
        d_rbars(axs[0], allc, cols, base, bs)
        axs[0].set_title("(a) Speed ripple per setting")
        k = 1
        if sh_r:
            d_peaks(axs[k], sh_r, base, rpk, ylabel="Speed-ripple amplitude (RPM, zero-to-peak)", fmt="{:.2f}")
            axs[k].set_title("(b) Speed ripple at 1\u00d7 and 2\u00d7 shaft frequency")
            k += 1
        d_pwm_vs_rpm(axs[k], allc, shown, cols, base)
        axs[k].set_title(f"({'abc'[k]}) Flatter PWM vs steadier speed")
        fig.tight_layout()
        lo, hi = min(allc, key=lambda c: mean_m(c, "rpm_std")), max(allc, key=lambda c: mean_m(c, "rpm_std"))
        ccap = (f"Speed comparison across settings: (a) speed ripple (std) per setting, (b) shaft-order components of the speed ripple, (c) PWM jitter versus speed ripple. "
                f"Speed ripple ranges from {mean_m(lo, 'rpm_std'):.2f} RPM ({lo.id}) to {mean_m(hi, 'rpm_std'):.2f} RPM ({hi.id}); "
                f"the baseline is {bs:.2f} RPM." if base else "Speed comparison across settings.")
        out.comp(fig, "07_rpm_summary", ccap)
        out.single("07a_rpm_std", lambda ax: d_rbars(ax, allc, cols, base, bs), title="Speed ripple (std) per setting",
                   caption=ccap, size=(max(10, 0.9 * len(allc) + 3), 5.625))
        if sh_r:
            out.single("07b_rpm_ripple_peaks", lambda ax: d_peaks(ax, sh_r, base, rpk, ylabel="Speed-ripple amplitude (RPM, zero-to-peak)", fmt="{:.2f}"),
                       title="Speed ripple at 1\u00d7 and 2\u00d7 shaft frequency, per setting", caption=ccap, size=(max(10, 1.2 * len(sh_r) + 3), 5.625))
        out.single("07c_pwm_vs_rpm", lambda ax: d_pwm_vs_rpm(ax, allc, shown, cols, base), title="Flatter PWM vs steadier speed (one point per setting)", caption=ccap)

    # ---------------------------------------------------------------- per-setting step responses and speed + PWM graphs
    if shown:
        rmax_s = max(float(np.nanmax(r.rpm[r.t <= 1.2])) for c in shown for r in c.runs[:2])
        pmin_s = min(float(np.nanmin(r.pwm[r.t <= 1.2])) for c in shown for r in c.runs[:2])
        pmax_s = max(float(np.nanmax(r.pwm[r.t <= 1.2])) for c in shown for r in c.runs[:2])
        ppad = 0.08 * (pmax_s - pmin_s)
        rlim_s, plim_s = (0.0, rmax_s * 1.25), (max(0.0, pmin_s - ppad), pmax_s + ppad)

        def step_title(c):
            st = mean_m(c, "settle")
            return (f"{label_of(c, base)}   \u2192  overshoot {mean_m(c, 'overshoot'):.1f} %, "
                    + (f"settles in {st:.2f} s" if st >= 0 else "does not settle within the window"))
        fig, axs = plt.subplots(len(shown), 1, figsize=(10, (2.4 if len(shown) <= 5 else (1.9 if len(shown) <= 8 else 1.6)) * len(shown) + 1.2),
                                sharex=True, sharey=True, squeeze=False)
        for ax, c in zip(axs[:, 0], shown):
            d_step(ax, None, c, cols[c.name], tgt, rlim_s, None)
            ax.set_xlabel("")
            ax.set_title(step_title(c), loc="left", fontsize=10.5, color=cols[c.name], fontweight="bold")
        axs[-1, 0].set_xlabel("Time (s)")
        fig.suptitle(f"Step response 0 \u2192 {tgt:g} RPM, one panel per setting (solid = repeat 1, dashed = repeat 2; grey band = \u00b15 %)", y=1.0, fontsize=12)
        fig.tight_layout()
        stxt = "; ".join(f"{c.id} {mean_m(c, 'overshoot'):.1f} % / {mean_m(c, 'settle'):.2f} s" for c in shown)
        out.comp(fig, "08_step_by_setting", f"Step response from rest to {tgt:g} RPM for each setting on the same axes. Overshoot / settling time to \u00b15 % (mean of repeats): {stxt}.")
        for c in shown:
            out.single2(f"08_step_{c.id}", lambda a1, a2, c=c: d_step(a1, a2, c, cols[c.name], tgt, rlim_s, plim_s), title=step_title(c),
                        caption=f"Step response of {label_of(c, base)}: speed (top) and PWM command (bottom), both repeats. Overshoot {mean_m(c, 'overshoot'):.1f} %, settling {mean_m(c, 'settle'):.2f} s. "
                                f"Same axes as the other 08_step graphs.")
            out.single2(f"09_rpm_pwm_{c.id}", lambda a1, a2, c=c: d_rpm_pwm(a1, a2, c, cols[c.name], args.window, rlim, ylim, tgt),
                        title=f"{label_of(c, base)}:  speed ripple {mean_m(c, 'rpm_std'):.2f} RPM,  PWM jitter {mean_m(c, 'jitter'):.3f} %",
                        caption=f"Steady-state speed (top) and PWM command (bottom) for {label_of(c, base)}; same axes as the other 09_rpm_pwm graphs.")

    # ---------------------------------------------------------------- settings table
    def grp(c):
        if c is base:
            return 0
        if abs(c.alpha - 1) < 1e-6:
            return 1
        if base and abs(c.kp - base.kp) < 1e-3 and abs(c.ki - base.ki) < 0.05:
            return 2
        return 3
    order = sorted(cfgs.values(), key=lambda c: (grp(c), -c.kp if grp(c) == 1 else 0, c.id))
    hdr = ["ID", "What changed", "Kp", "Ki", "Filter \u03b1", "PWM jitter\n(% duty)", "vs baseline", "Overshoot\n(%)", "Settling to\n\u00b15 % (s)", "RPM std\n(RPM)", "RPM std\nvs baseline", "runs"]
    data_t = []
    for c in order:
        j = mean_m(c, "jitter")
        what = "Baseline (original design)" if c is base else (label_of(c, base).split(": ", 1)[-1] if c.known else "(gains unknown: name the file kp..)")
        vs = "\u2014" if (c is base or not base or not c.known) else (f"{MINUS}{abs(j / bj - 1) * 100:.0f} %" if j < bj else f"+{(j / bj - 1) * 100:.0f} %")
        data_t.append([c.id, what, f"{c.kp:.2f}" if c.known else "\u2014", f"{c.ki:.1f}" if c.known else "\u2014",
                       ("1.0 (off)" if abs(c.alpha - 1) < 1e-6 else fg(c.alpha)) if c.known else "\u2014", f"{j:.3f}", vs,
                       f"{mean_m(c, 'overshoot'):.1f}", f"{mean_m(c, 'settle'):.2f}", f"{mean_m(c, 'rpm_std'):.2f}",
                       "\u2014" if (c is base or not base or not c.known) else ("\u2248 0 %" if abs(mean_m(c, "rpm_std") / bs - 1) < 0.005 else (f"{MINUS}{abs(mean_m(c, 'rpm_std') / bs - 1) * 100:.0f} %" if mean_m(c, "rpm_std") < bs else f"+{(mean_m(c, 'rpm_std') / bs - 1) * 100:.0f} %")),
                       str(len(c.rows))])
    fig, ax = plt.subplots(figsize=(13.5, 0.42 * len(data_t) + 1.3))
    ax.axis("off")
    tb = ax.table(cellText=data_t, colLabels=hdr, loc="upper center", cellLoc="center", colWidths=[.04, .19, .05, .05, .08, .09, .085, .085, .09, .075, .09, .04])
    tb.auto_set_font_size(False)
    tb.set_fontsize(11)
    tb.scale(1, 1.7)
    ovr_col = {}
    for row in data_t:
        o = float(row[7])
        ovr_col[row[0]] = "#E8F6EF" if o <= 5 else ("#FDF2DC" if o <= 15 else "#FBE3E0")
    for (r_, c_), cell in tb.get_celld().items():
        cell.set_edgecolor("#BBBBBB")
        if r_ == 0:
            cell.set_facecolor("#2C3E50")
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
            cell.set_height(cell.get_height() * 1.3)
        else:
            rid = data_t[r_ - 1][0]
            cell.set_facecolor("#E3EAF4" if (base and rid == base.id) else ovr_col[rid])
            if c_ in (5, 7, 9):
                cell.get_text().set_fontweight("bold")
    pd.DataFrame(data_t, columns=[h.replace("\n", " ") for h in hdr]).to_csv(os.path.join(out.dir, "05_settings_table_editable.csv"), index=False, encoding="utf-8-sig")
    fig.canvas.draw()
    bb = tb.get_window_extent(fig.canvas.get_renderer())
    inv = ax.transAxes.inverted()
    y_top, y_bot = inv.transform((0, bb.y1))[1], inv.transform((0, bb.y0))[1]
    ax.text(0.5, y_top + 0.03, f"Settings tested: step from rest to {tgt:g} RPM, mean of repeats", ha="center", va="bottom", fontsize=12.5, transform=ax.transAxes)
    ax.text(0.5, y_bot - 0.03, "PWM jitter = RMS sample-to-sample change of the PWM command over the last 40 % of the run.  Blue row = baseline.  "
            "Row colour = startup overshoot: green \u2264 5 %, amber 5\u201315 %, red > 15 %.", ha="center", va="top", fontsize=9, color="#444444", transform=ax.transAxes)
    out.single_fig("05_settings_table", fig, "Settings tested and results (mean of repeats).")
    out.comp(fig, "05_settings_table", f"Settings tested and results (mean of repeats), Ki values present: {sorted({round(c.ki, 1) for c in cfgs.values() if c.known})}.")


# ============================================================================================ open-loop: draw functions
def d_ol_trace(ax, r, col, half, win):
    mask, _ = trace_window(r, win)
    mean = float(np.mean(steady_arr(r, r.rpm)))
    ax.plot(r.t[mask], r.rpm[mask], color=col, lw=None)
    ax.axhline(mean, color=col, ls=":", lw=1, alpha=0.7)
    ax.set_ylim(mean - half, mean + half)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (RPM)")
    ax.text(0.995, 0.05, f"one revolution = {60 / mean:.2f} s", transform=ax.transAxes, ha="right")


def d_ol_dev(ax, firsts, cmap, win):
    for i, r in enumerate(firsts):
        mask, _ = trace_window(r, win)
        x = steady_arr(r, r.rpm)
        mean, sd = float(np.mean(x)), float(np.std(x))
        ax.plot(r.t[mask], r.rpm[mask] - mean, color=cmap(0.05 + 0.85 * i / max(1, len(firsts) - 1)), lw=None,
                label=f"{mean:.0f} RPM: std {sd:.2f} ({sd / mean * 100:.1f} % of speed)")
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed minus its mean (RPM)")
    ax.legend(loc="upper right")


def d_scale_amp(ax, M, slide=False):
    ax.set_xscale("log")
    ax.set_yscale("log")
    f2, f1 = loglog_fit(M.rpm, M.amp_2x), loglog_fit(M.rpm, M.amp_1x)
    l2 = "2\u00d7 component" + (mn(f" (fitted slope {f2[0]:+.2f} \u00b1 {f2[1]:.2f}, R\u00b2 {f2[2]:.2f})") if f2 else "")
    l1 = "1\u00d7 component" + (mn(f" (slope {f1[0]:+.2f} \u00b1 {f1[1]:.2f}, R\u00b2 {f1[2]:.2f})") if f1 else "")
    ms = 10 if slide else 8
    ax.plot(M.rpm, M.amp_2x, "o", color="#2471A3", ms=ms, label=l2)
    ax.plot(M.rpm, M.amp_1x, "s", mfc="none", mec="#C0392B", ms=ms, mew=1.8, label=l1)
    m12 = M.dropna(subset=["amp_12x"])
    if len(m12):
        ax.plot(m12.rpm, m12.amp_12x, "^", color="#E08A00", ms=ms + 1, label="12\u00d7 component (below ~250 RPM only)")
    ok = M.amp_2x.notna() & (M.amp_2x > 0)
    xr = np.array([M.rpm.min() * 0.8, M.rpm.max() * 1.25])
    gx, gy = np.exp(np.mean(np.log(M.rpm[ok]))), np.exp(np.mean(np.log(M.amp_2x[ok])))
    ax.plot(xr, gy * (xr / gx) ** -1, color="gray", ls="--", lw=1.4, label="Reference slope \u22121 (constant torque ripple on rotor inertia)")
    ax.set_xlim(*xr)
    allamp = pd.concat([M.amp_1x, M.amp_2x, M.amp_12x]).dropna()
    ax.set_ylim(allamp.min() * 0.7, allamp.max() * (4.5 if slide else 1.5))
    log_axis(ax, "x", xr[0], xr[1], [50, 100, 150, 200, 300, 500, 800, 1000, 1500, 2000])
    log_axis(ax, "y", ax.get_ylim()[0], ax.get_ylim()[1], [0.1, 0.2, 0.5, 1, 2, 5, 10, 20])
    ax.set_xlabel("Mean shaft speed (RPM)")
    ax.set_ylabel("Speed-ripple amplitude (RPM, zero-to-peak)")
    if slide:
        ax.legend(loc="upper right", fontsize=11)
    else:
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), fontsize=8.5, frameon=False)


def d_scale_pct(ax, M):
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.plot(M.rpm, M.std_pct, "o", color="#222222", ms=8)
    gm = M.groupby("pwm", as_index=False).mean(numeric_only=True).sort_values("rpm")
    ax.plot(gm.rpm, gm.std_pct, "-", color="#222222", lw=1, alpha=0.5)
    for _, rw in gm.iterrows():
        ax.annotate(f"{rw.std_pct:.2f} %" if rw.std_pct < 1 else f"{rw.std_pct:.1f} %", (rw.rpm, rw.std_pct), textcoords="offset points", xytext=(7, 7))
    xr = (M.rpm.min() * 0.8, M.rpm.max() * 1.25)
    ax.set_xlim(*xr)
    ax.set_ylim(M.std_pct.min() * 0.6, M.std_pct.max() * 1.8)
    log_axis(ax, "x", xr[0], xr[1], [50, 100, 150, 200, 300, 500, 800, 1000, 1500, 2000])
    log_axis(ax, "y", ax.get_ylim()[0], ax.get_ylim()[1], [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20])
    ax.set_xlabel("Mean shaft speed (RPM)")
    ax.set_ylabel("Speed ripple, std as % of mean speed")


def d_ol_orders(ax, firsts, cmap, xmax):
    for i, r in enumerate(firsts):
        s = order_spectrum(r, r.rpm, xmax=xmax)
        if s is not None:
            ax.plot(s[0], s[1], color=cmap(0.05 + 0.85 * i / max(1, len(firsts) - 1)), lw=None, label=f"{s[2] * 60:.0f} RPM")
    ax.set_xlim(0, xmax)
    ax.set_xticks(list(range(0, int(xmax) + 1, 1 if xmax < 6 else 2)))
    ax.set_xlabel("Frequency in multiples of shaft rotation frequency")
    ax.set_ylabel("Speed-ripple amplitude (RPM)")
    ax.legend(title="Mean speed", ncol=2, loc="upper right")


def d_rep_spec(ax, group):
    for i, r in enumerate(sorted(group, key=lambda x: x.stem)):
        s = order_spectrum(r, r.rpm)
        if s is None:
            continue
        x = steady_arr(r, r.rpm)
        ax.plot(s[0], s[1], color=PALETTE[i % len(PALETTE)], lw=None, ls="-" if i % 2 == 0 else "--", label=f"{r.stem}: {np.mean(x):.1f} RPM, std {np.std(x):.2f}")
    ax.set_xlim(0, 4.5)
    ax.set_xticks([0, 1, 2, 3, 4])
    ax.set_xticklabels(["0", "1\u00d7", "2\u00d7", "3\u00d7", "4\u00d7"])
    ax.set_xlabel("Frequency in multiples of shaft rotation frequency")
    ax.set_ylabel("Speed-ripple amplitude (RPM)")
    ax.legend(loc="upper right")


def d_rep_trace(ax, group):
    for i, r in enumerate(sorted(group, key=lambda x: x.stem)):
        mask, _ = trace_window(r, (10, 11.5))
        ax.plot(r.t[mask], r.rpm[mask], color=PALETTE[i % len(PALETTE)], lw=None, ls="-" if i % 2 == 0 else "--", label=r.stem)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw RPM")
    ax.legend(loc="upper right")


def ol_section(runs, args, out):
    global OL_RUNS
    OL_RUNS = runs
    groups = {}
    for r in runs:
        groups.setdefault(r.pwm_cmd, []).append(r)

    def spd_of(k):
        x = steady_arr(groups[k][0], groups[k][0].rpm)
        return float(np.mean(x)) if x is not None else 0.0
    order = sorted(groups, key=spd_of)
    print(f"\n[Open-loop] {len(runs)} runs at {len(groups)} PWM levels: {order}")
    rec = []
    for r in runs:
        x = steady_arr(r, r.rpm)
        if x is None:
            print(f"  skip {r.stem}: shorter than ~10 s, no spectrum")
            continue
        fr, a = spectrum(x)
        sh = float(np.mean(x)) / 60.0
        d = dict(file=r.stem, pwm=r.pwm_cmd, pwm_pct=r.pwm_cmd * 100 / 4095, rpm=float(np.mean(x)), rpm_std=float(np.std(x)),
                 std_pct=float(np.std(x) / np.mean(x) * 100), p2p=float(np.ptp(x)), shaft_hz=sh)
        for h in (1, 2, 3, 4, 12):
            d[f"amp_{h}x"] = order_peak(fr, a, h * sh)
        if r.torque_nm is not None:
            xt = steady_arr(r, r.torque_nm)
            if xt is not None and np.isfinite(xt).all():
                ft, at = spectrum(xt)
                d.update(torque_mean=float(np.mean(xt)), torque_std=float(np.std(xt)), torque_amp_1x=order_peak(ft, at, sh), torque_amp_2x=order_peak(ft, at, 2 * sh))
        rec.append(d)
    if not rec:
        return
    M = pd.DataFrame(rec).sort_values("rpm")
    M.to_csv(os.path.join(out.dir, "metrics_openloop.csv"), index=False)
    cmap = plt.get_cmap("viridis")
    firsts = [groups[k][0] for k in order if steady_arr(groups[k][0], groups[k][0].rpm) is not None]
    spans = [np.ptp(r.rpm[trace_window(r, args.window)[0]]) for r in firsts]
    half = max(20.0, float(np.ceil(max(spans) / 2 * 1.3 / 5) * 5))

    # ---- stacked traces + one graph per speed
    def ol_title(r):
        x = steady_arr(r, r.rpm)
        mean, sd = float(np.mean(x)), float(np.std(x))
        return f"{mean:.1f} RPM, PWM fixed at {r.pwm_cmd * 100 / 4095:.1f} % (jitter 0.000 %)  \u2192  RPM std {sd:.2f} ({sd / mean * 100:.2f} % of speed)"
    fig, axs = plt.subplots(len(firsts), 1, figsize=(10, 2.0 * len(firsts) + 1.0), sharex=True, squeeze=False)
    for i, (ax, r) in enumerate(zip(axs[:, 0], firsts)):
        col = cmap(0.05 + 0.85 * i / max(1, len(firsts) - 1))
        d_ol_trace(ax, r, col, half, args.window)
        ax.set_xlabel("")
        ax.set_title(ol_title(r), loc="left", fontsize=10.5, color=col, fontweight="bold")
    axs[-1, 0].set_xlabel("Time (s)")
    fig.suptitle(f"Open-loop runs, constant PWM, no controller (every panel spans the same {2 * half:g} RPM)", y=1.0, fontsize=12)
    fig.tight_layout()
    out.comp(fig, "11_openloop_traces",
             f"Open-loop speed traces at {len(firsts)} constant-PWM settings ({M.rpm.min():.0f}\u2013{M.rpm.max():.0f} RPM). The PWM command is constant (zero jitter) yet the speed ripples: "
             f"std {M.std_pct.iloc[0]:.2f} % of speed at {M.rpm.iloc[0]:.0f} RPM and {M.std_pct.iloc[-1]:.2f} % at {M.rpm.iloc[-1]:.0f} RPM. Same vertical span in all panels.")
    for i, r in enumerate(firsts):
        col = cmap(0.05 + 0.85 * i / max(1, len(firsts) - 1))
        spd = float(np.mean(steady_arr(r, r.rpm)))
        out.single(f"11_openloop_trace_{spd:04.0f}rpm", lambda ax, r=r, col=col: d_ol_trace(ax, r, col, half, args.window), title=ol_title(r),
                   caption=f"Open-loop speed at {spd:.0f} RPM with constant PWM; every graph in this series spans the same {2 * half:g} RPM.")
    out.single("11_openloop_overlay_deviation", lambda ax: d_ol_dev(ax, firsts, cmap, args.window),
               title="Open-loop speed ripple, mean removed (constant PWM, no controller)",
               caption="All open-loop speeds with each mean removed so the ripple sizes can be compared directly.")

    # ---- scaling
    if len(M) >= 3:
        f2, f1 = loglog_fit(M.rpm, M.amp_2x), loglog_fit(M.rpm, M.amp_1x)
        fig, (a, b) = plt.subplots(1, 2, figsize=(11.8, 5.4))
        d_scale_amp(a, M)
        d_scale_pct(b, M)
        a.set_title("(a) Ripple amplitude by shaft order vs speed")
        b.set_title("(b) Total speed ripple relative to speed")
        fig.tight_layout()
        cap = (f"Open-loop speed ripple versus speed ({len(M)} runs, {M.rpm.min():.0f}\u2013{M.rpm.max():.0f} RPM). (a) Amplitude of the 1\u00d7, 2\u00d7 and 12\u00d7 shaft-order components; "
               f"(b) total ripple as a percentage of speed.")
        if f2:
            cap += mn(f" The 2\u00d7 component follows a log\u2013log slope of {f2[0]:.2f} \u00b1 {f2[1]:.2f} (R\u00b2 = {f2[2]:.2f}); a speed-independent torque ripple on a fixed inertia predicts \u22121, an encoder geometry error +1.")
        if f1:
            cap += mn(f" The 1\u00d7 component has slope {f1[0]:.2f} \u00b1 {f1[1]:.2f} (R\u00b2 = {f1[2]:.2f})") + (" and shows no clear trend with speed." if f1[2] < 0.5 else ".")
        out.comp(fig, "12_openloop_scaling", cap)
        out.single("12a_scaling_amplitude", lambda ax: d_scale_amp(ax, M, slide=True), title="Ripple amplitude by shaft order vs speed", caption=cap)
        out.single("12b_ripple_percent", lambda ax: d_scale_pct(ax, M), title="Speed ripple as a percentage of speed",
                   caption=f"Total open-loop speed ripple relative to speed: {M.std_pct.iloc[0]:.1f} % at {M.rpm.iloc[0]:.0f} RPM down to {M.std_pct.iloc[-1]:.2f} % at {M.rpm.iloc[-1]:.0f} RPM.")

    # ---- orders
    xmax = 13.0 if M.rpm.min() < 260 else 4.5
    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    d_ol_orders(ax, firsts, cmap, xmax)
    ax.set_title("Open-loop speed spectrum by shaft order")
    fig.tight_layout()
    ocap = "Amplitude spectra of the open-loop speed signal with frequency normalised to the shaft rotation frequency. Peaks at integer orders show the ripple is tied to rotation. Components above 50 Hz (100 Hz sampling) alias into the band."
    out.comp(fig, "13_openloop_orders", ocap)
    out.single("13_openloop_orders", lambda ax: d_ol_orders(ax, firsts, cmap, xmax), title="Open-loop speed spectrum by shaft order", caption=ocap)

    # ---- repeatability
    rep = [k for k in order if len(groups[k]) >= 2]
    if rep:
        fig, axs = plt.subplots(len(rep), 2, figsize=(13, 4.0 * len(rep)), squeeze=False)
        for row, k in zip(axs, rep):
            d_rep_spec(row[0], groups[k])
            d_rep_trace(row[1], groups[k])
            row[0].set_title(f"(a) Repeats at PWM {k}: spectrum")
            row[1].set_title("(b) Raw speed, 1.5 s window")
        fig.tight_layout()
        sub = M[M.pwm == rep[0]]
        rcap = (f"Repeatability at identical PWM settings. At PWM {rep[0]}: mean speed {sub.rpm.min():.1f}\u2013{sub.rpm.max():.1f} RPM, "
                f"1\u00d7 amplitude {sub.amp_1x.min():.2f}\u2013{sub.amp_1x.max():.2f} RPM, 2\u00d7 amplitude {sub.amp_2x.min():.2f}\u2013{sub.amp_2x.max():.2f} RPM.")
        out.comp(fig, "14_openloop_repeatability", rcap)
        for k in rep:
            out.single(f"14a_repeat_spectrum_pwm{k}", lambda ax, k=k: d_rep_spec(ax, groups[k]), title=f"Repeats at PWM {k}: speed spectrum", caption=rcap)
            out.single(f"14b_repeat_trace_pwm{k}", lambda ax, k=k: d_rep_trace(ax, groups[k]), title=f"Repeats at PWM {k}: raw speed", caption=rcap)

    # ---- torque
    if "torque_mean" in M.columns and M.torque_mean.notna().any():
        T = M.dropna(subset=["torque_mean"])

        def d_tq_mean(ax):
            ax.errorbar(T.rpm, T.torque_mean, yerr=T.torque_std, fmt="o-", color="#2471A3", capsize=3)
            ax.set_xlabel("Mean shaft speed (RPM)")
            ax.set_ylabel("Torque (N\u00b7m), mean \u00b1 std")

        def d_tq_rip(ax):
            ax.plot(T.rpm, T.torque_amp_1x, "s", mfc="none", mec="#C0392B", ms=9, mew=1.8, label="1\u00d7")
            ax.plot(T.rpm, T.torque_amp_2x, "o", color="#2471A3", ms=9, label="2\u00d7")
            ax.set_xlabel("Mean shaft speed (RPM)")
            ax.set_ylabel("Torque ripple amplitude (N\u00b7m, zero-to-peak)")
            ax.legend()
        fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.6))
        d_tq_mean(ax[0])
        d_tq_rip(ax[1])
        ax[0].set_title("(a) Open-loop torque vs speed")
        ax[1].set_title("(b) Torque ripple by shaft order")
        fig.tight_layout()
        out.comp(fig, "15_openloop_torque", "Measured torque versus speed (open loop) and the shaft-order components of the torque ripple.")
        out.single("15a_torque_vs_speed", d_tq_mean, title="Open-loop torque vs speed", caption="Measured open-loop torque versus speed.")
        out.single("15b_torque_ripple", d_tq_rip, title="Torque ripple by shaft order", caption="Shaft-order components of the measured torque ripple.")


# ============================================================================================ dashboard exports
def detect_events(t, rpm, target, skip=3.0, thr=0.03, min_dur=0.3):
    dt = float(np.median(np.diff(t)))
    a = 1 - np.exp(-dt / 0.1)
    sm = np.empty_like(rpm)
    sm[0] = rpm[0]
    for i in range(1, len(rpm)):
        sm[i] = sm[i - 1] + a * (rpm[i] - sm[i - 1])
    bad = (np.abs(sm - target) > thr * target) & (t >= skip)
    ev, i = [], 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j < len(bad) and bad[j]:
                j += 1
            if t[j - 1] - t[i] >= min_dur:
                dev = rpm[i:j] - target
                k = i + int(np.argmax(np.abs(dev)))
                ev.append(dict(t_start=float(t[i]), t_peak=float(t[k]), depth_rpm=float(rpm[k] - target),
                               depth_pct=float((rpm[k] - target) / target * 100), recovery_s=float(t[j - 1] - t[i])))
            i = j
        else:
            i += 1
    return ev, sm


def dash_section(runs, args, out):
    print(f"\n[Dashboard exports] {len(runs)} files")
    allev, evruns = [], []
    for r in runs:
        tag = re.sub(r"^mixr1_log_\d+_\d+_?", "", r.stem) or r.stem
        safe = re.sub(r"[^A-Za-z0-9_-]+", "_", tag)
        has_pwm, has_t = bool(np.isfinite(r.pwm).any()), r.torque_nm is not None
        ev = []
        if np.isfinite(r.target) and r.tgt_series is not None and (r.tgt_series > 0).mean() > 0.5:
            ev, _ = detect_events(r.t, r.rpm, r.target, thr=args.event_thr / 100)
            allev += [dict(file=tag, **e) for e in ev]
            if ev:
                evruns.append((tag, r, ev))

        def d_speed(ax, r=r, ev=ev):
            ax.plot(r.t, r.rpm, color="#2471A3", lw=0.9, alpha=0.6, label="raw RPM")
            if r.filt is not None:
                ax.plot(r.t, r.filt, color="#1B4F72", lw=None, label="filtered RPM")
            if np.isfinite(r.target) and r.tgt_series is not None and (r.tgt_series > 0).mean() > 0.5:
                ax.axhline(r.target, color="k", ls=":", lw=1, label=f"target {r.target:g} RPM")
            for e in ev:
                ax.axvspan(e["t_start"], e["t_start"] + e["recovery_s"], color="#E08A00", alpha=0.18)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Speed (RPM)")
            ax.legend(loc="lower right")

        def d_pwm(ax, r=r):
            ax.plot(r.t, r.pwm, color="#1E8449", lw=None)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("PWM duty (%)")

        def d_tq(ax, r=r):
            ax.plot(r.t, r.torque_nm, color="#C0392B", lw=None)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Torque (N\u00b7m)")
        n = 1 + int(has_pwm) + int(has_t)
        fig, axs = plt.subplots(n, 1, figsize=(10, 2.8 * n + 0.8), sharex=True, squeeze=False)
        d_speed(axs[0, 0])
        k = 1
        if has_pwm:
            d_pwm(axs[k, 0])
            k += 1
        if has_t:
            d_tq(axs[k, 0])
        for a in axs[:-1, 0]:
            a.set_xlabel("")
        axs[0, 0].set_title(tag + (f"   ({len(ev)} disturbance event(s) shaded)" if ev else ""), loc="left", fontsize=10.5)
        fig.tight_layout()
        cap = (f"Dashboard recording '{tag}': speed" + (", PWM" if has_pwm else "") + (", torque" if has_t else "") + f" versus time over {r.t[-1]:.0f} s."
               + (f" Shaded: speed outside \u00b1{args.event_thr:g} % of target ({len(ev)} event(s))." if ev else ""))
        out.comp(fig, f"21_dashboard_{safe}", cap)
        out.single(f"21_{safe}_speed", d_speed, title=f"{tag}: speed" + (f" ({len(ev)} disturbance event(s) shaded)" if ev else ""), caption=cap)
        if has_pwm:
            out.single(f"21_{safe}_pwm", d_pwm, title=f"{tag}: PWM command", caption=cap)
        if has_t:
            out.single(f"21_{safe}_torque", d_tq, title=f"{tag}: torque", caption=cap)
    if allev:
        pd.DataFrame(allev).to_csv(os.path.join(out.dir, "metrics_disturbance_events.csv"), index=False)

        def d_dist(ax):
            for i, (tag, r, ev) in enumerate(evruns):
                e = ev[0]
                m = (r.t >= e["t_start"] - 2) & (r.t <= e["t_start"] + 8)
                ax.plot(r.t[m] - e["t_start"], r.rpm[m] - r.target, color=PALETTE[i % len(PALETTE)], lw=None,
                        label=mn(f"{tag}: depth {e['depth_rpm']:.1f} RPM, back in band after {e['recovery_s']:.1f} s"))
            ax.axhline(0, color="k", lw=0.8)
            ax.axvline(0, color="gray", ls=":", lw=1)
            ax.axhspan(-args.event_thr / 100 * evruns[0][1].target, args.event_thr / 100 * evruns[0][1].target, color="k", alpha=0.06)
            ax.set_xlabel("Time since the disturbance was detected (s)")
            ax.set_ylabel("Speed minus target (RPM)")
            ax.legend(loc="lower right")
        fig, ax = plt.subplots(figsize=(9.5, 4.8))
        d_dist(ax)
        ax.set_title("Response to a load disturbance (first event of each recording; grey band = detection threshold)")
        fig.tight_layout()
        dcap = (f"Speed deviation from target after the first detected load disturbance in each recording, aligned at the detection time. Depth = largest raw deviation during the event; "
                f"recovery = time the speed (smoothed, 0.1 s time constant) stays outside \u00b1{args.event_thr:g} % of target.")
        out.comp(fig, "22_disturbance_overlay", dcap)
        out.single("22_disturbance_overlay", d_dist, title="Response to a load disturbance", caption=dcap)


# ============================================================================================ main
def main():
    ap = argparse.ArgumentParser(description="Automatic thesis-quality and PowerPoint-ready figures for MIXR-1 test data")
    ap.add_argument("folder")
    ap.add_argument("--out", default=None, help="output folder (default: <folder>/figures)")
    ap.add_argument("--raw", default=None, help="results_raw.txt (default: <folder>/results_raw.txt if present)")
    ap.add_argument("--show", default=None, help="comma-separated config IDs or names for the time-series graphs")
    ap.add_argument("--n-show", type=int, default=3, help="auto-pick this many configs besides the baseline")
    ap.add_argument("--max-overshoot", type=float, default=15.0, help="auto-pick only configs below this overshoot %%")
    ap.add_argument("--window", type=float, nargs=2, default=(10.0, 13.0), help="time window for trace graphs (s)")
    ap.add_argument("--event-thr", type=float, default=3.0, help="disturbance detection threshold, %% of target")
    ap.add_argument("--slide-size", type=float, nargs=2, default=(10.0, 5.625), help="individual graph size in inches (default 16:9)")
    ap.add_argument("--no-title", action="store_true", help="individual graphs without titles")
    ap.add_argument("--other-alpha", type=float, default=0.3,
                    help="opacity (0-1) of non-baseline lines in overlay graphs so the baseline stands out; 1.0 = no fading (default 0.3)")
    ap.add_argument("--no-individual", action="store_true", help="only the multi-panel thesis figures")
    ap.add_argument("--list", action="store_true", help="only list how each file was classified")
    ap.add_argument("-r", "--recursive", action="store_true")
    args = ap.parse_args()
    set_style()
    EMPH["other_alpha"] = min(1.0, max(0.05, args.other_alpha))

    pattern = os.path.join(args.folder, "**", "*.csv") if args.recursive else os.path.join(args.folder, "*.csv")
    files = sorted(glob.glob(pattern, recursive=args.recursive))
    outdir = args.out or os.path.join(args.folder, "figures")
    raw_path = args.raw or os.path.join(args.folder, "results_raw.txt")
    raw_rows = read_raw(raw_path) if os.path.exists(raw_path) else []
    if not files and not raw_rows:
        sys.exit(f"No CSV files (and no results_raw.txt) found in {args.folder}")

    print(f"Reading {len(files)} CSV files from {args.folder}")
    runs = [r for r in (load_run(f) for f in files) if r is not None]
    pi = [r for r in runs if r.kind == "PI"]
    ol = [r for r in runs if r.kind == "OL"]
    dash = [r for r in runs if r.kind == "DASH"]
    print(f"\nClassification: {len(pi)} PI step runs, {len(ol)} open-loop runs, {len(dash)} dashboard exports"
          + (f", plus {len(raw_rows)} result lines from {os.path.basename(raw_path)}" if raw_rows else ""))
    for r in runs:
        print(f"  {r.kind:4s} {os.path.basename(r.path):45s} {r.t[-1]:6.1f} s" + (f"  target {r.target:g} RPM" if r.kind == 'PI' else (f"  PWM {r.pwm_cmd}" if r.kind == 'OL' else '')))
    if args.list:
        return
    out = Out(outdir, args)
    global OL_RUNS
    OL_RUNS = ol
    if pi or raw_rows:
        pi_section(pi, raw_rows, args, out)
    if ol:
        ol_section(ol, args, out)
    if dash:
        dash_section(dash, args, out)
    if not out.caps:
        print("\nNothing to plot: no PI runs, open-loop runs, dashboard exports or results_raw.txt found.")
        return
    with open(os.path.join(outdir, "captions.md"), "w", encoding="utf-8") as f:
        f.write("# Auto-generated figure captions\n\nNumbers come straight from the data; edit the wording, not the numbers.\n\n")
        for name, cap in out.caps:
            f.write(f"**{name}** - {cap}\n\n")
    print(f"\nDone. {len([c for c in out.caps if not c[0].startswith('individual/')])} composite figures in {outdir}"
          + (f" and {out.n_ind} individual graphs in {out.ind}" if not args.no_individual else "") + "  (captions.md + metrics_*.csv alongside)")


if __name__ == "__main__":
    main()
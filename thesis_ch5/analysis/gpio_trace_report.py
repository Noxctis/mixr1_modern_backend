#!/usr/bin/env python3
"""
gpio_trace_report.py - integrity and latency of the GPIO trace written by the daemon (--trace-gpio).

    python gpio_trace_report.py mode1_logs/gpio_trace_YYYYmmdd_HHMMSS.csv [--cpr 1024] [--out report.txt]

Pins as in config.hpp: A = 24, B = 23, X = 22 (encoder). Integrity: edges of A and B equal in number and CPR/2 per revolution each, one rising
X edge per revolution, time stamps monotonic, no gap longer than 0.5 s inside the active period. Latency: host time minus pigpio tick time,
after removing the constant offset of the two clocks (the 1st percentile of the difference over each 1 s window); reported per pin in us.
"""
import argparse, sys
import numpy as np, pandas as pd


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--cpr", type=float, default=1024.0); ap.add_argument("--out"); a = ap.parse_args()
    d = pd.read_csv(a.csv); out = []
    tick = d.tick_us.to_numpy(dtype=np.int64); wraps = np.cumsum(np.r_[0, (np.diff(tick) < -(2 ** 31)).astype(np.int64)]); t = (tick + wraps * 2 ** 32) / 1e6
    d["t"] = t; back = int((np.diff(t) < -1e-9).sum())
    out.append(f"events {len(d)}, span {t.max() - t.min():.2f} s, 32-bit tick wraps {int(wraps.max())}, time stamps going backwards {back}")
    for g, name in ((24, "A"), (23, "B"), (22, "X"), (20, "knob A"), (21, "knob B"), (26, "knob push"), (4, "start key"), (18, "driver EN"), (27, "INA"), (17, "INB")):
        s = d[d.gpio == g]
        if len(s): out.append(f"GPIO {g:2d} {name:9s}: {len(s):8d} edges, {int((s.level == 1).sum())} rising, first {s.t.iloc[0] - t.min():.2f} s, last {s.t.iloc[-1] - t.min():.2f} s")
    A, B, X = (d[d.gpio == g] for g in (24, 23, 22)); xr = int(((X.level == 1)).sum())
    if len(A) and len(B):
        out.append(f"A edges {len(A)}, B edges {len(B)}, difference {len(A) - len(B)} ({100 * (len(A) / len(B) - 1):+.3f} %)")
        if xr > 1:
            per = len(A) / xr; out.append(f"A edges per index pulse {per:.1f} (expected {a.cpr / 2:.0f}); revolutions by X {xr}, by A edges {len(A) / (a.cpr / 2):.1f} ({100 * (xr / (len(A) / (a.cpr / 2)) - 1):+.2f} %)")
        gaps = np.diff(A.t.to_numpy()); big = gaps[gaps > 0.5]
        out.append(f"gaps of A longer than 0.5 s: {len(big)} (longest {gaps.max():.2f} s)")
    # latency: host_s - tick time, minus its 1st percentile in every 1 s window
    d["diff"] = d.host_s - d.t; d["win"] = np.floor(d.host_s).astype(int); off = d.groupby("win")["diff"].transform(lambda v: np.percentile(v, 1)); d["lat_us"] = 1e6 * (d["diff"] - off)
    for g, name in ((24, "A"), (23, "B"), (22, "X")):
        s = d[d.gpio == g].lat_us
        if len(s): out.append(f"latency GPIO {g} {name}: median {s.median():.0f} us, p99 {np.percentile(s, 99):.0f} us, max {s.max():.0f} us, jitter (SD) {s.std():.0f} us")
    txt = "\n".join(out); print(txt)
    if a.out: open(a.out, "w", encoding="utf-8").write(txt + "\n")


if __name__ == "__main__":
    main()

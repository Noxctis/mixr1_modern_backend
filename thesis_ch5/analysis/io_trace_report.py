#!/usr/bin/env python3
"""
io_trace_report.py - integrity and latency of the motor-driver writes and the torque-amplifier reads (daemon option --trace-io).

    python io_trace_report.py gpio_traces/io_trace_YYYYmmdd_HHMMSS.csv [--torque-period-ms 20] [--out report.txt]

motor_pwm   : every hardware_PWM() call (round trip to pigpiod): calls, failures, duration (median, p99, max), longest interval between calls.
torque_read : every Modbus read: calls, failures by kind (timeout, header, CRC, port), duration of request + reply, interval between reads
              against the period, longest time without a good reading (how stale the torque value of the loop can get).
torque_zero : every zero command and whether it was acknowledged.
"""
import argparse
import numpy as np, pandas as pd

ERR = {0: "none", 1: "no/short reply", 2: "wrong header", 3: "CRC error", 4: "write failed / port", 5: "cannot decode"}


def pct(v, q): return float(np.percentile(v, q)) if len(v) else float("nan")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--torque-period-ms", type=float, default=20.0); ap.add_argument("--out"); a = ap.parse_args()
    d = pd.read_csv(a.csv); out = [f"calls {len(d)}, span {d.start_s.max() - d.start_s.min():.1f} s"]
    for ch in ("motor_pwm", "torque_read", "torque_zero"):
        s = d[d.channel == ch].sort_values("start_s")
        if s.empty: continue
        bad = s[s.ok == 0]; dt = np.diff(s.start_s.to_numpy()) * 1e3
        out.append(f"{ch}: {len(s)} calls, {len(bad)} failed ({100 * len(bad) / len(s):.2f} %), duration median {s.dur_us.median():.0f} us, p99 {pct(s.dur_us, 99):.0f} us, max {s.dur_us.max():.0f} us")
        if len(dt): out.append(f"   interval between calls: median {np.median(dt):.1f} ms, p99 {pct(dt, 99):.1f} ms, longest {dt.max():.1f} ms")
        if len(bad) and ch == "motor_pwm": out.append(f"   {len(bad)} calls: the driver library returned an error")
        elif len(bad): out.append("   failures: " + ", ".join(f"{ERR.get(int(k), k)} {int(n)}" for k, n in bad.err.value_counts().items()))
        if ch == "torque_read":
            good = s[s.ok == 1].start_s.to_numpy(); gap = np.diff(good) * 1e3
            if len(gap):
                out.append(f"   longest time between good readings {gap.max():.0f} ms ({int((gap > 5 * a.torque_period_ms).sum())} gaps longer than 5 periods of {a.torque_period_ms:.0f} ms)")
            v = s[s.ok == 1].value
            out.append(f"   raw count: min {int(v.min())}, max {int(v.max())}, changes between reads {int((np.diff(v.to_numpy()) != 0).sum())}")
    txt = "\n".join(out); print(txt)
    if a.out: open(a.out, "w", encoding="utf-8").write(txt + "\n")


if __name__ == "__main__":
    main()

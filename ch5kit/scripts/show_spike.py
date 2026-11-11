#!/usr/bin/env python3
"""Shows the rows of one run around its biggest speed spike:  python3 show_spike.py logs/ch5_new/estimators/est-dct10_A_r1.csv
Columns: raw_rpm = the estimate, delta_count / delta_tick_us = the count and the microseconds that estimate used (DCT: accumulated over its window;
CET: between the two snapshots), loop_period_us = the loop. A dip just BEFORE the spike (count delivered late, then all at once) is the signature of
a count that does not follow the clock."""
import sys, pandas as pd, numpy as np
d = pd.read_csv(sys.argv[1]); x = d.raw_rpm.abs()
med = x.rolling(101, center=True, min_periods=50).median(); r = (x / med).where(med > 100)
i = int(r.idxmax()); lo, hi = max(0, i - 12), min(len(d), i + 6)
print(f"biggest spike: row {i}, t = {d.elapsed_s.iloc[i]:.3f} s, raw_rpm = {x.iloc[i]:.0f}, local median {med.iloc[i]:.0f} (ratio {r.iloc[i]:.2f})")
cols = [c for c in ("elapsed_s", "raw_rpm", "new_snap", "delta_count", "delta_tick_us", "loop_period_us", "late_us") if c in d]
print(d[cols].iloc[lo:hi].to_string())

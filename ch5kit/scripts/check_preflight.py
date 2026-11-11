#!/usr/bin/env python3
"""Reads the daemon's output of the 8-second preflight run and says PASS or FAIL.
It needs  edge_total, quad_errors  and  revs  from the line '[TEST] edge_total=... quad_errors=... revs=...'."""
import re, sys
t = open(sys.argv[1], errors="ignore").read()
m = re.search(r"edge_total=(\d+)\s+quad_errors=(\d+).*?revs=(-?\d+)", t)
if not m:
    print("FAIL: the daemon did not print its edge counters. Did the motor run? Last lines of its output:\n" + "\n".join(t.splitlines()[-8:])); sys.exit(1)
edges, qerr, revs = int(m.group(1)), int(m.group(2)), abs(int(m.group(3)))
if revs < 5:
    print(f"FAIL: only {revs} revolutions counted (need 5 or more). The motor did not turn, or the index wire is not connected."); sys.exit(1)
per_rev = edges / revs
print(f"edges per revolution = {per_rev:.1f} (must be about 1024), lost edges = {qerr}")
if not (990 <= per_rev <= 1060):
    print("FAIL: edges per revolution is not 1024. The DIP switch is not at 0011 (256 PPR), or the encoder was not switched off and on after you changed it."); sys.exit(1)
if qerr > 0:
    print("FAIL: edges were lost. Check the encoder wires and the level converter, then run again."); sys.exit(1)
print("PASS: encoder at 256 PPR, no lost edges.")

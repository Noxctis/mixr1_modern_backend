#!/usr/bin/env python3
"""Build the run order for the PPR study and write run_plan.csv.

Two designs:
  interleaved (default)  every block visits every setting once (ascending, descending, then shuffled orders) and ends with a repeat
                         of the reference setting (256 PPR) as a drift check. Run order is then NOT confounded with PPR.
                         Needs one DIP change per visit: about (settings + 1) x blocks changes.
  blocked                every setting is visited once and its --blocks repeats are run back to back; the reference is measured
                         first, last and after every --ref-every settings. Needs few DIP changes, but order and setting are
                         confounded: report the reference drift as the run-to-run scatter and say so in the thesis.

Test A = open-loop PWM staircase, 0..90 % in 10 % steps, 10 s each (speed noise, update fraction, scale / edge loss, CPU load).
Test B = closed-loop staircase 10..1500 rpm (10 setpoints), 30 s each (update floor, n_min, controller behaviour, tachometer check).
A visit at one DIP setting runs A, then B when the setting is in --b-settings.

Usage:  python3 plan_runs.py --out logs/ppr_20261112 [--settings 48,100,125,250,256,512,1024] [--blocks 3] [--seed 7]
"""
import argparse, csv, os, random, sys

DIP = {48: "1111", 96: "1101", 100: "0111", 125: "1011", 192: "1110", 200: "0101", 250: "1001", 256: "0011",
       384: "1100", 400: "0110", 500: "1010", 512: "0001", 800: "0100", 1000: "1000", 1024: "0010", 2048: "0000"}
A_MIN, B_MIN, PREP_MIN = 2.0, 5.5, 1.5      # minutes: Test A (110 s), Test B (240 s + tare), per-run settling / rest wait


def build_visits(a, settings):
    """List of (block, ppr, role, repeats_in_a_row)."""
    rng = random.Random(a.seed)
    visits = []
    if a.design == "interleaved":
        for b in range(1, a.blocks + 1):
            o = list(settings)
            if b == 2:
                o.reverse()
            elif b >= 3:
                rng.shuffle(o)
            visits += [(b, p, "measure", 1) for p in o]
            visits.append((b, a.ref, "drift", 1))
    else:
        others = [p for p in settings if p != a.ref]
        rng.shuffle(others)
        visits.append((1, a.ref, "drift", max(1, a.blocks - 1)))   # reference first: blocks-1 runs, plus 1 at the end = `blocks` repeats of the reference
        for i, p in enumerate(others, 1):
            visits.append((1, p, "measure", a.blocks))
            if i % a.ref_every == 0 and i != len(others):
                visits.append((1, a.ref, "drift", 1))
        visits.append((1, a.ref, "drift", 1))
    merged = []                                    # consecutive visits at one setting need no DIP change in between
    for blk, p, role, m in visits:
        if merged and merged[-1][1] == p:
            b0, _, r0, m0 = merged[-1]
            merged[-1] = (b0, p, "measure" if "measure" in (r0, role) else "drift", m0 + m)
        else:
            merged.append((blk, p, role, m))
    return merged


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="campaign directory (run_plan.csv is written into it)")
    ap.add_argument("--settings", default="48,100,125,250,256,512,1024")
    ap.add_argument("--ref", type=int, default=256, help="reference / drift setting")
    ap.add_argument("--blocks", type=int, default=3, help="interleaved: number of blocks; blocked: repeats per setting")
    ap.add_argument("--design", choices=["interleaved", "blocked"], default="interleaved")
    ap.add_argument("--b-settings", default="48,125,256,512", help="settings that also get the low-speed closed-loop test B")
    ap.add_argument("--ref-every", type=int, default=3, help="blocked design: reference visit after every N settings")
    ap.add_argument("--current-ppr", type=int, default=None, help="PPR the encoder is set to NOW (default: the reference): no DIP change for the first visit at it")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()

    settings = [int(x) for x in a.settings.split(",")]
    if a.ref not in settings:
        settings.append(a.ref)
    b_set = {int(x) for x in a.b_settings.split(",") if x}
    for p in settings:
        if p not in DIP:
            sys.exit(f"unknown PPR {p}: add its DIP code to DIP[] (from the datasheet)")

    rows, seq, reps, dips, t = [], 0, {}, 0, 0.0
    prev = a.current_ppr if a.current_ppr else a.ref
    for block, ppr, role, mult in build_visits(a, settings):
        change = int(ppr != prev)
        dips += change
        prev = ppr
        first = True
        for _ in range(mult):
            for test in ("A", "B"):
                if test == "B" and ppr not in b_set:
                    continue
                seq += 1
                reps[(ppr, test)] = reps.get((ppr, test), 0) + 1
                t += PREP_MIN + (A_MIN if test == "A" else B_MIN)
                rows.append(dict(seq=seq, block=block, ppr=ppr, dip=DIP[ppr], cpr=4 * ppr, test=test, repeat=reps[(ppr, test)],
                                 role=role, dip_change=change if first else 0, est_cum_min=round(t, 1)))
                first = False
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "run_plan.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    print(f"Wrote {path}: {len(rows)} runs, {dips} DIP changes, about {t/60:.1f} h of running time (+ DIP changes and warm-up).")
    print("\n seq blk  PPR  DIP   CPR test rep role     DIP change")
    for r in rows:
        print(f"{r['seq']:4d} {r['block']:3d} {r['ppr']:4d} {r['dip']} {r['cpr']:5d}  {r['test']}   r{r['repeat']}  {r['role']:8s} {'<-- set DIP' if r['dip_change'] else ''}")
    if dips > 3:
        print(f"\n!! {dips} DIP changes. The AMT102 datasheet limits mounting/removing the top cover to 3 cycles. If the DIP switch is under that cover,"
              "\n   no design with repeated visits can stay inside that limit. Options: (a) confirm the switch is reachable without the cover;"
              "\n   (b) do the study on a spare encoder, so the unit of the final rig is untouched; (c) the minimum design is"
              "\n   --design blocked --settings 48,256,512 --current-ppr 256 --ref-every 9: exactly 3 changes (256>48>512>256), no margin.")


if __name__ == "__main__":
    main()

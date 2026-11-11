#!/usr/bin/env python3
"""Run the PPR study on the Raspberry Pi, one planned run at a time (stdlib only, Python 3.8+).

  python3 ppr_campaign.py run     --dir logs/ppr_20261112 [--daemon "sudo ./build/mixr1_daemon"] [--from-seq N] [--only A|B]
  python3 ppr_campaign.py warmup  --dir logs/ppr_20261112 [--minutes 5]
  python3 ppr_campaign.py preflight --ppr 256 --dir logs/ppr_20261112      (one short check of the DIP / CPR setting)

What `run` does for every row of <dir>/run_plan.csv (made by plan_runs.py):
  1. at a DIP change: tells you which switch code to set, waits for Enter, then runs a PREFLIGHT (8 s open loop at ~15 % PWM) that checks
       counts between index pulses == 4 x PPR, index-derived speed == estimator speed, no invalid quadrature transitions.
     If it fails it tells you which PPR the encoder actually seems to be at, and does not continue until it passes.
  2. runs the daemon in --test mode with --cpr = 4 x PPR (so CPR can never disagree with the DIP code you were told to set),
       Test A  open loop, PWM 0..90 % step 10 %, 10 s per step                       (no extra socket traffic: CPU numbers stay clean)
       Test B  PI control, 10 25 50 100 250 500 750 1000 1250 1500 rpm, 30 s per step, --log-age
  3. records pidstat (per thread, 1 s, epoch time stamps) while the daemon runs, platform state before and after
     (temperature, throttling flags, governor, pigpiod arguments, git commit) and your supply voltage / motor temperature entries,
  4. in Test B prompts you three times per hold (at 10, 15, 20 s) for a handheld-tachometer reading: type the number + Enter.
  5. checks the run (rows, loop timing, edges, index pulses) and prints PASS / WARN lines.

Files written per run (base = mixr1_log_<date>_<time>_<PPR>ppr_cet_r<k>_test<A|B>):
  <base>.csv  <base>_index.csv  <base>.stdout.txt  <base>.pidstat.txt  <base>.meta.json  [<base>_tach.csv]
"""
import argparse, csv, datetime, glob, json, os, re, select, shlex, signal, statistics, subprocess, sys, threading, time

A_SCHEDULE = list(range(0, 100, 10))                       # PWM % (0 ... 90)
A_HOLD_S = 10.0
B_SCHEDULE = [10, 25, 50, 100, 250, 500, 750, 1000, 1250, 1500]       # rpm setpoints
B_HOLD_S = 30.0
TACH_AT_S = (10.0, 15.0, 20.0)                             # seconds into each Test-B hold at which you are prompted for a tachometer reading
PREFLIGHT_PWM_COUNTS = 614                                 # 15 % of 4095, about 350 rpm on this rig (no load)
TOKEN = {"aligned": "cet", "newest": "cetn"}
ABORT_RPM_A, ABORT_RPM_B = 3200, 2200                      # safety cut-offs passed to the daemon


# ------------------------------------------------------------------------------------------------------------------------------
def sh(cmd, timeout=10):
    try:
        out = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, text=True, timeout=timeout).stdout.strip()
        return out or "n/a"
    except Exception as exc:                              # tool missing (e.g. vcgencmd on a PC)
        return f"n/a ({exc.__class__.__name__})"


def platform_state(repo):
    gov = sorted({open(p).read().strip() for p in glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")}) or ["n/a"]
    freq = sh("cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    return dict(
        utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        temp=sh("vcgencmd measure_temp"), throttled=sh("vcgencmd get_throttled"), governor=gov, cpu0_khz=freq,
        pigpiod=sh("ps -C pigpiod -o args="), loadavg=sh("cat /proc/loadavg"), uname=sh("uname -a"),
        model=sh("tr -d '\\0' < /proc/device-tree/model"),
        git_commit=sh(f"git -C {shlex.quote(repo)} rev-parse HEAD") if repo else "n/a",
        git_dirty=bool(sh(f"git -C {shlex.quote(repo)} status --porcelain")) if repo else None)


def platform_warnings(st):
    w = []
    if "-s 5" not in st["pigpiod"] and "-s5" not in st["pigpiod"]:
        w.append(f"pigpiod is not running with '-s 5' (found: {st['pigpiod'] or 'not running'}). Run: sudo killall pigpiod; sudo pigpiod -s 5")
    if st["governor"] != ["performance"] and st["governor"] != ["n/a"]:
        w.append(f"CPU governor is {st['governor']}, not 'performance'. Run Tools/ppr/prepare_platform.sh")
    if "0x0" not in st["throttled"] and "n/a" not in st["throttled"]:
        w.append(f"throttling flags {st['throttled']} (0x0 = never throttled): check the power supply and cooling")
    return w


def ask(prompt, default=""):
    if not sys.stdin.isatty():
        return default
    try:
        v = input(prompt).strip()
    except EOFError:
        v = ""
    return v or default


# ------------------------------------------------------------------------------------------------------------------------------
def read_csv(path):
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        cols = {k: [] for k in rd.fieldnames}
        for row in rd:
            for k, v in row.items():
                try:
                    cols[k].append(float(v))
                except (TypeError, ValueError):
                    cols[k].append(float("nan"))
    return cols


def unwrap(ticks):
    out, add, prev = [], 0, None
    for t in ticks:
        if prev is not None and t < prev - 2 ** 31:
            add += 2 ** 32
        out.append(t + add)
        prev = t
    return out


def qc_window(cols, idx_ticks, idx_counts, t_lo, t_hi):
    """Estimator mean vs index-derived speed and counts per index pulse over [t_lo, t_hi) seconds of the log."""
    sel = [i for i, t in enumerate(cols["elapsed_s"]) if t_lo <= t < t_hi]
    if len(sel) < 20:
        return None
    est = statistics.fmean(cols["raw_rpm"][i] for i in sel)
    tick = unwrap([cols["snap_tick"][i] for i in range(len(cols["snap_tick"]))])
    k0, k1 = tick[sel[0]], tick[sel[-1]]
    pairs = [(t, c) for t, c in zip(unwrap(idx_ticks), idx_counts) if k0 <= t <= k1]
    if len(pairs) < 3 or est < 20:
        return dict(est=est, idx_rpm=float("nan"), counts_per_index=float("nan"), n_index=len(pairs))
    dts = [b[0] - a[0] for a, b in zip(pairs, pairs[1:])]
    dcs = [abs(b[1] - a[1]) for a, b in zip(pairs, pairs[1:])]
    return dict(est=est, idx_rpm=60e6 / statistics.median(dts), counts_per_index=statistics.median(dcs), n_index=len(pairs))


def run_qc(csv_path, test, cpr, quiet=False):
    """Returns (ok, messages). Reads <csv> and <csv minus .csv>_index.csv."""
    msgs, ok = [], True
    if not os.path.exists(csv_path):
        return False, ["no CSV was written"]
    cols = read_csv(csv_path)
    n = len(cols.get("elapsed_s", []))
    dur = A_HOLD_S * len(A_SCHEDULE) if test == "A" else B_HOLD_S * len(B_SCHEDULE) if test == "B" else 8.0
    if n < 0.95 * dur * 100:
        ok = False; msgs.append(f"WARN only {n} rows, expected about {int(dur * 100)} (run aborted?)")
    if "quad_err" not in cols:
        return False, msgs + ["WARN CSV has no PPR-study columns: the patched firmware is not the one that ran"]
    per = cols["loop_period_us"][1:]
    late = cols["late_us"]
    if per:
        msgs.append(f"loop period mean {statistics.fmean(per):.0f} us, SD {statistics.pstdev(per):.0f} us, max lateness {max(late):.0f} us")
        if statistics.fmean(per) < 9900 or statistics.fmean(per) > 10100:
            ok = False; msgs.append("WARN mean loop period is not 10 ms")
    qe = cols["quad_err"][-1] if cols["quad_err"] else 0
    if qe > 0:
        ok = False; msgs.append(f"WARN {int(qe)} invalid quadrature transitions: edges were LOST (speed reads low, alignment is broken)")
    base = csv_path[:-4]
    idx_path = base + "_index.csv"
    if os.path.exists(idx_path):
        ix = read_csv(idx_path)
        windows = [(2.0, 8.0)] if test == "P" else [(s * A_HOLD_S + 1.0, s * A_HOLD_S + 9.0) for s in range(1, len(A_SCHEDULE))] if test == "A" else \
                  [(s * B_HOLD_S + 5.0, s * B_HOLD_S + 25.0) for s in range(3, len(B_SCHEDULE))]
        worst_scale, worst_cpi = 0.0, 0.0
        for lo, hi in windows:
            q = qc_window(cols, ix["tick_us"], ix["count"], lo, hi)
            if q and q["n_index"] >= 3 and q["idx_rpm"] == q["idx_rpm"]:
                worst_scale = max(worst_scale, abs(q["idx_rpm"] / q["est"] - 1.0))
                worst_cpi = max(worst_cpi, abs(q["counts_per_index"] - cpr))
                if abs(q["counts_per_index"] - cpr) > 0.5:
                    implied = q["counts_per_index"] / 4.0
                    ok = False
                    msgs.append(f"WARN counts per index pulse = {q['counts_per_index']:.1f}, expected {cpr:.0f} (encoder looks like {implied:.1f} PPR; "
                                f"is the DIP switch at the code you were given, and was the encoder power-cycled?)")
                    break
        if worst_scale > 0.02:
            ok = False; msgs.append(f"WARN index-derived speed differs from the estimator by up to {100 * worst_scale:.1f} % (expected < 1 %)")
        elif windows:
            msgs.append(f"index check ok: max |index speed / estimator - 1| = {100 * worst_scale:.2f} %, max |counts per index - CPR| = {worst_cpi:.2f}")
    else:
        ok = False; msgs.append("WARN no index log")
    if cols["idx_rej"] and cols["idx_rej"][-1] > 0:
        msgs.append(f"note: {int(cols['idx_rej'][-1])} index pulses rejected as noise")
    if cols["idx_miss"] and cols["idx_miss"][-1] > 0:
        msgs.append(f"note: {int(cols['idx_miss'][-1])} index pulses missed")
    return ok, msgs


# ------------------------------------------------------------------------------------------------------------------------------
class Daemon:
    """One daemon --test run with stdout capture, optional pidstat and (Test B) tachometer prompts."""

    def __init__(self, args, base, ppr, test, role="measure"):
        self.args, self.base, self.ppr, self.test = args, base, ppr, test
        self.t0 = None
        self.lines = []
        self.cpr = 4 * ppr

    def command(self):
        a = self.args
        c = shlex.split(a.daemon) + ["--test", f"--cpr={self.cpr}", "--fifo", "--no-stream", "--wait-rest=20", f"--csv={self.base}.csv"]
        c += [] if a.torque else ["--no-torque"]
        c += ["--estimator=newest"] if a.estimator == "newest" else []
        if self.test == "A":
            c += ["--no-pi", "--schedule=" + ",".join(str(x) for x in A_SCHEDULE), f"--hold={A_HOLD_S:g}", f"--abort-rpm={ABORT_RPM_A}"]
        elif self.test == "B":
            c += ["--pi", "--schedule=" + ",".join(str(x) for x in B_SCHEDULE), f"--hold={B_HOLD_S:g}", f"--abort-rpm={ABORT_RPM_B}", "--log-age"]
        else:                                                                # preflight
            c += ["--no-pi", f"--pwm={PREFLIGHT_PWM_COUNTS}", "--duration=8", f"--abort-rpm={ABORT_RPM_A}"]
        return c

    def run(self, tach=False, pidstat=True):
        a = self.args
        cmd = self.command()
        print("   $ " + " ".join(shlex.quote(x) for x in cmd))
        ps = None
        if pidstat and not a.no_pidstat and self.test in ("A", "B"):
            ps_out = open(self.base + ".pidstat.txt", "w")
            ps = subprocess.Popen(shlex.split(a.pidstat) + ["-h", "-u", "-t", "-C", "pigpiod|mixr1_daemon", "1"], stdout=ps_out, stderr=subprocess.DEVNULL)
        out = open(self.base + ".stdout.txt", "w")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        stop = threading.Event()

        def pump():
            for line in proc.stdout:
                out.write(line); out.flush()
                self.lines.append(line.rstrip())
                m = re.search(r"t0_unix=([0-9.]+)", line)
                if m:
                    self.t0 = float(m.group(1))
                if a.verbose or any(k in line for k in ("[SAFETY]", "ABORT", "WARNING", "TEST] step", "index log", "edge_total", "CRITICAL", "Cannot")):
                    print("   | " + line.rstrip())
            stop.set()

        th = threading.Thread(target=pump, daemon=True); th.start()
        tach_rows, fired = [], set()
        try:
            while not stop.is_set():
                time.sleep(0.05)
                if tach and self.t0 and sys.stdin.isatty():
                    el = time.time() - self.t0
                    k, within = int(el // B_HOLD_S), el % B_HOLD_S
                    if 0 <= k < len(B_SCHEDULE):
                        for j, ts in enumerate(TACH_AT_S, 1):
                            if (k, j) not in fired and within >= ts:
                                fired.add((k, j))
                                print(f"\n   >>> READ TACH NOW ({j}/3)  setpoint {B_SCHEDULE[k]} rpm  (type the reading, Enter) <<<", flush=True)
                    while select.select([sys.stdin], [], [], 0)[0]:
                        s = sys.stdin.readline().strip()
                        if not s:
                            break
                        try:
                            v = float(s)
                        except ValueError:
                            print("   (not a number, ignored)"); continue
                        now = time.time()
                        step = int(((now - self.t0) // B_HOLD_S)) if self.t0 else -1
                        step = max(0, min(step, len(B_SCHEDULE) - 1))
                        tach_rows.append((now, step, B_SCHEDULE[step], v))
                        print(f"   tach {v:g} rpm recorded for step {step} ({B_SCHEDULE[step]} rpm)")
        except KeyboardInterrupt:
            print("\n   Ctrl-C: telling the daemon to stop the motor ...")
            proc.send_signal(signal.SIGINT)
        proc.wait(); th.join(timeout=2); out.close()
        if ps:
            ps.send_signal(signal.SIGINT); ps.wait(timeout=5)
        if tach_rows:
            with open(self.base + "_tach.csv", "w", newline="") as f:
                w = csv.writer(f); w.writerow(["unix_s", "step", "setpoint_rpm", "tach_rpm"]); w.writerows(tach_rows)
        return proc.returncode


# ------------------------------------------------------------------------------------------------------------------------------
def preflight(args, ppr, outdir):
    print(f"-- PREFLIGHT at {ppr} PPR (CPR {4 * ppr}): 8 s open loop ...")
    base = os.path.join(outdir, f"preflight_{ppr}ppr_{datetime.datetime.now():%H%M%S}")
    d = Daemon(args, base, ppr, "P")
    d.run(tach=False, pidstat=False)
    ok, msgs = run_qc(base + ".csv", "P", 4 * ppr)
    for m in msgs:
        print("   " + m)
    return ok


def cmd_warmup(args):
    base = os.path.join(args.dir, "warmup")
    os.makedirs(args.dir, exist_ok=True)
    print(f"Warm-up: open loop, 40 % PWM, {args.minutes:g} min. Ctrl-C ends it early.")
    c = shlex.split(args.daemon) + ["--test", "--cpr=1024", "--fifo", "--no-stream", "--no-torque", "--no-pi", "--pwm=1638",
                                    f"--duration={int(args.minutes * 60)}", f"--csv={base}.csv", f"--abort-rpm={ABORT_RPM_A}"]
    p = subprocess.Popen(c)
    try:
        p.wait()
    except KeyboardInterrupt:
        p.send_signal(signal.SIGINT); p.wait()


def cmd_preflight(args):
    os.makedirs(args.dir, exist_ok=True)
    sys.exit(0 if preflight(args, args.ppr, args.dir) else 1)


def cmd_run(args):
    plan_path = os.path.join(args.dir, "run_plan.csv")
    plan = list(csv.DictReader(open(plan_path)))
    campaign_path = os.path.join(args.dir, "campaign.json")
    if os.path.exists(campaign_path):
        campaign = json.load(open(campaign_path))
    else:
        print("One-time notes for the whole campaign (kept in campaign.json; Enter to skip):")
        campaign = dict(started=datetime.datetime.now().isoformat(timespec="seconds"),
                        supply=ask("  supply: source and nominal voltage (e.g. 'bench PSU 24.0 V'): "),
                        load=ask("  load condition (e.g. 'no impeller, shaft coupled to torque sensor, no vessel'): "),
                        coupling=ask("  shaft coupling / encoder mounting notes: "),
                        tach=ask("  handheld tachometer: model, resolution, accuracy: "))
        json.dump(campaign, open(campaign_path, "w"), indent=2)
    st = platform_state(args.repo)
    for w in platform_warnings(st):
        print("!! " + w)
    if platform_warnings(st) and not args.force:
        sys.exit("Fix the platform first (or --force).")

    current_ppr = None
    for row in plan:
        seq, ppr, test, rep = int(row["seq"]), int(row["ppr"]), row["test"], int(row["repeat"])
        if seq < args.from_seq or (args.only and test != args.only):
            continue
        existing = glob.glob(os.path.join(args.dir, f"*_{ppr}ppr_{TOKEN[args.estimator]}_r{rep}_test{test}.meta.json"))
        if existing and not args.redo:
            print(f"[{seq}] {ppr} PPR test {test} r{rep}: already done ({os.path.basename(existing[0])}), skipping")
            current_ppr = ppr
            continue
        print(f"\n==== [{seq}/{len(plan)}] {ppr} PPR (CPR {4 * ppr}, DIP {row['dip']})  test {test}  repeat {rep}  role {row['role']}  block {row['block']} ====")
        if int(row["dip_change"]) or current_ppr != ppr:
            if int(row["dip_change"]):
                print(f"   SET THE DIP SWITCH TO {row['dip']} (switches 1-4) for {ppr} PPR, power-cycle the encoder supply, then:")
            ask("   press Enter when the encoder is ready (motor stopped) ... ")
            for attempt in range(3):
                if preflight(args, ppr, args.dir):
                    break
                ask(f"   Preflight FAILED (attempt {attempt + 1}/3). Fix it (DIP code {row['dip']}, power-cycle, wiring) and press Enter to retry ... ")
            else:
                sys.exit("Preflight failed three times: stopping.")
            current_ppr = ppr
        if args.settle_s > 0 and not args.no_settle:
            print(f"   conditioning: {args.settle_s:g} s at 30 % PWM so every run starts from the same thermal state ...")
            c = shlex.split(args.daemon) + ["--test", f"--cpr={4 * ppr}", "--fifo", "--no-stream", "--no-torque", "--no-pi", "--pwm=1229",
                                            f"--duration={int(args.settle_s)}", f"--csv=/tmp/ppr_settle.csv", f"--abort-rpm={ABORT_RPM_A}"]
            subprocess.run(c, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        v_in = ask("   supply voltage now (V, Enter to skip): ")
        t_motor = ask("   motor temperature now (deg C, Enter to skip): ")
        stamp = datetime.datetime.now()
        base = os.path.join(args.dir, f"mixr1_log_{stamp:%Y%m%d_%H%M%S}_{ppr}ppr_{TOKEN[args.estimator]}_r{rep}_test{test}")
        before = platform_state(args.repo)
        d = Daemon(args, base, ppr, test, row["role"])
        rc = d.run(tach=(test == "B"))
        after = platform_state(args.repo)
        ok, msgs = run_qc(base + ".csv", test, 4 * ppr)
        meta = dict(seq=seq, ppr=ppr, cpr=4 * ppr, dip=row["dip"], test=test, repeat=rep, role=row["role"], block=int(row["block"]),
                    estimator=args.estimator, loop_period_us=10000,
                    schedule=A_SCHEDULE if test == "A" else B_SCHEDULE, hold_s=A_HOLD_S if test == "A" else B_HOLD_S,
                    schedule_unit="pwm_percent" if test == "A" else "rpm", t0_unix=d.t0, daemon_cmd=d.command(), return_code=rc,
                    supply_v=v_in, motor_temp_c=t_motor, platform_before=before, platform_after=after, qc_ok=ok, qc=msgs, campaign=campaign,
                    csv=os.path.basename(base) + ".csv")
        json.dump(meta, open(base + ".meta.json", "w"), indent=2)
        print(f"   QC: {'PASS' if ok else 'WARN'}")
        for m in msgs:
            print("   " + m)
        if not ok and not args.keep_going:
            if ask("   QC warnings: [Enter] keep the run, 'r' = discard and redo this plan row, 'q' = quit: ").lower() == "r":
                for f in glob.glob(base + "*"):
                    os.rename(f, f + ".discarded")
                print("   discarded; rerun this command to repeat it (use --from-seq", seq, ")")
                sys.exit(1)
        if "0x0" not in after["throttled"] and "n/a" not in after["throttled"]:
            print(f"!! throttling flags after the run: {after['throttled']}")
    print("\nCampaign finished. Next: python3 Tools/ppr/analyze_ppr_study.py --dir", args.dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["run", "warmup", "preflight"])
    ap.add_argument("--dir", required=True)
    ap.add_argument("--daemon", default="sudo ./build/mixr1_daemon", help="command that starts the patched daemon (root is needed for SCHED_FIFO)")
    ap.add_argument("--pidstat", default="pidstat", help="pidstat command (sudo apt install sysstat)")
    ap.add_argument("--no-pidstat", action="store_true")
    ap.add_argument("--estimator", choices=["aligned", "newest"], default="aligned",
                    help="speed estimator in the daemon: aligned snapshots (default) or the paper's newest-edge synchronous CET. Use a SEPARATE --dir per estimator")
    ap.add_argument("--torque", action="store_true", help="leave the RS485 torque sensor ON (default off: no tare delay, no serial load)")
    ap.add_argument("--repo", default=".", help="git repository of the firmware (commit hash goes into every .meta.json)")
    ap.add_argument("--ppr", type=int, default=256, help="preflight: PPR to check")
    ap.add_argument("--minutes", type=float, default=5.0, help="warmup length")
    ap.add_argument("--from-seq", type=int, default=1)
    ap.add_argument("--only", choices=["A", "B"], default=None)
    ap.add_argument("--redo", action="store_true", help="repeat runs that already have a .meta.json")
    ap.add_argument("--settle-s", type=float, default=45.0, help="conditioning spin before every run (same thermal state)")
    ap.add_argument("--no-settle", action="store_true")
    ap.add_argument("--keep-going", action="store_true", help="never ask about QC warnings")
    ap.add_argument("--force", action="store_true", help="continue despite platform warnings")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    {"run": cmd_run, "warmup": cmd_warmup, "preflight": cmd_preflight}[args.cmd](args)


if __name__ == "__main__":
    main()

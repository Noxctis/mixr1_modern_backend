#!/usr/bin/env python3
"""Guided, auto-labelling runner for the Chapter 5 tests that are NOT the PPR campaign (stdlib only, Python 3.8+).

    python3 Tools/ch5/ch5_guided.py --daemon "./build/mixr1_daemon"          # menu; everything is saved in logs/ch5_<date>/
    python3 Tools/ch5/ch5_guided.py --daemon ... --yes run steps              # scripted: run one test with defaults

What it does for you:
  * asks the session questions once (impeller, fluid, temperatures, torque settings) and saves them in session.json,
  * builds every daemon command (CPR 1024 = 256 PPR, FIFO, abort limits from the impeller) so you type no flags,
  * names every file <test>_<label>_<date>_<time>.* and records it in index.csv with its command, return code and quick checks,
  * stamps events when you press Enter during the disturbance runs, records tachometer / pass-fail entries you type,
  * shows what is done and what is missing (menu 14), and runs the analysis on everything (menu 13).
It never edits raw data. Safe to quit and resume: choose the same --dir again.
"""
import argparse, csv, datetime, glob, json, os, re, select, shlex, signal, statistics, subprocess, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYZE = os.path.join(HERE, "analyze_ch5.py")
ABORT = {"96": 1500, "174": 400}
SETPOINTS = {"96": [100, 250, 500, 1000, 1400], "174": [50, 100, 200, 300, 350]}
REPS = 3
YES = False


# ------------------------------------------------------------------------------------------------ small helpers
def say(msg=""):
    print(msg, flush=True)


def ask(prompt, default=""):
    if YES or not sys.stdin.isatty():
        return default
    try:
        v = input(f"{prompt}" + (f" [{default}]" if default != "" else "") + ": ").strip()
    except EOFError:
        v = ""
    return v or default


def yesno(prompt, default=True):
    d = "y" if default else "n"
    return ask(f"{prompt} (y/n)", d).lower().startswith("y")


def wait_enter(prompt):
    if YES or not sys.stdin.isatty():
        return
    try:
        input(f"\n>>> {prompt}  (press Enter when done) ")
    except EOFError:
        pass


def stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


class Session:
    def __init__(self, a):
        self.dir = a.dir or os.path.join("logs", "ch5_" + datetime.date.today().strftime("%Y%m%d"))
        os.makedirs(self.dir, exist_ok=True)
        self.path = os.path.join(self.dir, "session.json")
        self.daemon = a.daemon
        self.d = json.load(open(self.path)) if os.path.exists(self.path) else {}
        self.index_path = os.path.join(self.dir, "index.csv")

    def save(self):
        json.dump(self.d, open(self.path, "w"), indent=2)

    @property
    def imp(self):
        return str(self.d.get("impeller", "96"))

    def setup(self, force=False):
        if self.d.get("ready") and not force:
            say(f"Session already set up ({self.d.get('impeller')} mm, fluid {self.d.get('fluid')}). Choose menu 1 again to change it.")
            return
        say("\nSESSION SETUP: answer once. These go into every file's record.")
        say("Before you start: DIP switch must be at 0011 (256 PPR), vessel filled to the standard volume, impeller fitted.")
        self.d["impeller"] = ask("Impeller diameter in mm (96 or 174)", self.d.get("impeller", "96"))
        self.d["fluid"] = ask("Fluid (e.g. tap water)", self.d.get("fluid", "water"))
        self.d["fluid_temp_c"] = ask("Fluid temperature now, deg C", self.d.get("fluid_temp_c", ""))
        self.d["room_temp_c"] = ask("Room temperature, deg C", self.d.get("room_temp_c", ""))
        self.d["supply_v"] = ask("Supply voltage, V", self.d.get("supply_v", ""))
        self.d["level"] = ask("Liquid level / volume (e.g. '5 L' or '12 cm')", self.d.get("level", ""))
        self.d["baffles"] = ask("Baffles fitted? (yes/no)", self.d.get("baffles", "no"))
        say("\nTorque sensor flags (the ones that WORK on your rig), e.g.:")
        say("  --torque-port=/dev/ttyUSB0 --torque-baud=9600 --torque-slave=1 --torque-reg=0 --torque-regcount=2 --torque-format=int32 --torque-word-order=big --torque-scale=0.001")
        say("Leave empty to run WITHOUT torque (the torque tests then cannot run).")
        self.d["torque_flags"] = ask("Torque flags", self.d.get("torque_flags", ""))
        self.d["notes"] = ask("Anything else worth remembering", self.d.get("notes", ""))
        self.d["git_commit"] = sh("git rev-parse HEAD")
        self.d["git_dirty"] = bool(sh("git status --porcelain")) if self.d["git_commit"] != "n/a" else None
        self.d["abort_rpm"] = ABORT.get(self.imp, 1500)
        self.d["ready"] = True
        self.save()
        say(f"Saved {self.path}. Safety cut-off for this impeller: {self.d['abort_rpm']} rpm.")

    def need_ready(self):
        if not self.d.get("ready"):
            say("Run menu 1 (session setup) first.")
            self.setup()

    def log_run(self, test, label, base, cmd, rc, qc, extra=None):
        new = not os.path.exists(self.index_path)
        with open(self.index_path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "test", "label", "file", "return_code", "quick_check", "command"])
            w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), test, label, os.path.basename(base), rc, qc, " ".join(cmd)])
        meta = dict(test=test, label=label, session=self.d, command=cmd, return_code=rc, quick_check=qc, **(extra or {}))
        json.dump(meta, open(base + ".meta.json", "w"), indent=2)

    def runs(self, test=None):
        if not os.path.exists(self.index_path):
            return []
        rows = list(csv.DictReader(open(self.index_path)))
        return [r for r in rows if test is None or r["test"] == test]


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10).stdout.strip() or "n/a"
    except Exception:
        return "n/a"


# ------------------------------------------------------------------------------------------------ running the daemon
def read_csv_cols(path):
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


def quick_check(csv_path, expect_s=None):
    if not os.path.exists(csv_path):
        return "NO CSV WRITTEN"
    try:
        c = read_csv_cols(csv_path)
    except Exception as exc:
        return f"unreadable CSV ({exc})"
    key = "elapsed_s" if "elapsed_s" in c else list(c)[0]
    n, last = len(c[key]), c[key][-1] if c[key] else 0
    msg = f"{n} rows, {last:.1f} s"
    if expect_s and last < 0.95 * expect_s:
        msg += f"  WARNING: expected about {expect_s:.0f} s, run ended early (abort / trip?)"
    if "late_us" in c and c["late_us"]:
        msg += f", max lateness {max(c['late_us']):.0f} us"
    return msg


def daemon_cmd(S, csv_path, extra, torque=True):
    c = shlex.split(S.daemon) + ["--test", "--cpr=1024", "--fifo", "--no-stream", "--wait-rest=30", f"--abort-rpm={S.d['abort_rpm']}", f"--csv={csv_path}"]
    tq = S.d.get("torque_flags", "").strip()
    if tq and torque:
        c += shlex.split(tq)
    else:
        c += ["--no-torque"]
    return c + extra


def run_daemon(S, test, label, extra, expect_s=None, events=False, torque=True, platform_log=False):
    base = os.path.join(S.dir, f"{test}_{label}_{stamp()}")
    cmd = daemon_cmd(S, base + ".csv", extra, torque)
    say(f"\n--- RUN {test} / {label}")
    say("    file: " + os.path.basename(base) + ".csv")
    say("    command: " + " ".join(shlex.quote(x) for x in cmd))
    out = open(base + ".stdout.txt", "w")
    t0 = {"v": None}
    ev_rows = []
    plog = None
    if platform_log:
        plog = subprocess.Popen([sys.executable, ANALYZE, "platform-log", "--out", base + ".platform.csv", "--period", "10"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    flagged = []

    def pump():
        for line in proc.stdout:
            out.write(line); out.flush()
            m = re.search(r"t0_unix=([0-9.]+)", line)
            if m:
                t0["v"] = float(m.group(1))
            if re.search(r"\[SAFETY\]|ABORT|TRIP|WARNING|CRITICAL|Cannot|STALL|step \d|RPM target", line):
                say("    | " + line.rstrip()); flagged.append(line.rstrip())
    th = threading.Thread(target=pump, daemon=True); th.start()
    if events and sys.stdin.isatty() and not YES:
        say("    >>> Press Enter at the moment of EACH disturbance (type a short label first if you like, e.g. 'water drop'). The run stops by itself.")
    try:
        while th.is_alive():
            time.sleep(0.1)
            if events and sys.stdin.isatty() and not YES and select.select([sys.stdin], [], [], 0)[0]:
                lab = sys.stdin.readline().strip() or "event"
                ev_rows.append((time.time(), lab)); say(f"    event stamped: {lab}")
    except KeyboardInterrupt:
        say("\n    Ctrl-C: asking the daemon to stop the motor ...")
        proc.send_signal(signal.SIGINT)
    proc.wait(); th.join(timeout=3); out.close()
    if plog:
        plog.send_signal(signal.SIGINT)
    if ev_rows:
        with open(base + ".events.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["unix_s", "label"]); w.writerows(ev_rows)
    qc = quick_check(base + ".csv", expect_s)
    say(f"    result: return code {proc.returncode}; {qc}")
    S.log_run(test, label, base, cmd, proc.returncode, qc, dict(t0_unix=t0["v"], flagged=flagged[-10:]))
    return base, proc.returncode, t0["v"]


# ------------------------------------------------------------------------------------------------ the tests
def t_smoke(S):
    S.need_ready()
    say("\nSMOKE TEST: the motor spins about 8 s at 15 % duty. Make sure the shaft is free and the vessel is as for the tests.")
    wait_enter("Ready?")
    run_daemon(S, "smoke", "pwm15", ["--no-pi", "--pwm=614", "--duration=8"], expect_s=8)


def t_duty(S):
    S.need_ready()
    up = ",".join(str(x) for x in range(0, 101, 4)); down = ",".join(str(x) for x in range(100, -1, -4))
    fup = ",".join(str(x) for x in range(0, 16)); fdown = ",".join(str(x) for x in range(15, -1, -1))
    say("\nTC-U6 DUTY SWEEP (4 runs, about 22 min). Vessel at working volume, impeller fitted. The run stops itself at the speed cut-off or the power limit; that is normal.")
    wait_enter("Ready to start run 1 of 4 (coarse, going up)?")
    run_daemon(S, "duty", "coarse_up_hold12", ["--no-pi", f"--schedule={up}", "--hold=12"])
    run_daemon(S, "duty", "coarse_down_hold12", ["--no-pi", f"--schedule={down}", "--hold=12"])
    run_daemon(S, "duty", "fine_up_hold8", ["--no-pi", f"--schedule={fup}", "--hold=8"], expect_s=128)
    run_daemon(S, "duty", "fine_down_hold8", ["--no-pi", f"--schedule={fdown}", "--hold=8"], expect_s=128)


def t_steps(S):
    S.need_ready()
    sp = SETPOINTS.get(S.imp, SETPOINTS["96"])
    say(f"\nTC-S1 STEP RESPONSE FROM REST: setpoints {sp} rpm, {REPS} repeats each = {len(sp) * REPS} runs of 40 s (about {len(sp) * REPS * 1.3:.0f} min).")
    wait_enter("Ready?")
    for r in range(1, REPS + 1):
        for T in sp:
            run_daemon(S, "steps", f"T{T}_r{r}", ["--pi", f"--target={T}", "--duration=40", "--start-at-zero"], expect_s=40)


def t_refsteps(S):
    S.need_ready()
    lo, hi = (300, 400) if S.imp == "96" else (150, 250)
    say(f"\nSMALL REFERENCE STEPS {lo} -> {hi} -> {lo} rpm, {REPS} repeats.")
    wait_enter("Ready?")
    for r in range(1, REPS + 1):
        run_daemon(S, "refstep", f"{lo}_{hi}_r{r}", ["--pi", f"--target={lo}", f"--ref-step-to={hi}", "--ref-step-at=15", f"--ref-step-to2={lo}", "--ref-step-at2=30", "--duration=45"], expect_s=45)


def t_limiter(S):
    S.need_ready()
    say("\nPOWER-LIMITER RUN. Choose a HIGH setpoint where the power cap engages (look for '[SAFETY] ... power limit reached' in the output), then a lower setpoint to step down to.")
    hi = ask("High setpoint, rpm", "1300" if S.imp == "96" else "340")
    lo = ask("Lower setpoint to step down to, rpm", "600" if S.imp == "96" else "150")
    wait_enter("Ready?")
    run_daemon(S, "limiter", f"{hi}_to_{lo}", ["--pi", f"--target={hi}", f"--ref-step-to={lo}", "--ref-step-at=20", "--duration=45"], expect_s=45)
    say("    Check the output above: did the power limit engage? If there was NO '[SAFETY]' line, repeat with a higher setpoint (menu 6).")


def t_disturb(S):
    S.need_ready()
    tgt = ask("Setpoint for the disturbance runs, rpm", "350")
    pwm = ask("Open-loop duty (counts 0-4095) that gives about the same speed (see your duty sweep; 15 % = 614)", "614")
    say("\nLOAD-DISTURBANCE RUNS: 120 s each. During the run you do the SAME disturbance sequence every time (e.g. five water drops, then baffles in, then baffles out).")
    say("Press Enter the instant each disturbance happens (type a label first if you want, e.g. 'drop1').")
    for mode in ("PI", "OPEN"):
        for r in range(1, REPS + 1):
            wait_enter(f"Run {mode} r{r}: be ready to do the disturbance sequence")
            extra = ["--pi", f"--target={tgt}"] if mode == "PI" else ["--no-pi", f"--pwm={pwm}"]
            run_daemon(S, "disturb", f"{mode}_{tgt}rpm_r{r}", extra + ["--duration=120"], expect_s=120, events=True)


def t_cek(S):
    say("\nCEK-A KNOB COMPARISON (the original open-loop knob). Set the knob to each mark, wait 30 s, read the handheld tachometer 3 times.")
    out = os.path.join(S.dir, f"cek_knob_{stamp()}.csv"); rows = []
    while True:
        lab = ask("Knob mark / nominal speed label (empty = finish)", "")
        if not lab:
            break
        nom = ask("Nominal speed for this mark, rpm", "")
        vals = []
        for i in range(1, 4):
            v = ask(f"  tachometer reading {i} (rpm)", "")
            if v:
                vals.append(float(v))
        if vals:
            rows.append([lab, nom, *vals, statistics.fmean(vals)])
        if YES:
            break
    if rows:
        with open(out, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["knob_mark", "nominal_rpm", "tach1", "tach2", "tach3", "tach_mean"]); w.writerows(rows)
        say("    saved " + os.path.basename(out)); S.log_run("cek", "knob", out[:-4], ["manual entry"], 0, f"{len(rows)} marks")


def t_torque_cold(S):
    S.need_ready()
    if not S.d.get("torque_flags"):
        return say("No torque flags in the session: run menu 1 and enter them.")
    mins = float(ask("Log length in minutes", "90"))
    say("\nTC-U3 COLD START. The amplifier must have been OFF for hours. Motor OFF, nothing touching the shaft.")
    say("Switch the amplifier ON now, then start this immediately.")
    wait_enter("Amplifier just switched on?")
    ambient = ask("Room temperature now, deg C", S.d.get("room_temp_c", ""))
    base = os.path.join(S.dir, f"torque_cold_{stamp()}")
    cmd = shlex.split(S.daemon) + ["--test-torque", f"--seconds={int(mins * 60)}"] + shlex.split(S.d["torque_flags"]) + [f"--csv={base}.csv"]
    say("    command: " + " ".join(shlex.quote(x) for x in cmd)); say(f"    Running {mins:g} min. Do paperwork; do NOT touch the shaft or switch the motor on.")
    with open(base + ".stdout.txt", "w") as out:
        rc = subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT).returncode
    qc = quick_check(base + ".csv"); say(f"    result: return code {rc}; {qc}"); S.log_run("torque_cold", "cold", base, cmd, rc, qc, dict(room_temp_start=ambient))


def t_torque_validate(S):
    S.need_ready()
    if not S.d.get("torque_flags"):
        return say("No torque flags in the session: run menu 1 and enter them.")
    say("\nTC-U4 TORQUE VALIDATION with hanging masses on a lever arm: reference torque = m x g x r.")
    arm = ask("Lever arm length r in metres, measured with a caliper (e.g. 0.100)", S.d.get("arm_m", "0.100")); S.d["arm_m"] = arm; S.save()
    settle = int(float(ask("Seconds to wait after loading before reading", "30")))
    loads = os.path.join(S.dir, "loads.csv"); new = not os.path.exists(loads)
    f = open(loads, "a", newline=""); w = csv.writer(f)
    if new:
        w.writerow(["mass_kg", "arm_m", "measured_nm", "dir", "file"])
    say("Work in series: first UP (0, small ... large), then DOWN (large ... 0). Repeat the pair 3 times. Include the 0 kg load in every series.")
    while True:
        direction = ask("Series direction: up / down (empty = finish)", "")
        if direction not in ("up", "down"):
            break
        while True:
            m = ask("  Hang this mass now. Mass in kg (empty = end of series)", "")
            if m == "":
                break
            say(f"    waiting {settle} s for the load to settle ...")
            if not YES:
                time.sleep(settle)
            base = os.path.join(S.dir, f"torque_load_{float(m) * 1000:.0f}g_{direction}_{stamp()}")
            cmd = shlex.split(S.daemon) + ["--test-torque", "--seconds=25"] + shlex.split(S.d["torque_flags"]) + [f"--csv={base}.csv"]
            with open(base + ".stdout.txt", "w") as out:
                rc = subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT).returncode
            try:
                c = read_csv_cols(base + ".csv"); yk = [k for k in c if "torque" in k.lower()][0]
                vals = [v for t, v, ok in zip(c[list(c)[0]], c[yk], c.get("ok", [1] * len(c[yk]))) if t >= 5 and ok == 1 and v == v]
                mean = statistics.fmean(vals)
            except Exception as exc:
                say(f"    could not read the result ({exc}); repeat this load"); continue
            w.writerow([m, arm, f"{mean:.6f}", direction, os.path.basename(base) + ".csv"]); f.flush()
            S.log_run("torque_load", f"{m}kg_{direction}", base, cmd, rc, f"mean {mean:.5f} N.m")
            say(f"    mass {m} kg -> mean torque {mean:.5f} N.m  (saved in loads.csv)")
        if YES:
            break
    f.close()


def t_endurance(S, minutes=None):
    S.need_ready()
    mins = float(minutes if minutes is not None else ask("Endurance duration in minutes (use the duration in your TC-S2/S3 test case)", "120"))
    tgt = ask("Representative setpoint, rpm", "350")
    ftemp0 = ask("Fluid temperature at START, deg C", S.d.get("fluid_temp_c", ""))
    say(f"\nENDURANCE: {mins:g} min at {tgt} rpm. Torque and power are logged. Leave the rig alone; Ctrl-C stops it safely.")
    wait_enter("Ready?")
    base, rc, t0 = run_daemon(S, "endurance", f"{tgt}rpm_{mins:g}min", ["--pi", f"--target={tgt}", f"--duration={int(mins * 60)}"], expect_s=mins * 60, platform_log=True)
    ftemp1 = ask("Fluid temperature at END, deg C", "")
    S.d["endurance_fluid_temps"] = [ftemp0, ftemp1]; S.save()
    say("Now stop the motor, wait for the shaft to rest, and (if torque is on) take an idle torque reading with menu 9-style log: it shows the torque zero drift across the run.")


MODE1 = [
    "Boot: motor is stopped and the impeller menu shows 96 MM / 174 MM",
    "Knob selects the impeller in the menu",
    "Knob turned slowly changes the target 1 rpm per click (motor stopped, nothing moves)",
    "Knob turned fast jumps to the next multiple of 10",
    "First KO press shows 'IMPELLER 96 MM?' (or the selected impeller) and does not start the motor",
    "Second KO press within 10 s starts the motor",
    "Knob push cancels the start confirmation",
    "KO press while running stops the motor",
    "Single knob click zeroes the target",
    "Double click within 0.5 s returns to the impeller menu with the motor off",
    "Knob limit: cannot set above 1400 rpm (96 mm) / 350 rpm (174 mm)",
    "Display shows the same target and speed that the logs show",
    "Dashboard silent for more than 2 s (after it was connected) returns to Mode 1 with the motor stopped",
]


def t_mode1(S):
    say("\nMODE 1 CHECKLIST. For each item do the action on the bench (no water needed), then answer p = pass, f = fail, n = not testable.")
    rows = []
    for i, item in enumerate(MODE1, 1):
        say(f"\n[{i}/{len(MODE1)}] {item}")
        res = ask("  result p/f/n", "n"); note = ask("  what you saw (short)", "")
        rows.append([i, item, res, note])
    out = os.path.join(S.dir, f"mode1_checklist_{stamp()}")
    with open(out + ".csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["no", "action_expected", "result", "observed"]); w.writerows(rows)
    with open(out + ".md", "w") as f:
        f.write("| # | Action / expected | Result | Observed |\n|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r[0]} | {r[1]} | {r[2].upper()} | {r[3]} |\n")
    S.log_run("mode1", "checklist", out, ["manual entry"], 0, f"{sum(r[2] == 'p' for r in rows)} pass, {sum(r[2] == 'f' for r in rows)} fail")
    say("    saved " + os.path.basename(out) + ".csv / .md")


def t_dashboard_notes(S):
    say("\nDASHBOARD / SWEEP / INTEGRITY are done in the dashboard itself. This only registers your files so nothing gets lost.")
    say("Copy the dashboard exports (sequence_summary.csv, sequence_log.csv, .mat) into: " + S.dir)
    wait_enter("Files copied?")
    for p in sorted(glob.glob(os.path.join(S.dir, "mixr1_*"))):
        say("   found " + os.path.basename(p)); S.log_run("dashboard", os.path.basename(p), p, ["copied by hand"], 0, "registered")


# ------------------------------------------------------------------------------------------------ status and analysis
EXPECT = [("smoke", "Smoke test", 1), ("duty", "TC-U6 duty sweeps", 4), ("steps", "TC-S1 steps from rest", None), ("refstep", "Small reference steps", REPS),
          ("limiter", "Power-limiter run", 1), ("disturb", "Load disturbance PI + open loop", 2 * REPS), ("cek", "CEK-A knob table", 1), ("torque_cold", "TC-U3 cold start", 1),
          ("torque_load", "TC-U4 loads (series of masses)", 12), ("endurance", "Endurance", 1), ("mode1", "Mode 1 checklist", 1)]


def t_status(S):
    say(f"\nSESSION {S.dir}\n  impeller {S.d.get('impeller')} mm, fluid {S.d.get('fluid')} {S.d.get('fluid_temp_c')} C, commit {S.d.get('git_commit', '')[:10]}")
    for key, name, n in EXPECT:
        k = len(S.runs(key))
        need = n if n else len(SETPOINTS.get(S.imp, SETPOINTS["96"])) * REPS
        say(f"  [{'x' if k >= need else ' '}] {name}: {k} of {need}")
    say("  (PPR campaign: see its own run_plan.csv.  Dashboard sweep, integrity and TC-S5 files: register with menu 12.)")


def run_analysis(args):
    r = subprocess.run([sys.executable, ANALYZE] + args, capture_output=True, text=True)
    say(r.stdout[-1500:] if r.returncode == 0 else "    analysis failed: " + r.stderr[-800:])


def t_analyze(S):
    out = os.path.join(S.dir, "analysis"); os.makedirs(out, exist_ok=True)
    def files(test, pat=""):
        return [os.path.join(S.dir, r["file"] + ".csv") for r in S.runs(test) if pat in r["label"] and os.path.exists(os.path.join(S.dir, r["file"] + ".csv"))]
    say("\nANALYSING everything that exists. Results: " + out)
    up, dn = files("duty", "coarse_up"), files("duty", "coarse_down")
    if up and dn:
        run_analysis(["duty", "--csv", up[-1], dn[-1], "--labels", "up,down", "--hold", "12", "--out", os.path.join(out, "duty_coarse")])
    fu, fd = files("duty", "fine_up"), files("duty", "fine_down")
    if fu and fd:
        run_analysis(["duty", "--csv", fu[-1], fd[-1], "--labels", "up,down", "--hold", "8", "--out", os.path.join(out, "duty_fine")])
    st = files("steps") + files("refstep") + files("limiter")
    if st:
        run_analysis(["steps", "--csv"] + st + ["--out", os.path.join(out, "steps")])
    for r in S.runs("disturb"):
        ev = os.path.join(S.dir, r["file"] + ".events.csv"); meta = os.path.join(S.dir, r["file"] + ".meta.json")
        if os.path.exists(ev) and os.path.exists(meta):
            t0 = json.load(open(meta)).get("t0_unix")
            if t0:
                run_analysis(["disturbance", "--csv", os.path.join(S.dir, r["file"] + ".csv"), "--events", ev, "--t0-unix", str(t0), "--out", os.path.join(out, "disturbance_" + r["label"])])
    for r in S.runs("torque_cold"):
        run_analysis(["torque-drift", "--csv", os.path.join(S.dir, r["file"] + ".csv"), "--out", os.path.join(out, "torque_drift")])
    loads = os.path.join(S.dir, "loads.csv")
    if os.path.exists(loads):
        run_analysis(["torque-validate", "--csv", loads, "--u-mass", ask("Mass uncertainty, kg (scale resolution / calibration)", "0.0005"), "--u-arm", ask("Arm length uncertainty, m", "0.0002"),
                      "--g", ask("Local g, m/s^2 (Cebu about 9.782)", "9.782"), "--out", os.path.join(out, "torque_validation")])
    for r in S.runs("endurance"):
        base = os.path.join(S.dir, r["file"]); args = ["endurance", "--csv", base + ".csv", "--stdout", base + ".stdout.txt", "--out", os.path.join(out, "endurance")]
        if os.path.exists(base + ".platform.csv"):
            args += ["--platform", base + ".platform.csv"]
        run_analysis(args)
    say("\nStill by hand: power uncertainty (README 5d), integrity (needs the dashboard export), plausibility (needs your literature curve).")


# ------------------------------------------------------------------------------------------------ menu
MENU = [("1", "Session setup (do this first, once per day)", lambda S: S.setup(True)), ("2", "Smoke test (8 s spin)", t_smoke), ("3", "TC-U6 duty sweeps (22 min)", t_duty),
        ("4", "TC-S1 step response from rest", t_steps), ("5", "TC-S1 small reference steps", t_refsteps), ("6", "TC-S1 power-limiter run", t_limiter),
        ("7", "TC-S1 load disturbance, PI then open loop", t_disturb), ("8", "CEK-A knob table (tachometer)", t_cek), ("9", "TC-U3 torque cold-start log", t_torque_cold),
        ("10", "TC-U4 torque validation with masses", t_torque_validate), ("11", "Endurance run", t_endurance), ("12", "Register dashboard files (sweep, integrity)", t_dashboard_notes),
        ("13", "Analyse everything so far", t_analyze), ("14", "What is done / what is missing", t_status), ("15", "Mode 1 checklist", t_mode1), ("0", "Quit", None)]
BY_NAME = {"setup": lambda S: S.setup(True), "smoke": t_smoke, "duty": t_duty, "steps": t_steps, "refstep": t_refsteps, "limiter": t_limiter, "disturb": t_disturb, "cek": t_cek,
           "torque-cold": t_torque_cold, "torque-validate": t_torque_validate, "endurance": t_endurance, "dashboard": t_dashboard_notes, "analyze": t_analyze, "status": t_status, "mode1": t_mode1}


def main():
    global YES, REPS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--daemon", default="./build/mixr1_daemon", help="how to start the patched daemon (sudo is added for you if it is not root)")
    ap.add_argument("--dir", default=None, help="session folder (default logs/ch5_<today>)")
    ap.add_argument("--yes", action="store_true", help="no questions: use defaults (for scripted runs / self-test)")
    ap.add_argument("--reps", type=int, default=3, help="repeats for step / disturbance runs")
    ap.add_argument("cmd", nargs="*", help="optional: run <name> (" + ", ".join(BY_NAME) + ")")
    a = ap.parse_args(); YES = a.yes; REPS = a.reps
    if os.geteuid() != 0 and not a.daemon.startswith("sudo") and "fake_daemon" not in a.daemon:
        a.daemon = "sudo " + a.daemon
    S = Session(a)
    if a.cmd:
        name = a.cmd[-1]; fn = BY_NAME[name]
        if YES and not S.d.get("ready"):
            S.d.update(impeller="96", fluid="water", ready=True, abort_rpm=ABORT["96"], torque_flags=os.environ.get("CH5_TQ", ""), git_commit=sh("git rev-parse HEAD")); S.save()
        return fn(S)
    say("MIXR-1 Chapter 5 guided runner. Every file is named and recorded for you.")
    S.setup()
    while True:
        say("\n==== MENU ====")
        for k, name, _ in MENU:
            say(f"  {k:>2}  {name}")
        c = ask("Choose a number", "").strip()
        if c == "0":
            return
        for k, name, fn in MENU:
            if k == c and fn:
                try:
                    fn(S)
                except KeyboardInterrupt:
                    say("\n(interrupted; back to the menu)")
                break
        else:
            say("Type one of the numbers above.")


if __name__ == "__main__":
    main()

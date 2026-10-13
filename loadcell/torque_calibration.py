#!/usr/bin/env python3
"""
read_loadcell.py - torque sensor test: you tell it the real weight and the lever arm, it records the sensor for each trial and saves a CSV.

You are asked for:
  1. the lever arm length (distance from the sensor axis to the line of the hanging weight)
  2. how many trials to do for each weight
  3. the REAL weight of each load you attach (from a scale), one load after another; type  done  when you have finished
For every trial the script reads the amplifier over Modbus (averaging a few samples) and appends a row to a CSV file
(one file per run, named loadcell_test_<date>_<time>.csv, written after every trial so nothing is lost).

Run it from a real terminal (not an output panel):   python3 read_loadcell.py
Watch the zero / noise / drift live (nothing attached):  python3 read_loadcell.py --watch            (add --duration 120 to stop by itself)
Rehearsal without hardware:                           python3 read_loadcell.py --simulate   (also works with --watch)
"""
import csv
import math
import os
import statistics
import struct
import sys
import time
from datetime import datetime

# ------------------------------------------------------------------------ settings (same as your original script)
PORT = "/dev/ttyUSB0"
BAUD = 9600        # must match the amplifier
SLAVE_ID = 1       # must match the amplifier
REG_ADDR = 0       # PLACEHOLDER: take from the manual's register map (find it with:  python3 torque_calibration.py --probe)
REG_COUNT = 2      # PLACEHOLDER: depends on 16-bit vs 32-bit weight

# how to turn the raw registers into a number (PLACEHOLDERS until you know the amplifier's format)
VALUE_FORMAT = "int32"      # int16 | uint16 | int32 | uint32 | float32
WORD_ORDER = "big"          # 32-bit values: "big" = first register holds the high word, "little" = second register does
SCALE = 0.001                 # reading = decoded number * SCALE   (for example 0.001 if the amplifier counts in mNm)
UNIT = "Nm"                 # label for the reading

SENSOR_CAPACITY_NM = 2.0    # PLACEHOLDER: the capacity of YOUR sensor (2, 10, 30 ... N.m); used for an overload warning
# Parts that stay on the sensor and add their own torque (used ONLY if you zeroed the bare sensor before mounting them):
#   (name, mass in g, distance of its centre of mass from the sensor axis in mm)
FIXED_ITEMS = [("printed lever arm", 9.01, 43.91),
               ("nut and bolt at the load point", 2.4, 102.0)]
G = 9.80665                   # local gravity in m/s2 (standard gravity 9.80665 would be 0.25 % higher)
SETTLE_S = 3.0              # wait this long after you press Enter, so the reading settles
N_SAMPLES = 10              # readings averaged for one trial
SAMPLE_INTERVAL_S = 0.2

SIMULATE = "--simulate" in sys.argv
WATCH = "--watch" in sys.argv
if SIMULATE:
    SCALE = 0.001           # the simulated amplifier counts in mN.m (1000 counts = 1 N.m)


# ------------------------------------------------------------------------ small helpers
def decode(regs):
    """Raw Modbus registers -> number."""
    if VALUE_FORMAT in ("int16", "uint16"):
        v = regs[0] & 0xFFFF
        return v - 0x10000 if (VALUE_FORMAT == "int16" and v >= 0x8000) else v
    if len(regs) < 2:
        raise ValueError("this VALUE_FORMAT needs REG_COUNT = 2")
    hi, lo = (regs[0], regs[1]) if WORD_ORDER == "big" else (regs[1], regs[0])
    u = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    if VALUE_FORMAT == "uint32":
        return u
    if VALUE_FORMAT == "int32":
        return u - 0x100000000 if u >= 0x80000000 else u
    if VALUE_FORMAT == "float32":
        return struct.unpack(">f", struct.pack(">I", u))[0]
    raise ValueError("unknown VALUE_FORMAT")


def parse_length_m(text):
    """'150', '150mm', '15cm', '0.15m' -> metres. A bare number is millimetres (a bare number below 2 is taken as metres)."""
    t = text.strip().lower().replace(" ", "")
    for unit, factor in (("mm", 0.001), ("cm", 0.01), ("m", 1.0)):
        if t.endswith(unit):
            return float(t[: -len(unit)]) * factor
    v = float(t)
    return v if v < 2 else v / 1000.0


def parse_weight_g(text):
    """'250', '250g', '0.25kg' -> (grams, was_ambiguous). A bare number is grams."""
    t = text.strip().lower().replace(" ", "")
    if t.endswith("kg"):
        return float(t[:-2]) * 1000.0, False
    if t.endswith("g"):
        return float(t[:-1]), False
    v = float(t)
    return v, (0 < v < 10)


def ask_text(prompt):
    return input(prompt).strip()


def ask_zero_method():
    """How the amplifier was zeroed decides whether the arm's own torque must be added to the expected torque. Returns (baseline N.m, text)."""
    print("\nHow was the amplifier zeroed (tared)?")
    print("  1 = WITH the lever arm, nut and bolt mounted and nothing hanging  -> their torque is cancelled; enter only what you hang on it  (recommended)")
    print("  2 = with the BARE sensor, before the arm was mounted             -> the arm's own torque is part of every reading")
    while True:
        c = ask_text("Choice [1]: ") or "1"
        if c in ("1", "2"):
            break
        print("  please type 1 or 2")
    if c == "1":
        return 0.0, "zeroed with the arm mounted (arm torque cancelled)"
    items = list(FIXED_ITEMS)
    print("  Parts left on the sensor:")
    for n, m, d in items:
        print(f"    {n}: {m:g} g at {d:g} mm = {m / 1000 * G * d / 1000:.5f} N.m")
    if not yes("  Are these the right parts and numbers?", True):
        items = []
        print("  Enter each part as: mass_in_g distance_from_axis_in_mm  (for example  9.01 43.91). Press Enter on an empty line to finish.")
        while True:
            t = ask_text("    part: ")
            if t == "":
                break
            try:
                m, d = [float(x) for x in t.replace(",", " ").split()[:2]]
                items.append(("part", m, d))
            except ValueError:
                print("    type two numbers, e.g.  2.4 102")
    base = sum(m / 1000 * G * d / 1000 for _, m, d in items)
    print(f"  -> torque already on the sensor at zero added weight: {base:.5f} N.m (added to every expected torque)")
    return base, f"zeroed bare; fixed parts add {base:.5f} N.m"


def ask_int(prompt, default):
    while True:
        s = ask_text(f"{prompt} [{default}]: ")
        if s == "":
            return default
        try:
            v = int(s)
            if v >= 1:
                return v
        except ValueError:
            pass
        print("  please enter a whole number, 1 or more")


def yes(prompt, default=False):
    s = ask_text(prompt + (" [Y/n]: " if default else " [y/N]: ")).lower()
    return default if s == "" else s.startswith("y")


# ------------------------------------------------------------------------ the reader (real Modbus, or a simulated sensor)
class FakeClient:
    """Stand-in for --simulate: reads roughly the torque you declared, with a little gain error, offset and noise."""

    def __init__(self):
        self.torque = 0.0
        self.reads = 0

    def connect(self):
        return True

    def close(self):
        pass

    def read(self):
        if os.environ.get("LOADCELL_SIM_FAIL"):
            return None
        import random
        self.reads += 1
        v = 1.004 * self.torque + 0.003 + 0.000004 * self.reads + random.gauss(0, 0.0007)
        raw = int(round(v / SCALE))
        u = raw & 0xFFFFFFFF
        return [(u >> 16) & 0xFFFF, u & 0xFFFF][:REG_COUNT] if REG_COUNT == 2 else [raw & 0xFFFF]


class Sensor:
    def __init__(self):
        if SIMULATE:
            self.client = FakeClient()
            print("*** SIMULATION: no real sensor is being read ***\n")
            return
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError:
            raise SystemExit("pymodbus is not installed for this Python:  sudo apt install python3-pymodbus python3-serial")
        try:
            self.client = ModbusSerialClient(port=PORT, baudrate=BAUD, parity="N", stopbits=1, bytesize=8, timeout=1)
        except RuntimeError as e:                                   # newer pymodbus needs pyserial for RS485
            raise SystemExit(f"{e}\nInstall it with:  sudo apt install python3-serial")
        if not self.client.connect():
            raise SystemExit(f"Could not open {PORT}.\n"
                             f"  'Could not exclusively lock port' means another program still has it: sudo fuser -v {PORT}  (stop it with pkill -f <name>)\n"
                             f"  Otherwise check the USB-RS485 adapter, the port name and permissions (sudo usermod -aG dialout $USER).")

    def set_torque(self, t):                                        # only used by the simulator
        if SIMULATE:
            self.client.torque = t

    def read(self):
        """One reading: a list of raw registers, or None on an error."""
        if SIMULATE:
            return self.client.read()
        rr = None
        for kw in ("slave", "device_id", "unit"):                   # pymodbus renamed slave -> device_id; try each
            try:
                rr = self.client.read_holding_registers(address=REG_ADDR, count=REG_COUNT, **{kw: SLAVE_ID})
                break
            except TypeError:
                continue
            except Exception:                                       # noqa: BLE001  (timeouts, framing errors)
                return None
        if rr is None or rr.isError():
            return None
        return list(rr.registers)

    def close(self):
        self.client.close()


def measure(sensor):
    """Average N_SAMPLES readings. Returns (mean, std, n, first_raw_registers) or None if the amplifier does not answer."""
    vals, first = [], None
    for _ in range(N_SAMPLES):
        regs = sensor.read()
        if regs is not None:
            if first is None:
                first = regs
            vals.append(decode(regs) * SCALE)
        time.sleep(SAMPLE_INTERVAL_S if not SIMULATE else 0)
    if len(vals) < max(3, N_SAMPLES // 2):
        return None
    return statistics.mean(vals), (statistics.pstdev(vals) if len(vals) > 1 else 0.0), len(vals), "|".join(str(r) for r in first)


# ------------------------------------------------------------------------ summary at the end
def summarise(rows):
    if not rows:
        return
    print("\n================ SUMMARY ================")
    groups = {}
    for r in rows:
        groups.setdefault(r["weight_g"], []).append(r)
    print(f"{'weight g':>9} {'trials':>7} {'expected ' + 'N.m':>12} {'mean reading':>13} {'spread (std)':>13}")
    pts = []
    for w, rs in sorted(groups.items()):
        vals = [r["reading"] for r in rs]
        m = statistics.mean(vals)
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        T = rs[0]["expected_torque_Nm"]
        print(f"{w:>9.1f} {len(rs):>7} {T:>12.4f} {m:>13.5f} {sd:>13.5f}")
        pts.append((T, m))
    if len(pts) >= 3:
        n = len(pts)
        sx, sy = sum(p[0] for p in pts), sum(p[1] for p in pts)
        sxx, sxy = sum(p[0] ** 2 for p in pts), sum(p[0] * p[1] for p in pts)
        den = n * sxx - sx * sx
        if den > 0:
            gain = (n * sxy - sx * sy) / den
            off = (sy - gain * sx) / n
            ybar = sy / n
            ss_tot = sum((p[1] - ybar) ** 2 for p in pts)
            ss_res = sum((p[1] - (gain * p[0] + off)) ** 2 for p in pts)
            r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
            print(f"\nStraight-line fit of the weight averages:  reading = {gain:.4f} x torque {off:+.4f}   (R2 = {r2:.5f})")
            print(f"  gain error {abs(gain - 1) * 100:.2f} % and zero offset {off:+.4f} {UNIT} (the ideal is gain 1.000, offset 0 if the reading is in N.m)")



# ------------------------------------------------------------------------ watch mode: is the zero stable?
def watch():
    """Live readings with nothing changing on the sensor: shows the zero offset, the noise and the drift."""
    duration = float(sys.argv[sys.argv.index("--duration") + 1]) if "--duration" in sys.argv else None
    sensor = Sensor()
    out = f"loadcell_watch_{datetime.now():%Y%m%d_%H%M%S}.csv"
    print("=== Live reading: leave the sensor completely untouched (no hand on the arm, no moving cables) ===")
    print(f"Ctrl+C stops it{'' if duration is None else f' (or it stops after {duration:g} s)'}. Every sample is saved to {os.path.abspath(out)}\n")
    t0, data, last_print, bad = time.time(), [], 0.0, 0
    try:
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "reading", "raw_registers", "unit"])
            while duration is None or time.time() - t0 < duration:
                t = time.time() - t0
                regs = sensor.read()
                if regs is None:
                    bad += 1
                else:
                    v = decode(regs) * SCALE
                    data.append((t, v))
                    w.writerow([round(t, 3), v, "|".join(str(r) for r in regs), UNIT])
                    f.flush()
                    if t - last_print >= 1.0:
                        last_print = t
                        recent = [x[1] for x in data[-20:]]
                        print(f"  {int(t) // 60:02d}:{int(t) % 60:02d}   {v:+.5f} {UNIT}    last {len(recent)}: mean {statistics.mean(recent):+.5f}  std {statistics.pstdev(recent):.5f}")
                time.sleep(0.2 if not SIMULATE else 0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
    if len(data) < 5:
        print(f"\nToo few readings ({len(data)}, {bad} errors) to summarise. Check the wiring and the register settings.")
        return
    ts, vs = [d[0] for d in data], [d[1] for d in data]
    n, tm, vm = len(vs), statistics.mean(ts), statistics.mean(vs)
    slope = sum((t - tm) * (v - vm) for t, v in data) / max(sum((t - tm) ** 2 for t in ts), 1e-12)
    print("\n================ ZERO CHECK ================")
    print(f"{n} readings over {ts[-1]:.0f} s ({bad} errors)")
    print(f"mean {vm:+.5f} {UNIT}     std (noise) {statistics.pstdev(vs):.5f}     min {min(vs):+.5f}     max {max(vs):+.5f}")
    print(f"drift: {slope * 60:+.5f} {UNIT} per minute   (total change over the run about {slope * ts[-1]:+.5f})")
    if SENSOR_CAPACITY_NM:
        print(f"as a share of the {SENSOR_CAPACITY_NM:g} N.m capacity: offset {abs(vm) / SENSOR_CAPACITY_NM * 100:.2f} % FS, noise {statistics.pstdev(vs) / SENSOR_CAPACITY_NM * 100:.3f} % FS "
              f"(datasheet: zero balance +-2 % FS, repeatability 0.05 % FS)")
    print("A steady offset is fine: zero (tare) the amplifier with the lever arm and empty hanger mounted. A reading that keeps wandering is the real problem:")
    print("let the sensor and amplifier warm up for 15-30 minutes, keep the cable still, and repeat.")

# ------------------------------------------------------------------------ main
def main():
    print("=== Torque sensor test ===")
    while True:
        try:
            L = parse_length_m(ask_text("Lever arm length (distance from the sensor axis to the hanging weight), e.g. 150mm, 15cm or 0.15m: "))
            if L > 0:
                break
        except ValueError:
            pass
        print("  please type a length such as 150mm")
    print(f"  -> lever arm = {L * 1000:.1f} mm ({L:.4f} m)")
    trials = ask_int("How many trials per weight?", 3)
    baseline, zero_text = ask_zero_method()

    sensor = Sensor()
    out = f"loadcell_test_{datetime.now():%Y%m%d_%H%M%S}.csv"
    cols = ["timestamp", "weight_g", "lever_arm_mm", "trial", "baseline_torque_Nm", "expected_torque_Nm", "reading", "reading_std", "n_samples", "raw_registers", "source", "unit"]
    rows = []
    print(f"\nResults will be saved to: {os.path.abspath(out)}")
    print(f"Zero convention: {zero_text}.")
    print("For each load, type the TOTAL mass of everything that is hanging and was NOT there when you zeroed (for example your water bottle + string, weighed on the scale).")
    try:
        input("Press Enter when you are ready to start ... ")
        while True:
            s = ask_text("\nReal weight of the load now attached (from your scale, e.g. 250 or 0.25kg), or 'done': ").lower()
            if s in ("done", "d", "q", "quit", "exit"):
                break
            try:
                w_g, ambiguous = parse_weight_g(s)
            except ValueError:
                print("  please type a weight such as 250 (grams) or 0.25kg")
                continue
            if w_g < 0:
                print("  the weight cannot be negative")
                continue
            if ambiguous and not yes(f"  You typed {s} with no unit: is that {w_g:g} grams?"):
                print("  please type the weight again, with a unit (for example 0.5kg or 500g)")
                continue
            T = w_g / 1000.0 * G * L + baseline
            print(f"  -> {w_g:g} g at {L * 1000:.0f} mm = {T:.4f} N.m expected" + (f" (includes {baseline:.4f} N.m from the arm and parts)" if baseline else "")
                  + f" ({T / SENSOR_CAPACITY_NM * 100:.0f} % of a {SENSOR_CAPACITY_NM:g} N.m sensor)")
            if T > SENSOR_CAPACITY_NM and not yes("  !!! This is ABOVE the sensor capacity (120 % is the safe overload). Continue anyway?"):
                continue
            sensor.set_torque(T)
            for k in range(1, trials + 1):
                input(f"  Trial {k} of {trials}: attach the load (re-attach it for each trial if you want to test repeatability), then press Enter to record ... ")
                if not SIMULATE:
                    print(f"    settling {SETTLE_S:g} s ...")
                    time.sleep(SETTLE_S)
                res = measure(sensor)
                if res is None:
                    print("    !! no usable reply from the amplifier (check wiring, baud rate, slave ID, register settings).")
                    try:
                        v = float(ask_text(f"    Type the value the amplifier shows in {UNIT} (or just press Enter to skip this trial): ") or "nan")
                    except ValueError:
                        v = float("nan")
                    if math.isnan(v):
                        continue
                    mean, sd, n, raw, source = v, 0.0, 1, "", "typed"
                else:
                    mean, sd, n, raw = res
                    source = "simulated" if SIMULATE else "modbus"
                msg = f"    reading = {mean:.5f} {UNIT}" + (f" (+-{sd:.5f}, {n} samples)" if n > 1 else "")
                if UNIT.lower() in ("nm", "n.m"):
                    msg += f"   expected {T:.5f}   difference {mean - T:+.5f}"
                print(msg)
                row = dict(timestamp=datetime.now().isoformat(timespec="seconds"), weight_g=w_g, lever_arm_mm=round(L * 1000, 3), trial=k,
                           baseline_torque_Nm=round(baseline, 6), expected_torque_Nm=round(T, 6), reading=mean, reading_std=sd, n_samples=n, raw_registers=raw, source=source, unit=UNIT)
                rows.append(row)
                new = not os.path.exists(out)
                with open(out, "a", newline="") as f:                       # written after EVERY trial
                    w = csv.DictWriter(f, fieldnames=cols)
                    if new:
                        w.writeheader()
                    w.writerow(row)
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    finally:
        sensor.close()
    summarise(rows)
    if rows:
        print(f"\n{len(rows)} trial(s) saved to {os.path.abspath(out)}")
    else:
        print("\nNothing was recorded.")


if __name__ == "__main__":
    watch() if WATCH else main()
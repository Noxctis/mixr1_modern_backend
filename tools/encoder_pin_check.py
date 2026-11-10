#!/usr/bin/env python3
"""Live check of the three encoder pins (A, B, index X) on the Raspberry Pi. Tells you in plain words what is dead.

  1. STOP the MIXR-1 daemon (two programs must not use the pins at once).
  2. sudo pigpiod -s 5            (if it is not running)
  3. python3 encoder_pin_check.py              # default pins from config.hpp: A=24 B=23 X=22
     python3 encoder_pin_check.py --seconds 20 --pull up
     python3 encoder_pin_check.py --a 14 --b 9 --x 22      # test other pins by moving the wires (then change config.hpp to match)

It reads the idle levels for 3 s, then counts edges for --seconds while YOU turn the shaft by hand (or spin the motor).
To test the Pi pin itself, with the encoder DISCONNECTED touch a jumper wire from the pin to GND and to 3.3 V (never 5 V!) during the window:
a working pin will count edges. It only READS: it never drives the pins.
"""
import argparse, sys, time, threading

try:
    import pigpio
except ImportError:
    sys.exit("python3-pigpio is missing: sudo apt install -y python3-pigpio")

ap = argparse.ArgumentParser()
ap.add_argument("--a", type=int, default=24); ap.add_argument("--b", type=int, default=23); ap.add_argument("--x", type=int, default=22)
ap.add_argument("--seconds", type=float, default=15.0)
ap.add_argument("--pull", choices=["up", "down", "off", "keep"], default="keep", help="pull resistor on the Pi pins (keep = do not change)")
a = ap.parse_args()

pi = pigpio.pi()
if not pi.connected:
    sys.exit("Cannot reach pigpiod. Start it:  sudo pigpiod -s 5   (and make sure the daemon is stopped)")
pins = {"A": a.a, "B": a.b, "X": a.x}
for n, p in pins.items():
    pi.set_mode(p, pigpio.INPUT)
    if a.pull != "keep":
        pi.set_pull_up_down(p, {"up": pigpio.PUD_UP, "down": pigpio.PUD_DOWN, "off": pigpio.PUD_OFF}[a.pull])

print(f"Pins (BCM GPIO): A={a.a}  B={a.b}  X={a.x}")
print("Phase 1: idle levels for 3 s. Keep the shaft STILL.")
idle = {n: set() for n in pins}
for _ in range(6):
    for n, p in pins.items():
        idle[n].add(pi.read(p))
    time.sleep(0.5)
for n in pins:
    print(f"  {n} (GPIO {pins[n]}): level(s) {sorted(idle[n])}")

edges = {n: 0 for n in pins}; rises = {n: 0 for n in pins}; seen = {n: set(idle[n]) for n in pins}
lock = threading.Lock(); state = {"a": pi.read(a.a), "b": pi.read(a.b)}
valid = [0]; invalid = [0]; lead = {"A leads B": 0, "B leads A": 0}


def cb_for(name):
    def cb(gpio, level, tick):
        if level > 1:
            return
        with lock:
            edges[name] += 1; rises[name] += level; seen[name].add(level)
            if name in ("A", "B"):
                old = (state["a"] << 1) | state["b"]
                state["a" if name == "A" else "b"] = level
                new = (state["a"] << 1) | state["b"]
                if bin(old ^ new).count("1") == 1:
                    valid[0] += 1
                    # sequence 00 -> 10 -> 11 -> 01 -> 00 means A leads B
                    if (old, new) in ((0, 2), (2, 3), (3, 1), (1, 0)):
                        lead["A leads B"] += 1
                    else:
                        lead["B leads A"] += 1
                else:
                    invalid[0] += 1
    return cb


cbs = [pi.callback(p, pigpio.EITHER_EDGE, cb_for(n)) for n, p in pins.items()]
print(f"\nPhase 2: {a.seconds:g} s. NOW TURN THE SHAFT BY HAND (slowly, one direction) or run the motor.")
t_end = time.time() + a.seconds
last = {n: 0 for n in pins}
while time.time() < t_end:
    time.sleep(1.0)
    with lock:
        d = {n: edges[n] - last[n] for n in pins}; last = dict(edges)
    print(f"  edges in the last second:  A {d['A']:5d}   B {d['B']:5d}   X {d['X']:3d}   | levels now A={pi.read(a.a)} B={pi.read(a.b)} X={pi.read(a.x)}")
for c in cbs:
    c.cancel()
final_levels = {n: pi.read(p) for n, p in pins.items()}
pi.stop()

print("\n================ DIAGNOSIS ================")
ok = True
for n in ("A", "B", "X"):
    p = pins[n]
    if edges[n] > 0:
        print(f"{n} (GPIO {p}): ALIVE, {edges[n]} edges.")
    else:
        ok = False
        lvl = final_levels[n]
        if lvl == 1:
            print(f"{n} (GPIO {p}): DEAD, stuck HIGH (3.3 V). Likely: line not connected (the pull-up holds it high), converter channel not powered or not connected, or the encoder output is off.")
        else:
            print(f"{n} (GPIO {p}): DEAD, stuck LOW (0 V). Likely: shorted to ground, converter output pulled low, encoder not powered, or the wire is on the wrong pin.")
if edges["A"] and edges["B"]:
    tot = valid[0] + invalid[0]
    frac = invalid[0] / tot if tot else 0
    print(f"A and B together: {valid[0]} valid steps, {invalid[0]} invalid ({100 * frac:.2f} %). "
          + ("Clean." if frac < 0.001 else "NOISE or too many edges for the pigpio sample rate: check grounding / shielding / slow the shaft down."))
    print(f"Direction seen: {max(lead, key=lead.get)} (A leads B = turning one way, B leads A = the other). Direction flips mid-run mean a very noisy signal.")
    if rises["X"] >= 2:
        cpr = (edges["A"] + edges["B"]) / rises["X"]
        print(f"Index: {rises['X']} pulses. Counts per index pulse = {cpr:.1f}  ->  the encoder is at about {cpr / 4:.0f} PPR (256 PPR expects 1024 counts).")
    elif rises["X"] == 0:
        print("No index pulse seen: turn the shaft a FULL revolution to catch it.")
elif edges["A"] or edges["B"]:
    print("ONLY ONE of A / B works: the speed will read zero and counts will be wrong. Check the dead channel's wire, converter channel and encoder output.")
elif edges["X"]:
    print("ONLY the index works: A and B are both dead. Check the encoder's A and B outputs and their converter channels first (that matches 'I only get something on X').")
else:
    print("NOTHING moves on any pin. Check, in order: encoder 5 V power and ground, shaft really turning, common ground between encoder, converter and Pi, then the wires.")
print("\nNext checks with a multimeter (encoder powered, shaft still): encoder output side should be 0 V or about 5 V, Pi side 0 V or about 3.3 V, and turning slowly must toggle them.")
sys.exit(0 if ok else 1)
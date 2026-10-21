#!/usr/bin/env python3
"""
test_tare.py - test the ATO-LCTR-OAR-DY610 clear (tare) command over RS-485 Modbus-RTU, with the motor OFF and no load on the sensor.

    pip install pyserial
    python test_tare.py --port COM5  --baud 9600 --slave 1          # Windows
    python test_tare.py --port /dev/ttyUSB0 --baud 9600 --slave 1   # Linux/Mac
    python test_tare.py --port COM5 --baud 9600 --read-only         # only check that reading works (does NOT zero anything)
    python test_tare.py --selftest                                  # no hardware: runs against a simulated amplifier

Steps: 1) read the net value 5x (shows the offset), 2) send CLEAR  01 05 00 00 FF 00 CRC, expect the same 8 bytes echoed,
       3) wait, read 5x again, 4) PASS if the mean is within --tol raw counts of 0.
Raw values are integers; the real value = raw / 10**decimals (the amplifier's decimal-point setting, 'Sd'), use --decimals.
"""
import argparse, statistics, struct, sys, time

def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])            # low byte first

hexs = lambda b: b.hex(" ").upper() if b else "(nothing)"

def xfer(ser, body: bytes, nresp: int) -> bytes:
    frame = body + crc16(body)
    ser.reset_input_buffer(); time.sleep(0.02)       # >3.5 character times of silence before a Modbus frame
    ser.write(frame)
    resp = ser.read(nresp)
    print(f"   TX {hexs(frame)}\n   RX {hexs(resp)}")
    return resp

def read_net(ser, slave, decimals):
    """Read net weight/torque from address 0 (2 registers = signed 32-bit, per Table 2). Returns (raw, scaled)."""
    r = xfer(ser, bytes([slave, 0x03, 0, 0, 0, 2]), 9)
    if len(r) < 5: raise RuntimeError("no / short reply to the read command")
    if r[-2:] != crc16(r[:-2]): raise RuntimeError("bad CRC in the reply")
    if r[0] != slave or r[1] != 0x03: raise RuntimeError("reply is not a read response (wrong address/function)")
    n = r[2]; data = r[3:3 + n]
    raw = struct.unpack(">i", data)[0] if n == 4 else struct.unpack(">h", data)[0] if n == 2 else None
    if raw is None: raise RuntimeError(f"unexpected byte count {n}")
    return raw, raw / 10 ** decimals

def sniff_active_upload(ser):
    ser.reset_input_buffer(); time.sleep(0.5); junk = ser.read(ser.in_waiting or 0)
    if junk and b"\r" in junk and any(48 <= c <= 57 for c in junk):
        print("!! The amplifier is streaming ASCII text by itself (active-upload mode) and will not answer Modbus.\n"
              f"   Seen: {junk[:40]!r}\n   Set the communication mode (CP on the K1 menu) to Modbus, then retry.")
        return True
    return False

def mean_reads(ser, slave, decimals, n=5):
    vals = []
    for _ in range(n):
        raw, sc = read_net(ser, slave, decimals); vals.append(raw); time.sleep(0.15)
    return statistics.mean(vals), (statistics.stdev(vals) if n > 1 else 0.0)

def run(ser, a):
    ok = True
    if sniff_active_upload(ser): return False
    print("\n[1] reading before the clear")
    try: m0, s0 = mean_reads(ser, a.slave, a.decimals)
    except Exception as e:
        print("   FAIL:", e)
        print("   If the message mentions 'multiple access' or 'readiness to read': another process has the port open (see: sudo fuser -v <port>), or the USB-RS485 adapter dropped.")
        return False
    print(f"   mean {m0:.2f} counts (= {m0 / 10 ** a.decimals:g}), noise (std) {s0:.2f} counts")
    if a.read_only:
        print("\nread-only test finished: communication works."); return True
    if a.method == "clear05":
        print("\n[2] sending CLEAR (function 05)")
        frame = bytes([a.slave, 0x05, 0x00, 0x00, 0xFF, 0x00]); frame += crc16(frame); expect = frame
        resp = xfer(ser, frame[:-2], 8)
    else:
        print("\n[2] sending WRITE-ZERO (function 10: value 0 to registers 0-1)")
        frame = bytes([a.slave, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0, 0, 0, 0]); frame += crc16(frame)
        expect = bytes([a.slave, 0x10, 0x00, 0x00, 0x00, 0x02]); expect += crc16(expect)
        resp = xfer(ser, frame[:-2], 8)
    if resp == expect: print("   PASS: the reply is the expected one")
    else: print(f"   FAIL: unexpected reply, expected {hexs(expect)} (check slave address, baud rate, wiring A/B, CRC)"); ok = False
    print(f"\n[3] waiting {a.settle:g} s, then reading again")
    time.sleep(a.settle)
    try: m1, s1 = mean_reads(ser, a.slave, a.decimals)
    except Exception as e: print("   FAIL:", e); return False
    print(f"   mean {m1:.2f} counts (= {m1 / 10 ** a.decimals:g}), noise (std) {s1:.2f} counts")
    if abs(m1) <= a.tol: print(f"   PASS: reading is within +-{a.tol:g} counts of zero")
    else: print(f"   FAIL: reading is {m1:.2f} counts from zero (tolerance {a.tol:g}); raise --settle or --tol, or check the filter setting"); ok = False
    print("\nRESULT:", "PASS - the tare works" if ok else "FAIL")
    return ok

# ------------------------------------------------------------------ simulated amplifier for --selftest
class FakePort:
    def __init__(self): self.offset = 600; self.buf = b""; self.t = 0
    def reset_input_buffer(self): self.buf = b""
    @property
    def in_waiting(self): return 0
    def write(self, f):
        assert f[-2:] == crc16(f[:-2]), "bad CRC sent"
        if f[1] == 0x05: self.offset = 0; self.buf = f                                    # clear -> echo
        elif f[1] == 0x10: self.offset = 0; body = f[:6]; self.buf = body + crc16(body)
        elif f[1] == 0x03: self.t += 1; body = bytes([f[0], 3, 4]) + struct.pack(">i", self.offset + (self.t % 3) - 1); self.buf = body + crc16(body)
    def read(self, n): r, self.buf = self.buf[:n], self.buf[n:]; return r

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port"); p.add_argument("--baud", type=int, default=9600); p.add_argument("--slave", type=int, default=1)
    p.add_argument("--decimals", type=int, default=3, help="amplifier decimal-point setting (value = raw / 10**decimals)")
    p.add_argument("--settle", type=float, default=2.0); p.add_argument("--tol", type=float, default=2.0, help="allowed distance from zero after the clear, in raw counts")
    p.add_argument("--method", choices=["clear05", "write10"], default="clear05", help="clear05: 01 05 00 00 FF 00 ; write10: 01 10 00 00 00 02 04 00 00 00 00")
    p.add_argument("--read-only", action="store_true"); p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest: ser = FakePort(); a.settle = 0.05
    else:
        if not a.port: sys.exit("give --port (or --selftest)")
        import serial
        try:
            ser = serial.Serial(a.port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=0.5, exclusive=True)   # n81 = the default format
        except (serial.SerialException, ValueError) as e:
            sys.exit(f"Cannot open {a.port} exclusively: {e}\nAnother program is probably using it (your backend/logger?). Stop it, then retry.\nCheck with:  sudo fuser -v {a.port}")
    sys.exit(0 if run(ser, a) else 1)
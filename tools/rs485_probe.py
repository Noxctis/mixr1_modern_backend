#!/usr/bin/env python3
"""
Find out how the DY610 load cell amplifier is really configured, independent of the C++ daemon.

  sudo apt install python3-serial
  python3 rs485_probe.py --loopback            # 1) short the Pi TX and RX pins together: proves the Pi UART itself works
  python3 rs485_probe.py                       # 2) scan baud rates x slave IDs x 1/2 registers, and listen for 'active upload' text
  python3 rs485_probe.py --port /dev/ttyAMA3 --ids 1-128

Make sure the motor daemon is NOT running (nothing else may have the port open).
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: sudo apt install python3-serial")

BAUDS = [9600, 19200, 4800, 2400, 38400, 57600, 115200]


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])          # low byte first, as Modbus RTU sends it


# Self-check against the well-known Modbus test frame: 01 03 00 00 00 01 -> CRC 84 0A
assert crc16(bytes.fromhex("01 03 00 00 00 01")) == bytes.fromhex("84 0A"), "CRC routine is wrong"


def parse_ids(text: str):
    ids = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            ids += range(int(a), int(b) + 1)
        else:
            ids.append(int(part))
    return ids


def open_port(port, baud, stopbits=1):
    return serial.Serial(port, baud, bytesize=8, parity="N", stopbits=stopbits, timeout=0.05)


def loopback(port):
    print(f"Loopback test on {port}: connect the Pi TX pin to the Pi RX pin with a wire (converter disconnected).")
    ser = open_port(port, 9600)
    ser.reset_input_buffer()
    msg = bytes(range(32))
    ser.write(msg)
    ser.flush()
    time.sleep(0.2)
    back = ser.read(64)
    ser.close()
    if back == msg:
        print("PASS: the Pi UART works (port name, serial login console and pin function are fine).")
        return 0
    print(f"FAIL: sent {len(msg)} bytes, got back {len(back)}.")
    print("  * is the wire really between the TX and RX pins of the SAME UART?")
    print("  * serial login console still on?  sudo raspi-config -> Interface -> Serial: login shell NO, hardware YES, then reboot")
    print("  * wrong port name?  ls -l /dev/serial0 /dev/ttyAMA* /dev/ttyS0")
    return 1


def query(ser, sid, count):
    req = bytes([sid, 0x03, 0x00, 0x00, 0x00, count])
    req += crc16(req)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    want = 5 + 2 * count
    deadline = time.time() + 0.12
    buf = b""
    while time.time() < deadline and len(buf) < want:
        buf += ser.read(want - len(buf))
    if len(buf) != want or buf[0] != sid or buf[1] != 0x03 or buf[2] != 2 * count or crc16(buf[:-2]) != buf[-2:]:
        return None
    return buf[3:-2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--ids", default="1-8", help="slave IDs to try, e.g. 1-8 or 1-128 or 1,5")
    ap.add_argument("--baud", type=int, help="only try this baud rate")
    ap.add_argument("--loopback", action="store_true")
    args = ap.parse_args()

    if args.loopback:
        return loopback(args.port)

    ids = parse_ids(args.ids)
    bauds = [args.baud] if args.baud else BAUDS
    found = []
    print(f"Scanning {args.port}: bauds {bauds}, slave IDs {ids[0]}..{ids[-1]}")
    for baud in bauds:
        ser = open_port(args.port, baud)
        # 1) Passive listen: an amplifier in 'active upload' mode streams ASCII text like '  123.456\r' all the time and ignores Modbus requests.
        ser.reset_input_buffer()
        time.sleep(0.6)
        spont = ser.read(200)
        if spont:
            print(f"  {baud:>6} baud: the line is NOT silent, received {len(spont)} bytes: {spont[:40]!r}")
            if sum(32 <= c < 127 or c in (10, 13) for c in spont) > 0.8 * len(spont):
                print("           -> readable text: the amplifier is probably in ACTIVE UPLOAD mode at this baud rate (CP parameter).")
                print("              Either read that text directly, or set CP so that it is Modbus (the two ATO manuals disagree on 0/1: try both).")
        # 2) Modbus function 0x03, register 0 (net weight), as 1 register (user manual) and as 2 registers (protocol manual)
        for sid in ids:
            for count in (1, 2):
                data = query(ser, sid, count)
                if data is not None:
                    raw = int.from_bytes(data, "big", signed=True)
                    print(f"  >>> ANSWER at {baud} baud, slave {sid}, {count} register(s): raw bytes {data.hex(' ')}  ->  signed value {raw}")
                    found.append((baud, sid, count, raw))
        ser.close()

    print()
    if not found:
        print("No Modbus answer at any baud / ID.  What that means:")
        print("  * run --loopback first: if that fails, the problem is the Pi UART, not the amplifier")
        print("  * Converter direction control: a bare MAX485 board has DE/RE pins that must be driven; only 'auto direction' converters work on a plain UART")
        print("  * A/B swapped: swap the two RS485 wires at the amplifier (harmless to try)")
        print("  * the amplifier needs its own 12-30 V supply and a common ground with the converter")
        print("  * if the line was NOT silent above, it is in active-upload mode: change the CP setting on the amplifier keys")
        print("  * the amplifier could be at slave ID > 8: rerun with --ids 1-128")
        return 1
    b, s, c, v = found[0]
    print("Use in the daemon:")
    print(f"  --torque --torque-port={args.port} --torque-baud={b} --torque-slave={s} --torque-reg=0 --torque-regcount={c} "
          f"--torque-format={'int16' if c == 1 else 'int32'}")
    print("Then set --torque-scale from the amplifier's decimal point setting (e.g. 0.001 for 3 decimals) and zero it with the SL key.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
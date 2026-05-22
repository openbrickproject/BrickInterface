#!/usr/bin/env python3
"""
BrickInterface integration test (host side).

Cycles 8 states, 2 seconds each, mirroring the firmware-only test we ran
on the device:

    state 0: IFA OUT0 ON + PF channel 1 blue forward (full)
    state 1: all off + PF blue float
    state 2: IFA OUT1 ON + PF channel 1 blue reverse (full)
    state 3: all off + PF blue float
    state 4: IFA OUT2 ON + PF channel 1 red forward (full)
    state 5: all off + PF red float
    state 6: IFA OUT3 ON + PF channel 1 red reverse (full)
    state 7: all off + PF red float

Usage: python3 dance.py [--port /dev/cu.usbmodem...] [--once]
"""
from __future__ import annotations

import argparse
import glob
import os
import signal
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("This script requires pyserial. Install with: pip install pyserial")


# --- Protocol constants (mirror firmware/BrickInterface/protocol.h) ---
PROTO_SOF = 0xAA

CMD_IFACE_SET_OUTPUTS = 0x10
CMD_PF_SEND = 0x20
CMD_IR_ABORT_ALL = 0x40

PF_MODE_SINGLE_PWM = 0x01

REPLY_OK = 0x84
REPLY_IR_ACCEPTED = 0xA0
REPLY_IR_DONE = 0xA1
REPLY_ERROR = 0xE0

# PF data byte (Single Output PWM mode):
#   bit 4    = output select (0 = A/red, 1 = B/blue)
#   bits 3-0 = PWM step (0 = float, 1-7 = forward, 8 = brake, 9-15 = reverse)
PF_BLUE_FWD7 = 0x17
PF_BLUE_REV7 = 0x19
PF_BLUE_FLT  = 0x10
PF_RED_FWD7  = 0x07
PF_RED_REV7  = 0x09
PF_RED_FLT   = 0x00


def build_packet(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    length = 2 + len(payload)
    chk = length ^ seq ^ cmd
    for b in payload:
        chk ^= b
    return bytes([PROTO_SOF, length, seq, cmd]) + payload + bytes([chk])


def parse_reply(buf: bytearray) -> tuple[dict | None, int]:
    """Try to parse one packet from the front of `buf`. Returns (packet, bytes_consumed)."""
    if len(buf) < 5:
        return None, 0
    if buf[0] != PROTO_SOF:
        return None, 1  # resync
    length = buf[1]
    total = 1 + 1 + length + 1
    if len(buf) < total:
        return None, 0
    seq = buf[2]
    cmd = buf[3]
    payload = bytes(buf[4 : 4 + length - 2])
    expected_chk = length ^ seq ^ cmd
    for b in payload:
        expected_chk ^= b
    actual_chk = buf[4 + length - 2]
    if expected_chk != actual_chk:
        return None, 1  # bad checksum, resync
    return {"seq": seq, "cmd": cmd, "payload": payload}, total


def find_port() -> str:
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not candidates:
        sys.exit("No /dev/cu.usbmodem* found. Plug in the board and try again.")
    if len(candidates) > 1:
        print(f"Multiple devices found, using {candidates[0]}")
        for c in candidates[1:]:
            print(f"  (also: {c})")
    return candidates[0]


class Brick:
    def __init__(self, port: str):
        # Baud rate is meaningless over USB CDC, but pyserial requires one.
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.seq = 0
        self.rxbuf = bytearray()

    def close(self):
        self.ser.close()

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        return self.seq

    def send(self, cmd: int, payload: bytes = b"") -> None:
        pkt = build_packet(self._next_seq(), cmd, payload)
        self.ser.write(pkt)

    def drain_replies(self) -> list[dict]:
        """Read whatever's available and parse complete packets."""
        data = self.ser.read(256)
        if data:
            self.rxbuf.extend(data)
        out = []
        while True:
            pkt, n = parse_reply(self.rxbuf)
            if pkt is not None:
                out.append(pkt)
                del self.rxbuf[:n]
            elif n > 0:
                # resync — drop one byte and keep going
                del self.rxbuf[:n]
            else:
                break
        return out

    def iface_set(self, duties: list[int]) -> None:
        assert len(duties) == 6
        self.send(CMD_IFACE_SET_OUTPUTS, bytes(duties))

    def pf_send(self, channel: int, mode: int, data: int, flags: int = 0) -> None:
        self.send(CMD_PF_SEND, bytes([channel, mode, data, flags]))

    def ir_abort(self) -> None:
        self.send(CMD_IR_ABORT_ALL)


STATES = [
    ("OUT0 + blue fwd",  [255, 0, 0, 0, 0, 0], PF_BLUE_FWD7),
    ("off  + blue float",[  0, 0, 0, 0, 0, 0], PF_BLUE_FLT),
    ("OUT1 + blue rev",  [  0,255, 0, 0, 0, 0], PF_BLUE_REV7),
    ("off  + blue float",[  0, 0, 0, 0, 0, 0], PF_BLUE_FLT),
    ("OUT2 + red  fwd",  [  0, 0,255, 0, 0, 0], PF_RED_FWD7),
    ("off  + red  float",[  0, 0, 0, 0, 0, 0], PF_RED_FLT),
    ("OUT3 + red  rev",  [  0, 0, 0,255, 0, 0], PF_RED_REV7),
    ("off  + red  float",[  0, 0, 0, 0, 0, 0], PF_RED_FLT),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--port", help="Serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--once", action="store_true", help="One pass through the cycle, then exit")
    ap.add_argument("--dwell", type=float, default=2.0, help="Seconds per state (default 2.0)")
    args = ap.parse_args()

    port = args.port or find_port()
    print(f"Opening {port}")
    brick = Brick(port)
    # Give the device a moment to settle (some USB-CDC stacks reset on open)
    time.sleep(0.3)

    def cleanup(*_):
        print("\nStopping — clearing outputs.")
        try:
            brick.iface_set([0, 0, 0, 0, 0, 0])
            brick.ir_abort()
            time.sleep(0.1)
        finally:
            brick.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    cycle = 0
    while True:
        for label, duties, pf_data in STATES:
            print(f"  {label}")
            brick.iface_set(duties)
            brick.pf_send(0, PF_MODE_SINGLE_PWM, pf_data)

            # Pump replies during the dwell so the OS buffer doesn't fill up
            end = time.monotonic() + args.dwell
            while time.monotonic() < end:
                for r in brick.drain_replies():
                    if r["cmd"] == REPLY_ERROR:
                        print(f"    !! ERROR seq={r['seq']:#x} payload={r['payload'].hex()}")
                time.sleep(0.02)
        cycle += 1
        if args.once:
            break
    cleanup()


if __name__ == "__main__":
    sys.exit(main())

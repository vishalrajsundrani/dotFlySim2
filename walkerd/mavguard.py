#!/usr/bin/env python3
"""
mavguard — a MAVLink relay that can make QGroundControl read-only.

WHY THIS EXISTS
===============
QGroundControl and a C++ mission both want to command the same aircraft. A
warning in the UI is not a lock: an operator who clicks Disarm while a mission
is flying gets a disarmed aircraft, whatever the banner said.

So the lock is physical. PX4 talks to QGC over UDP, and this sits in that gap:

        telemetry, always
    PX4 :14541  ───────────────────────────────────────►  QGC :14550
                ◄───────────────────────────────────────
                     commands, only while unlocked

While a project or a replay holds the flight lock, every message that could
change what the aircraft is doing is DROPPED on the way to PX4. Telemetry is
never touched, so a locked QGC remains a complete instrument panel -- which is
the point: you can watch a mission fly on the map without being able to
interfere with it.

WHAT IS BLOCKED, AND WHY EACH ONE
=================================
  COMMAND_LONG / COMMAND_INT     arm, disarm, takeoff, land, RTL, calibration
  SET_MODE                       the direct "put the vehicle in mode X" path
  MANUAL_CONTROL                 virtual joystick
  RC_CHANNELS_OVERRIDE           synthetic stick input
  SET_POSITION_TARGET_*          offboard setpoints from a second source
  SET_ATTITUDE_TARGET            the same, one level lower
  MISSION_* (the write side)     uploading or clearing a flight plan
  PARAM_SET                      changing a parameter mid-flight

Mission READ traffic is allowed: QGC downloading the current plan to display it
changes nothing.

FRAMES ARE FORWARDED BYTE FOR BYTE
==================================
The only decision made here is forward-or-drop, on the message id. Nothing is
re-encoded, so MAVLink v2 signing, sequence numbers and CRCs are untouched --
a relay that rewrote frames would have to re-sign them, and getting that subtly
wrong produces a link that works until someone enables signing.

A datagram may carry several frames. Each is judged separately and the allowed
ones are re-packed, so blocking one command does not drop the telemetry sharing
its packet.
"""

from __future__ import annotations

import os
import socket
import struct
import threading
import time

# MAVLink message ids. Names kept beside the numbers because a bare integer
# here would be unreviewable.
BLOCKED_WHEN_LOCKED = {
    11: "SET_MODE",
    23: "PARAM_SET",
    38: "MISSION_WRITE_PARTIAL_LIST",
    39: "MISSION_ITEM",
    41: "MISSION_SET_CURRENT",
    44: "MISSION_COUNT",
    45: "MISSION_CLEAR_ALL",
    69: "MANUAL_CONTROL",
    70: "RC_CHANNELS_OVERRIDE",
    73: "MISSION_ITEM_INT",
    75: "COMMAND_INT",
    76: "COMMAND_LONG",
    82: "SET_ATTITUDE_TARGET",
    84: "SET_POSITION_TARGET_LOCAL_NED",
    86: "SET_POSITION_TARGET_GLOBAL_INT",
}

MAGIC_V1, MAGIC_V2 = 0xFE, 0xFD
HDR_V1, HDR_V2 = 6, 10
CRC = 2
SIGNATURE = 13


def split_frames(buf: bytes):
    """
    Yield (frame_bytes, msgid) for each complete MAVLink frame in a datagram.

    v1: FE | len | seq | sys | comp | msgid(1)  ... payload ... crc(2)
    v2: FD | len | inc | cmp | seq | sys | comp | msgid(3 LE) ... crc(2) [sig]
    """
    i, n = 0, len(buf)
    while i < n:
        magic = buf[i]
        if magic == MAGIC_V1:
            if i + HDR_V1 > n:
                break
            length = buf[i + 1]
            total = HDR_V1 + length + CRC
            if i + total > n:
                break
            yield buf[i:i + total], buf[i + 5]
            i += total
        elif magic == MAGIC_V2:
            if i + HDR_V2 > n:
                break
            length = buf[i + 1]
            incompat = buf[i + 2]
            total = HDR_V2 + length + CRC + (SIGNATURE if incompat & 0x01 else 0)
            if i + total > n:
                break
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            yield buf[i:i + total], msgid
            i += total
        else:
            # Not a frame boundary: skip a byte and resynchronise. Garbage in a
            # UDP datagram is not worth dropping the whole packet for.
            i += 1


class MavGuard:
    """
    Relay between PX4 and QGC. Thread-safe; `locked` may change at any time.

    Ports, with PX4's own defaults in mind:
        px4_port    where PX4 listens for GCS traffic          (14541)
        listen      where PX4 SENDS, i.e. where we receive     (14551)
        gcs_port    where QGC listens                          (14550)
    """

    def __init__(self, listen: int = 14551, px4_port: int = 14541,
                 gcs_port: int = 14550, host: str = "127.0.0.1"):
        self.host = host
        self.listen_port = listen
        self.px4_port = px4_port
        self.gcs_port = gcs_port

        self._locked = False
        self._holder = ""
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.blocked_counts: dict[str, int] = {}
        self.to_gcs = 0
        self.to_px4 = 0
        self.last_block = 0.0

        self._sock: socket.socket | None = None
        self._gcs_addr: tuple[str, int] | None = None

    # ── the lock ─────────────────────────────────────────────────────────────

    def set_locked(self, locked: bool, holder: str = "") -> None:
        with self._lock:
            self._locked = bool(locked)
            self._holder = holder if locked else ""

    @property
    def locked(self) -> bool:
        with self._lock:
            return self._locked

    def status(self) -> dict:
        with self._lock:
            return {
                "locked": self._locked,
                "holder": self._holder,
                "blocked": dict(self.blocked_counts),
                "blocked_total": sum(self.blocked_counts.values()),
                "to_gcs": self.to_gcs,
                "to_px4": self.to_px4,
                "last_block": self.last_block,
            }

    # ── running ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.listen_port))
        self._sock.settimeout(0.5)
        threading.Thread(target=self._pump, daemon=True, name="mavguard").start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def _pump(self) -> None:
        px4 = (self.host, self.px4_port)
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            from_px4 = addr[1] == self.px4_port
            if from_px4:
                # Telemetry upward: never filtered, never delayed.
                self._sock.sendto(data, (self.host, self.gcs_port))
                self.to_gcs += 1
                continue

            # Anything else is the GCS. Remember where it lives so a reply can
            # be addressed even if QGC bound an ephemeral port.
            self._gcs_addr = addr
            self._sock.sendto(self._filter(data), px4)
            self.to_px4 += 1

    def _filter(self, data: bytes) -> bytes:
        if not self.locked:
            return data
        kept = bytearray()
        for frame, msgid in split_frames(data):
            name = BLOCKED_WHEN_LOCKED.get(msgid)
            if name is None:
                kept += frame
                continue
            with self._lock:
                self.blocked_counts[name] = self.blocked_counts.get(name, 0) + 1
                self.last_block = time.time()
        return bytes(kept)


def main() -> int:                                   # pragma: no cover
    import argparse
    ap = argparse.ArgumentParser(prog="mavguard")
    ap.add_argument("--listen", type=int, default=14551)
    ap.add_argument("--px4", type=int, default=14541)
    ap.add_argument("--gcs", type=int, default=14550)
    a = ap.parse_args()
    g = MavGuard(a.listen, a.px4, a.gcs)
    g.start()
    print(f"mavguard: PX4 :{a.px4} -> :{a.listen} -> QGC :{a.gcs}", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        g.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

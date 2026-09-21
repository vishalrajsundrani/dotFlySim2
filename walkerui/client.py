"""
Talking to walkerd.

One connection, two directions: synchronous request/response for anything
walker asks, and an unsolicited event stream walkerd pushes. Both share the
socket, so a reply and an event can interleave -- the reader thread sorts them
by whether they carry an `id`.

WHY WALKER NEVER POLLS
======================
The dashboard refreshes twice a second. If each refresh asked walkerd for
state, the answer would always be a little stale and the cost would scale with
how much is running. Instead walkerd pushes a `unit` event the instant a state
changes, and walker keeps a local mirror. `state()` exists for the first paint
and for resynchronising after a reconnect, not for the steady state.
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
from typing import Callable


class WalkerdError(RuntimeError):
    pass


class Client:
    def __init__(self, path: str, on_event: Callable[[dict], None] | None = None):
        self.path = str(path)
        self.on_event = on_event
        self._sock: socket.socket | None = None
        self._pending: dict[int, queue.Queue] = {}
        self._next_id = 1
        self._lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._closed = threading.Event()

    # ── connection ───────────────────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> None:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            s.connect(self.path)
        except OSError as e:
            raise WalkerdError(
                f"walkerd is not answering on {self.path} ({e}).\n"
                f"  It runs inside the container; start it with:  walker up") from e
        s.settimeout(None)
        self._sock = s
        self._closed.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="walkerd-reader")
        self._reader.start()

    @property
    def connected(self) -> bool:
        return self._sock is not None and not self._closed.is_set()

    def close(self) -> None:
        self._closed.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # ── traffic ──────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        buf = b""
        while not self._closed.is_set():
            try:
                chunk = self._sock.recv(65536)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                rid = msg.get("id")
                if rid is not None and rid in self._pending:
                    self._pending.pop(rid).put(msg)
                elif "ev" in msg and self.on_event:
                    try:
                        self.on_event(msg)
                    except Exception:
                        pass
        self._closed.set()

    def call(self, op: str, timeout: float = 30.0, **kw) -> dict:
        if not self.connected:
            raise WalkerdError("not connected to walkerd")
        with self._lock:
            rid = self._next_id
            self._next_id += 1
        box: queue.Queue = queue.Queue(maxsize=1)
        self._pending[rid] = box
        payload = json.dumps({"id": rid, "op": op, **kw}) + "\n"
        try:
            self._sock.sendall(payload.encode())
        except OSError as e:
            self._pending.pop(rid, None)
            raise WalkerdError(f"walkerd went away: {e}") from e
        try:
            return box.get(timeout=timeout)
        except queue.Empty:
            self._pending.pop(rid, None)
            raise WalkerdError(f"walkerd did not answer '{op}' within {timeout:.0f}s")

    # ── the operations walker uses ───────────────────────────────────────────

    def ping(self) -> float:
        """Round-trip time in milliseconds."""
        t0 = time.perf_counter()
        self.call("ping", timeout=5)
        return (time.perf_counter() - t0) * 1000.0

    def state(self) -> dict:
        return self.call("state")

    def start(self, unit: str, **args) -> dict:
        return self.call("start", unit=unit, args=args, timeout=60)

    def stop(self, unit: str) -> dict:
        return self.call("stop", unit=unit, timeout=60)

    def select(self, **args) -> dict:
        return self.call("select", args=args)

    def probe(self, window: float = 2.0, wrapper: bool = True) -> dict:
        return self.call("probe", window=window, wrapper=wrapper,
                         timeout=window + 30)

    def logs(self, unit: str, lines: int = 200) -> list[str]:
        return self.call("logs", unit=unit, lines=lines).get("lines", [])

    def constraints(self) -> dict:
        return self.call("constraints")

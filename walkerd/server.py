"""
walkerd's wire protocol: newline-delimited JSON over a unix socket.

WHY A SOCKET AND NOT `docker exec`
==================================
Measured on this stack: a `docker exec` costs ~150 ms of daemon round-trip
before the process even starts, and a `ros2` CLI call inside it costs another
4-5 s. Walker refreshes its dashboard twice a second; built on `docker exec` it
would be permanently one refresh behind and would spend most of its life
waiting. Over this socket a full state snapshot costs well under a millisecond,
and walkerd PUSHES changes rather than being polled for them.

THE SHAPE
=========
Requests are objects with an `op` and an `id`; every request gets exactly one
response carrying the same `id`. Events (`ev`) arrive unsolicited, at any time,
on every connected client.

    -> {"id": 7, "op": "start", "unit": "sim", "args": {...}}
    <- {"id": 7, "ok": true}
    <- {"ev": "unit", "name": "sim", "state": "starting", ...}
    <- {"ev": "log", "unit": "sim", "text": "[sim] 1/4 starting Gazebo"}

A refusal is a normal response, not an error, and it always carries the reason
and the way out:

    <- {"id": 7, "ok": false, "error": "refused",
        "reason": "a replay owns the wrapper surface",
        "held_by": "replay", "hint": "stop the replay first (R)"}
"""

from __future__ import annotations

import json
import os
import socket
import threading
import traceback
from typing import Callable


class Server:
    def __init__(self, path: str, handle: Callable[[dict], dict]):
        self.path = path
        self.handle = handle
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._stop = threading.Event()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        # A socket left by a previous walkerd would make bind() fail with
        # EADDRINUSE even though nothing is listening.
        if os.path.exists(self.path):
            os.unlink(self.path)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(self.path)
        self._sock.listen(16)
        # The socket is on a bind mount shared with the host user, who is a
        # different uid inside the container. 0o666 keeps walker able to talk
        # to it without running anything as root.
        os.chmod(self.path, 0o666)
        threading.Thread(target=self._accept_loop, daemon=True, name="accept").start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        if os.path.exists(self.path):
            try:
                os.unlink(self.path)
            except OSError:
                pass

    # ── clients ──────────────────────────────────────────────────────────────

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except OSError:
                return
            with self._lock:
                self._clients.append(conn)
            threading.Thread(target=self._client_loop, args=(conn,),
                             daemon=True, name="client").start()

    def _client_loop(self, conn: socket.socket) -> None:
        buf = b""
        try:
            while not self._stop.is_set():
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        req = json.loads(line)
                    except ValueError:
                        self._send(conn, {"ok": False, "error": "bad json"})
                        continue
                    try:
                        resp = self.handle(req)
                    except Exception as e:
                        resp = {"ok": False, "error": "internal",
                                "reason": f"{type(e).__name__}: {e}",
                                "trace": traceback.format_exc()[-800:]}
                    resp.setdefault("id", req.get("id"))
                    # An attach subscribes this connection to a unit's output;
                    # the handler says so and the pump below does the work.
                    self._send(conn, resp)
        except OSError:
            pass
        finally:
            with self._lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass

    # ── output ───────────────────────────────────────────────────────────────

    @staticmethod
    def _send(conn: socket.socket, obj: dict) -> None:
        try:
            conn.sendall((json.dumps(obj) + "\n").encode())
        except OSError:
            pass

    def broadcast(self, event: dict) -> None:
        """
        Push an event to every client.

        A dead or wedged client must never block walkerd -- a terminal window
        that was closed without draining its socket is an ordinary occurrence,
        not an emergency -- so a send failure simply drops that client.
        """
        payload = (json.dumps(event) + "\n").encode()
        with self._lock:
            dead = []
            for c in self._clients:
                try:
                    c.sendall(payload)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                try:
                    c.close()
                except OSError:
                    pass

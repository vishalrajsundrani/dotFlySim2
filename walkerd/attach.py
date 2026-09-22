#!/usr/bin/env python3
"""
walker-attach — give this terminal a unit's console.

    walker-attach sim          PX4's pxh prompt, live
    walker-attach project      the running mission's output
    walker-attach bridge --read-only

WHAT ATTACHING IS AND IS NOT
============================
Attaching is a VIEW. The unit belongs to walkerd, which started it and will
stop it; closing this window detaches and leaves the unit running. That is the
property that makes per-unit terminals safe to open and close freely, and it is
why walker can offer one for any unit at any time.

Keystrokes go to the unit's pty, so `walker-attach sim` is a real PX4 console:
`commander status`, `param show MPC_THR_HOVER`, `listener sensor_combined` all
work, exactly as if you had started PX4 by hand. --read-only withholds that for
a unit you only want to watch.

Ctrl-\\ detaches. Ctrl-C is FORWARDED to the unit rather than killing this
viewer, because in a console that is what it means -- and a mission is meant to
be interruptible from its own terminal.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import socket
import sys
import termios
import tty

SOCKET = os.environ.get("WALKERD_SOCKET", "/run/walker/walkerd.sock")
DETACH_KEY = b"\x1c"          # Ctrl-\


def _send(sock: socket.socket, obj: dict) -> None:
    sock.sendall((json.dumps(obj) + "\n").encode())


def main() -> int:
    ap = argparse.ArgumentParser(prog="walker-attach")
    ap.add_argument("unit")
    ap.add_argument("--read-only", action="store_true",
                    help="watch without being able to type into the unit")
    ap.add_argument("--replay", type=int, default=200,
                    help="lines of history to show first (0 for none)")
    a = ap.parse_args()

    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(SOCKET)
    except OSError as e:
        print(f"walker-attach: walkerd is not answering on {SOCKET} ({e}).\n"
              f"  It is started with `walker up` from the host.", file=sys.stderr)
        return 69

    # History first, so an attached window is never blank. A terminal that
    # opens empty looks broken even when the unit is healthy and simply quiet.
    if a.replay > 0:
        _send(sock, {"id": 1, "op": "logs", "unit": a.unit, "lines": a.replay})

    _send(sock, {"id": 2, "op": "attach", "unit": a.unit})

    # Tell the unit how big this window is, so anything that draws (PX4's
    # prompt, a curses tool) wraps correctly.
    try:
        cols, rows = os.get_terminal_size()
        _send(sock, {"id": 3, "op": "resize", "unit": a.unit,
                     "rows": rows, "cols": cols})
    except OSError:
        pass

    interactive = sys.stdin.isatty() and not a.read_only
    old_term = None
    if interactive:
        old_term = termios.tcgetattr(sys.stdin)
        # Raw mode: every keystroke goes straight through, so the unit's own
        # line editing and control characters behave as they would natively.
        tty.setraw(sys.stdin.fileno())

    banner = (f"\r\n-- attached to '{a.unit}'. Ctrl-\\ detaches"
              f"{'' if interactive else ' (read-only)'}. "
              f"Closing this window leaves the unit running. --\r\n")
    sys.stdout.write(banner)
    sys.stdout.flush()

    def on_winch(_sig, _frm):
        try:
            cols, rows = os.get_terminal_size()
            _send(sock, {"id": 4, "op": "resize", "unit": a.unit,
                         "rows": rows, "cols": cols})
        except OSError:
            pass

    if interactive:
        signal.signal(signal.SIGWINCH, on_winch)

    buf = b""
    try:
        while True:
            watch = [sock] + ([sys.stdin] if interactive else [])
            ready, _, _ = select.select(watch, [], [], 0.5)

            if sys.stdin in ready:
                data = os.read(sys.stdin.fileno(), 4096)
                if DETACH_KEY in data:
                    break
                _send(sock, {"id": 5, "op": "input", "unit": a.unit,
                             "data": data.decode("utf-8", "replace")})

            if sock in ready:
                chunk = sock.recv(65536)
                if not chunk:
                    sys.stdout.write("\r\n-- walkerd closed the connection --\r\n")
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
                    # History reply.
                    if msg.get("id") == 1 and msg.get("lines"):
                        for l in msg["lines"]:
                            sys.stdout.write(l + "\r\n")
                    # Live output for our unit.
                    elif msg.get("ev") == "output" and msg.get("unit") == a.unit:
                        sys.stdout.write(msg.get("line", "") + "\r\n")
                    elif msg.get("ev") == "unit" and msg.get("name") == a.unit:
                        sys.stdout.write(
                            f"\r\n-- {a.unit} is now {msg.get('state')}"
                            f"{': ' + msg['detail'] if msg.get('detail') else ''} --\r\n")
                    sys.stdout.flush()
    except (OSError, KeyboardInterrupt):
        pass
    finally:
        if old_term is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        try:
            sock.close()
        except OSError:
            pass
        sys.stdout.write("\r\n-- detached; the unit is still running --\r\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

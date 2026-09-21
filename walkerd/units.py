"""
The unit table: every process walkerd owns, and how it owns it.

WHY A SUPERVISOR AT ALL
=======================
Version 1 started processes as bash background jobs and found them again with
`pkill -f <pattern>`. Three things went wrong with that, and all three are
fixed here rather than worked around:

  * ORPHANS. A background job whose parent script exits keeps running and
    keeps holding its port. Observed during this project's own bring-up: a
    stale MicroXRCEAgent held UDP 8888, the next one failed to bind, and PX4
    reported "connected" to an agent that was forwarding nothing. Every unit
    here is a child of walkerd, in its own process group, and stopping it
    stops the group.

  * PATTERN MATCHING IS NOT IDENTITY. `pkill -f "gz sim"` also matches the
    shell that runs it (see probes.py). A pid is identity; a pattern is a
    guess.

  * NO EXIT CODES. A job that died was indistinguishable from one that never
    started. Units carry their exit status and their last output.

EVERY UNIT GETS A PTY
=====================
Not a pipe. Two reasons, both learned here:

  * PX4's `pxh` console redraws its prompt forever when its output is a pipe
    -- 22 MB of escape sequences per minute, measured. On a pty it behaves.
  * A pty is what makes `walker-attach sim` useful: the attached terminal gets
    a real console, so `commander status` and `param show` work, exactly as if
    you had started PX4 by hand.

Line-buffering is also a pty behaviour: many programs (including ROS 2 nodes)
switch to block buffering when they detect a pipe, which would make walker's
log pane lag by kilobytes.
"""

from __future__ import annotations

import errno
import fcntl
import os
import pty
import re
import signal
import struct
import subprocess
import termios
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

# Unit states. A unit is in exactly one of these at any moment.
STOPPED = "stopped"
STARTING = "starting"
RUNNING = "running"
STOPPING = "stopping"
FAILED = "failed"

LOG_DIR = "/tmp/walker"
RING = 5000          # lines kept in memory per unit, for attach replay
REPLAY = 200         # lines a newly attached terminal is shown


@dataclass
class LogRule:
    """
    Which of a unit's lines are worth putting on walker's dashboard.

    A unit prints thousands of lines; the dashboard has room for a handful.
    Rather than have walker guess, each unit declares the patterns that matter
    and everything else goes to the file and the attached terminal only.
    """
    pattern: str
    level: str = "info"
    _rx: re.Pattern | None = None

    def match(self, line: str) -> bool:
        if self._rx is None:
            self._rx = re.compile(self.pattern)
        return bool(self._rx.search(line))


@dataclass
class UnitSpec:
    name: str
    argv: list[str]
    env: dict[str, str] = field(default_factory=dict)
    cwd: str | None = None
    # SIGINT, not SIGTERM, wherever a clean shutdown matters. rosbag2 finalises
    # its mcap and writes metadata.yaml on SIGINT; killed, it leaves a bag that
    # `ros2 bag play` refuses to open.
    stop_signal: int = signal.SIGINT
    stop_timeout: float = 20.0
    ready: Callable[[], bool] | None = None
    ready_timeout: float = 120.0
    log_rules: list[LogRule] = field(default_factory=list)
    # Units that must be running first, and units that may not be.
    depends_on: list[str] = field(default_factory=list)
    description: str = ""


class Unit:
    """One supervised process."""

    def __init__(self, spec: UnitSpec, on_event: Callable[[dict], None]):
        self.spec = spec
        self.on_event = on_event
        self.state = STOPPED
        self.pid: int | None = None
        self.started_at: float = 0.0
        self.exit_code: int | None = None
        self.detail = ""
        self.ring: deque[str] = deque(maxlen=RING)
        self._master: int | None = None
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._lock = threading.RLock()
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, f"{spec.name}.log")

    # ── introspection ────────────────────────────────────────────────────────

    def snapshot(self) -> dict:
        return {
            "name": self.spec.name,
            "state": self.state,
            "pid": self.pid,
            "uptime": (time.time() - self.started_at) if self.started_at and
                      self.state in (RUNNING, STARTING) else 0.0,
            "exit_code": self.exit_code,
            "detail": self.detail,
            "description": self.spec.description,
        }

    @property
    def alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        with self._lock:
            if self.alive:
                return
            self.exit_code = None
            self.detail = ""
            self._set_state(STARTING)

            # A pty, not a pipe -- see the module docstring.
            master, slave = pty.openpty()
            # 200x50 is a sane default for a log window; an attaching terminal
            # sends its real size and we resize then.
            self._set_winsize(master, 50, 200)

            env = os.environ.copy()
            env.update(self.spec.env)
            # Unbuffered Python in units, so a crash's last words are not lost
            # in a buffer that never flushed.
            env.setdefault("PYTHONUNBUFFERED", "1")

            try:
                self._proc = subprocess.Popen(
                    self.spec.argv,
                    stdin=slave, stdout=slave, stderr=slave,
                    cwd=self.spec.cwd, env=env,
                    # Its own process group: stopping the unit stops everything
                    # it started, which is what prevents the orphan problem.
                    start_new_session=True,
                    close_fds=True,
                )
            except OSError as e:
                os.close(master); os.close(slave)
                self._set_state(FAILED, f"could not start: {e}")
                return
            finally:
                try:
                    os.close(slave)
                except OSError:
                    pass

            self._master = master
            self.pid = self._proc.pid
            self.started_at = time.time()

            self._reader = threading.Thread(target=self._pump, daemon=True,
                                            name=f"pump-{self.spec.name}")
            self._reader.start()
            threading.Thread(target=self._await_ready, daemon=True,
                             name=f"ready-{self.spec.name}").start()

    def _await_ready(self) -> None:
        """Hold the unit in STARTING until its own probe says it is usable."""
        probe = self.spec.ready
        if probe is None:
            time.sleep(0.4)
            if self.alive:
                self._set_state(RUNNING)
            return

        deadline = time.time() + self.spec.ready_timeout
        while time.time() < deadline:
            if not self.alive:
                tail = " | ".join(list(self.ring)[-3:])
                self._set_state(FAILED, f"exited during start-up: {tail[:200]}")
                return
            try:
                if probe():
                    self._set_state(RUNNING)
                    return
            except Exception:
                pass
            time.sleep(1.0)
        # Running but not usable is a distinct, and much more confusing, state
        # than dead -- so say which one it is.
        self._set_state(FAILED, f"did not become ready within "
                                f"{self.spec.ready_timeout:.0f}s (process is alive)")

    def stop(self) -> None:
        with self._lock:
            if not self.alive:
                self._set_state(STOPPED)
                return
            self._set_state(STOPPING)
            pgid = None
            try:
                pgid = os.getpgid(self._proc.pid)
            except OSError:
                pass

            # Signal the whole group: a unit is usually a launcher plus the
            # processes it started, and signalling only the launcher is how
            # V1 left Gazebo running after "stopping" the simulation.
            try:
                os.killpg(pgid, self.spec.stop_signal) if pgid else \
                    self._proc.send_signal(self.spec.stop_signal)
            except OSError:
                pass

        deadline = time.time() + self.spec.stop_timeout
        while time.time() < deadline:
            if not self.alive:
                break
            time.sleep(0.2)

        with self._lock:
            if self.alive:
                self.detail = f"did not stop in {self.spec.stop_timeout:.0f}s; killed"
                try:
                    os.killpg(pgid, signal.SIGKILL) if pgid else self._proc.kill()
                except OSError:
                    pass
                try:
                    self._proc.wait(timeout=5)
                except Exception:
                    pass
            self.exit_code = self._proc.poll() if self._proc else None
            self._close_master()
            self.pid = None
            self.started_at = 0.0
            self._set_state(STOPPED)

    # ── i/o ──────────────────────────────────────────────────────────────────

    def write(self, data: str) -> None:
        """Send keystrokes to the unit -- this is what makes pxh usable."""
        if self._master is not None:
            try:
                os.write(self._master, data.encode("utf-8", "replace"))
            except OSError:
                pass

    def resize(self, rows: int, cols: int) -> None:
        if self._master is not None:
            self._set_winsize(self._master, rows, cols)

    @staticmethod
    def _set_winsize(fd: int, rows: int, cols: int) -> None:
        try:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
        except OSError:
            pass

    def _pump(self) -> None:
        """Read the pty, fan out to the ring, the file, and the dashboard."""
        buf = b""
        with open(self.log_path, "ab", buffering=0) as logf:
            while True:
                try:
                    chunk = os.read(self._master, 65536)
                except OSError as e:
                    # EIO is the normal end: the child closed the pty.
                    if e.errno not in (errno.EIO, errno.EBADF):
                        pass
                    break
                if not chunk:
                    break
                logf.write(chunk)
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    line = raw.decode("utf-8", "replace").rstrip("\r")
                    self.ring.append(line)
                    self.on_event({"ev": "output", "unit": self.spec.name, "line": line})
                    for rule in self.spec.log_rules:
                        if rule.match(line):
                            self.on_event({"ev": "log", "unit": self.spec.name,
                                           "level": rule.level, "text": line.strip(),
                                           "t": time.time()})
                            break

        # The process ended. Distinguish "we asked it to" from "it fell over",
        # because only one of those is worth interrupting the operator for.
        code = self._proc.poll() if self._proc else None
        with self._lock:
            self.exit_code = code
            if self.state not in (STOPPING, STOPPED):
                tail = " | ".join(list(self.ring)[-3:])
                self._set_state(FAILED, f"exited with code {code}: {tail[:200]}")
            self._close_master()

    def _close_master(self) -> None:
        if self._master is not None:
            try:
                os.close(self._master)
            except OSError:
                pass
            self._master = None

    def _set_state(self, state: str, detail: str = "") -> None:
        self.state = state
        if detail:
            self.detail = detail
        self.on_event({"ev": "unit", **self.snapshot()})

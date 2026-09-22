"""
Observable state: per-route counters and the log ring buffer.

Everything the TUI renders comes from here, behind one lock, so the render
thread never touches live rclpy objects.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

# ── log levels ────────────────────────────────────────────────────────────────

DEBUG, INFO, WARN, ERROR = 10, 20, 30, 40

LEVEL_NAMES = {DEBUG: "DEBUG", INFO: "INFO", WARN: "WARN", ERROR: "ERROR"}
LEVEL_ORDER = [DEBUG, INFO, WARN, ERROR]


@dataclass
class LogRecord:
    when: float
    level: int
    source: str
    message: str

    @property
    def level_name(self) -> str:
        return LEVEL_NAMES.get(self.level, "?")

    @property
    def clock(self) -> str:
        return time.strftime("%H:%M:%S", time.localtime(self.when))


class LogRing:
    """Bounded, thread-safe log buffer with level filtering."""

    def __init__(self, capacity: int = 4000):
        self._lock = threading.Lock()
        self._items: Deque[LogRecord] = deque(maxlen=capacity)
        self._counts = {DEBUG: 0, INFO: 0, WARN: 0, ERROR: 0}
        self._seq = 0

    def add(self, level: int, source: str, message: str) -> None:
        with self._lock:
            self._items.append(LogRecord(time.time(), level, source, message))
            self._counts[level] = self._counts.get(level, 0) + 1
            self._seq += 1

    def debug(self, source, message):
        self.add(DEBUG, source, message)

    def info(self, source, message):
        self.add(INFO, source, message)

    def warn(self, source, message):
        self.add(WARN, source, message)

    def error(self, source, message):
        self.add(ERROR, source, message)

    def tail(self, count: int, min_level: int = DEBUG, needle: str = "") -> List[LogRecord]:
        needle = needle.lower()
        with self._lock:
            items = [r for r in self._items if r.level >= min_level]
        if needle:
            items = [r for r in items if needle in r.message.lower() or needle in r.source.lower()]
        return items[-count:]

    def counts(self) -> Dict[int, int]:
        with self._lock:
            return dict(self._counts)

    @property
    def seq(self) -> int:
        with self._lock:
            return self._seq


# ── per-route statistics ─────────────────────────────────────────────────────


@dataclass
class RouteStats:
    """
    Counters for one route. `hz` is an exponentially weighted estimate of the
    inbound rate so the display settles quickly but does not jitter.
    """

    rx: int = 0
    tx: int = 0
    dropped_rate: int = 0  # discarded by the rate limiter
    dropped_error: int = 0  # converter raised
    last_rx: float = 0.0
    last_tx: float = 0.0
    hz: float = 0.0
    latency_ms: float = -1.0
    last_error: str = ""
    last_sample: str = ""

    # Field-level capture for the INSPECT screen. Only ever populated for the
    # one route the operator is currently inspecting -- exploding every message
    # of every route would cost more than the bridging itself. Each is a tuple
    # of (path, formatted value) pairs, replaced wholesale so the render thread
    # can read them without a lock.
    sample_in: Tuple[Tuple[str, str], ...] = ()
    sample_out: Tuple[Tuple[str, str], ...] = ()
    sample_in_type: str = ""
    sample_out_type: str = ""
    sample_out_topic: str = ""
    sample_note: str = ""
    sample_when: float = 0.0

    _prev_rx: float = field(default=0.0, repr=False)

    def note_rx(self, now: Optional[float] = None) -> None:
        now = now if now is not None else time.monotonic()
        if self._prev_rx > 0.0:
            dt = now - self._prev_rx
            if dt > 1e-6:
                instant = 1.0 / dt
                # 0.2 weight: ~15 samples to converge, immune to single spikes.
                self.hz = instant if self.hz == 0.0 else (0.8 * self.hz + 0.2 * instant)
        self._prev_rx = now
        self.rx += 1
        self.last_rx = time.time()

    def note_tx(self) -> None:
        self.tx += 1
        self.last_tx = time.time()

    def age(self) -> float:
        """Seconds since last inbound message; large sentinel when never."""
        if self.last_rx == 0.0:
            return 1e9
        return time.time() - self.last_rx

    def decay(self, stale_after: float) -> None:
        """Let the rate estimate fall to zero once traffic stops."""
        if self.hz > 0.0 and self.age() > stale_after:
            self.hz = 0.0


class BridgeState:
    """
    All mutable observable state, in one place, behind one lock.

    The rclpy executor writes; the TUI reads via snapshot(). Nothing else
    shares memory between those two threads.
    """

    def __init__(self, log_capacity: int = 4000):
        self._lock = threading.Lock()
        self.log = LogRing(log_capacity)
        self._stats: Dict[str, RouteStats] = {}
        self._service_calls: Dict[str, Tuple[int, int, str]] = {}  # key -> (ok, fail, last)
        self._flight: Dict[str, object] = {}

    # -- route stats --

    def stats(self, key: str) -> RouteStats:
        with self._lock:
            st = self._stats.get(key)
            if st is None:
                st = RouteStats()
                self._stats[key] = st
            return st

    def snapshot_stats(self) -> Dict[str, RouteStats]:
        with self._lock:
            return dict(self._stats)

    def forget(self, key: str) -> None:
        with self._lock:
            self._stats.pop(key, None)

    def decay_all(self, stale_after: float) -> None:
        with self._lock:
            values = list(self._stats.values())
        for st in values:
            st.decay(stale_after)

    # -- service call accounting --

    def note_service(self, key: str, ok: bool, detail: str = "") -> None:
        with self._lock:
            good, bad, _ = self._service_calls.get(key, (0, 0, ""))
            if ok:
                good += 1
            else:
                bad += 1
            self._service_calls[key] = (good, bad, detail)

    def service_calls(self) -> Dict[str, Tuple[int, int, str]]:
        with self._lock:
            return dict(self._service_calls)

    # -- flight state, for the dashboard --

    def set_flight(self, **kwargs) -> None:
        with self._lock:
            self._flight.update(kwargs)

    def flight(self) -> Dict[str, object]:
        with self._lock:
            return dict(self._flight)

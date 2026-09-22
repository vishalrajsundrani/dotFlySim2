"""
Time domain tracking. There are three clocks in this system, and they are not
the same clock.

  wall      time.time(). What a real Manifold lives in, and the only thing
            available when no simulator is running.

  PX4       every uORB message's `timestamp` field: microseconds since PX4
            boot, advanced by the Gazebo lockstep scheduler. Stamping a
            TrajectorySetpoint with `time.time() * 1e6` hands PX4 a timestamp
            roughly 56 years in the future, which breaks the offboard
            freshness check and makes any latency figure meaningless.
            Px4Clock tracks this one -- see below.

  Gazebo    /clock, the simulated world time. This is what every image
            bridged by ros_gz_bridge carries in its header, because gz-sim
            stamps sensor output in world time.

PX4 AND GAZEBO ARE NOT THE SAME DOMAIN. gz sim starts first and PX4 comes up
last (config/launch_sim.sh), so PX4 boot is some tens of seconds after world
t=0 and the two differ by that offset. Anything that has to line up with a
camera frame must therefore use SimClock (Gazebo), not Px4Clock (PX4).

WHY THIS MATTERS ENOUGH TO HAVE ITS OWN MODULE
----------------------------------------------
Under lockstep Gazebo runs at whatever real-time factor the machine sustains,
routinely well under 1.0 with rendering on. A message stamped with the wall
clock and an image stamped with sim time therefore drift apart without bound:
after two minutes at RTF 0.6 they are the better part of a minute apart. Any
tightly-coupled estimator -- anything that pairs an IMU sample with a frame --
is then being fed a lie, and fails in a way that looks like a bad extrinsic
rather than a bad clock.
"""

import threading
import time
from typing import Optional


class Px4Clock:
    """Tracks the PX4 (simulated) microsecond clock. Thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._px4_us = 0
        self._at_mono = 0.0
        self._samples = 0

    def feed(self, px4_timestamp_us) -> None:
        """
        Offer a `timestamp` field read off any PX4 message. Monotonic values
        only ever move the estimate forward; stale or zero values are ignored.
        """
        try:
            ts = int(px4_timestamp_us)
        except (TypeError, ValueError):
            return
        if ts <= 0:
            return
        with self._lock:
            if ts >= self._px4_us:
                self._px4_us = ts
                self._at_mono = time.monotonic()
                self._samples += 1

    def now_us(self) -> int:
        """
        Current time in PX4's domain. Falls back to the wall clock until the
        first PX4 message arrives, so the bridge still functions (with
        meaningless latency numbers) when PX4 is not running.
        """
        with self._lock:
            if self._px4_us == 0:
                return int(time.time() * 1e6)
            elapsed = time.monotonic() - self._at_mono
            return int(self._px4_us + elapsed * 1e6)

    def latency_ms(self, msg_timestamp_us) -> float:
        """
        Age of a PX4 message in milliseconds. Returns -1.0 when the clock is
        not yet synced or the timestamp is unusable, so callers can render
        "n/a" rather than a fabricated number.
        """
        try:
            ts = int(msg_timestamp_us)
        except (TypeError, ValueError):
            return -1.0
        with self._lock:
            if self._px4_us == 0 or ts <= 0:
                return -1.0
        return max(0.0, (self.now_us() - ts) / 1000.0)

    @property
    def synced(self) -> bool:
        with self._lock:
            return self._px4_us > 0

    @property
    def sample_count(self) -> int:
        with self._lock:
            return self._samples

    def describe(self) -> str:
        with self._lock:
            if self._px4_us == 0:
                return "wall clock (no PX4 traffic yet)"
            return f"PX4 sim clock, t={self._px4_us / 1e6:.1f}s"


class SimClock:
    """
    Tracks Gazebo's simulated world clock, fed from /clock. Thread-safe.

    NOTHING IS EXTRAPOLATED HERE, deliberately. The obvious implementation --
    remember the last sim time and add the wall seconds elapsed since -- is
    exactly the bug this class exists to prevent: it converts sim time back
    into wall time at a rate of (1 - RTF) per second. Gazebo publishes /clock
    every physics iteration (real_time_update_rate 250 in
    worlds/powerline.sdf), so the last received value is at worst ~4 ms stale,
    which is an order of magnitude better than one camera frame at 20 Hz and
    bounded rather than cumulative.

    `synced` goes false when /clock stops arriving, which is what makes the
    fallback to wall time safe: a real Manifold publishes no /clock at all and
    so never leaves wall time, and a simulator that dies mid-session degrades
    to wall time instead of freezing every stamp at the moment it stopped.
    """

    def __init__(self, stale_after_s: float = 2.0):
        self._lock = threading.Lock()
        self._sim_ns = 0
        self._at_mono = 0.0
        self._samples = 0
        self._stale_after_s = max(0.1, float(stale_after_s))

    def feed(self, sec, nanosec) -> None:
        """Offer a rosgraph_msgs/Clock payload. Monotonic values only."""
        try:
            ns = int(sec) * 1_000_000_000 + int(nanosec)
        except (TypeError, ValueError):
            return
        if ns <= 0:
            return
        with self._lock:
            # A sim-time reset (world reload) moves backwards; accept it rather
            # than wedging on a stale future value, but only when the jump is
            # large enough not to be reordered delivery.
            if ns >= self._sim_ns or (self._sim_ns - ns) > 1_000_000_000:
                self._sim_ns = ns
                self._at_mono = time.monotonic()
                self._samples += 1

    def now_ns(self) -> Optional[int]:
        """Latest sim time in nanoseconds, or None when not synced."""
        with self._lock:
            if self._sim_ns == 0:
                return None
            if (time.monotonic() - self._at_mono) > self._stale_after_s:
                return None
            return self._sim_ns

    @property
    def synced(self) -> bool:
        return self.now_ns() is not None

    @property
    def sample_count(self) -> int:
        with self._lock:
            return self._samples

    def describe(self) -> str:
        now = self.now_ns()
        if now is None:
            with self._lock:
                if self._sim_ns == 0:
                    return "no /clock seen; stamping wall time"
                age = time.monotonic() - self._at_mono
            return f"/clock stale by {age:.1f}s; stamping wall time"
        return f"Gazebo sim clock, t={now / 1e9:.1f}s"

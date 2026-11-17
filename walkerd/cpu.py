"""
Per-unit CPU and memory sampling, read straight from /proc.

A unit is a process plus whatever it spawns under it (its own process group --
see units.py). Reading only the one pid walkerd forked would undercount the
moment a mission shells out to anything, so every sample walks the pid's whole
descendant tree and sums it.

No psutil: /proc is already guaranteed to be there, and a sampler this small
is not worth a dependency.
"""

from __future__ import annotations

import os
import time

try:
    _CLK_TCK = os.sysconf("SC_CLK_TCK")
except (ValueError, AttributeError):
    _CLK_TCK = 100
try:
    _PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
except (ValueError, AttributeError):
    _PAGE_SIZE = 4096


def _cpu_seconds(pid: int) -> float | None:
    """utime+stime for one pid, in seconds. None if it is gone."""
    try:
        with open(f"/proc/{pid}/stat", encoding="utf-8", errors="replace") as f:
            data = f.read()
    except OSError:
        return None
    # Fields after the "(comm)" parenthetical, which may itself hold spaces
    # or parens -- split on the LAST ')' rather than the field position.
    rest = data.rsplit(")", 1)[-1].split()
    try:
        return (int(rest[11]) + int(rest[12])) / _CLK_TCK
    except (IndexError, ValueError):
        return None


def _children(pid: int) -> list[int]:
    out: list[int] = []
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"/proc/{pid}/task/{tid}/children", encoding="utf-8") as f:
                out += [int(p) for p in f.read().split()]
        except OSError:
            continue
    return out


def tree(pid: int) -> list[int]:
    """pid and every descendant, walked breadth-first."""
    seen = [pid]
    frontier = [pid]
    while frontier:
        nxt = [c for p in frontier for c in _children(p)]
        frontier = [p for p in nxt if p not in seen]
        seen.extend(frontier)
    return seen


def rss_bytes(pid: int) -> int:
    """Resident memory of the whole tree, in bytes."""
    total = 0
    for p in tree(pid):
        try:
            with open(f"/proc/{p}/statm", encoding="utf-8") as f:
                total += int(f.read().split()[1]) * _PAGE_SIZE
        except OSError:
            continue
    return total


class Sampler:
    """
    One unit's CPU cost, tracked across repeated calls to `sample`.

    cpu_pct is the load since the LAST sample -- it can read above 100% for a
    unit with more than one busy thread, which is correct, not a bug: 100% is
    "one core", not "all of it".

    cpu_seconds is the running total since the unit started: how much CPU
    TIME it has actually cost, which is the number that answers "was this run
    expensive" long after the instantaneous percentage has been forgotten.
    """

    def __init__(self) -> None:
        self.cpu_pct = 0.0
        self.cpu_seconds = 0.0
        self.rss = 0
        self._last_t = 0.0
        self._last_cpu = 0.0

    def reset(self) -> None:
        self.__init__()

    def sample(self, pid: int | None) -> None:
        if pid is None:
            self.cpu_pct = 0.0
            return
        now = time.time()
        total = 0.0
        alive = False
        for p in tree(pid):
            t = _cpu_seconds(p)
            if t is not None:
                alive = True
                total += t
        if not alive:
            self.cpu_pct = 0.0
            return
        if self._last_t:
            dt = now - self._last_t
            if dt > 0:
                self.cpu_pct = max(0.0, (total - self._last_cpu) / dt * 100.0)
        self._last_t, self._last_cpu = now, total
        self.cpu_seconds = total
        self.rss = rss_bytes(pid)

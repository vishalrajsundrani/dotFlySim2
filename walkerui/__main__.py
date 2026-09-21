"""
walker — the operator surface for dotFlySim2.

    walker                 the TUI (from M2 on)
    walker up              start the container
    walker down [--rm]     stop it (--rm also removes it)
    walker status          one screen: container, mounts, isolation
    walker doctor          every check, by name, with a fix for each failure
    walker shell           an interactive shell inside the container

WHY THERE IS A CLI AT ALL, WHEN THE POINT IS A TUI
--------------------------------------------------
Three reasons, and each of them is something V1 could not do:
  * CI has no terminal. `walker doctor` exits non-zero and prints a report.
  * A failure that stops the TUI from starting still has to be diagnosable.
  * Every check the TUI shows is a function here, so the two can never drift.
"""

from __future__ import annotations

import argparse
import subprocess
import sys

from . import dockerctl, doctor, paths, ui


def cmd_up(args: argparse.Namespace) -> int:
    ui.step(f"starting '{paths.CONTAINER}' from {paths.IMAGE}")
    try:
        for note in dockerctl.start(force_recreate=args.recreate):
            ui.info(note)
    except dockerctl.DockerError as e:
        for i, line in enumerate(str(e).splitlines()):
            (ui.err if i == 0 else ui.info)(line)
        return 1
    ui.ok("container is up")
    ui.info("next:  walker doctor    (verifies mounts, isolation and the ROS graph)")
    return 0


def cmd_down(args: argparse.Namespace) -> int:
    ui.step("stopping the container")
    try:
        for note in dockerctl.stop(remove=args.rm):
            ui.info(note)
    except dockerctl.DockerError as e:
        ui.err(str(e))
        return 1
    return 0


def cmd_doctor(args: argparse.Namespace) -> int:
    ui.step("dotFlySim2 — readiness")
    report = doctor.run(deep=args.deep)
    print()
    for c in report.checks:
        print(ui.mark(c.state, c.name, c.detail))
    fails = report.failed
    warns = [c for c in report.checks if c.state == "warn"]
    print()
    if fails:
        ui.err(f"{len(fails)} check(s) failed")
        for c in fails:
            if c.fix:
                print(f"      {ui.BOLD}{c.name}{ui.RESET}: {c.fix}")
        return 1
    if warns:
        for c in warns:
            if c.fix:
                print(f"      {ui.DIM}{c.name}: {c.fix}{ui.RESET}")
    ui.ok("everything the stack depends on is in place")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    st = dockerctl.state()
    print(f"\n{ui.BOLD}dotFlySim2{ui.RESET}   container {st.label}   image {paths.IMAGE}\n")
    print(f"  {ui.DIM}{'host':<34} {'container':<40} mode{ui.RESET}")
    for m in paths.MOUNTS:
        rel = str(m.host.relative_to(paths.REPO)) if m.host.is_relative_to(paths.REPO) else str(m.host)
        present = "" if m.host.is_dir() else f"  {ui.RED}(missing){ui.RESET}"
        print(f"  {rel:<34} {m.container:<40} {m.mode}{present}")
    print()
    return 0


def cmd_shell(args: argparse.Namespace) -> int:
    if not dockerctl.state().running:
        ui.err("the container is not running")
        ui.info("walker up")
        return 1
    return subprocess.call(
        dockerctl.engine() + ["exec", "-it", paths.CONTAINER, "bash", "-l"])


def cmd_tui(args: argparse.Namespace) -> int:
    ui.warn("the TUI lands in M2; until then use: walker up | doctor | status | shell")
    return cmd_status(args)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="walker", description="dotFlySim2 operator surface")
    sub = p.add_subparsers(dest="cmd")

    s = sub.add_parser("up", help="start the container")
    s.add_argument("--recreate", action="store_true",
                   help="remove and recreate it (needed after a mount change)")
    s.set_defaults(fn=cmd_up)

    s = sub.add_parser("down", help="stop the container")
    s.add_argument("--rm", action="store_true", help="also remove it")
    s.set_defaults(fn=cmd_down)

    s = sub.add_parser("doctor", help="check everything, with a fix for each failure")
    s.add_argument("--deep", action="store_true", help="include the slow checks")
    s.set_defaults(fn=cmd_doctor)

    sub.add_parser("status", help="container and mounts").set_defaults(fn=cmd_status)
    sub.add_parser("shell", help="interactive shell inside the container").set_defaults(fn=cmd_shell)

    args = p.parse_args(argv)
    fn = getattr(args, "fn", cmd_tui)
    for d in ("deep", "recreate", "rm"):
        if not hasattr(args, d):
            setattr(args, d, False)
    try:
        return fn(args)
    except KeyboardInterrupt:
        print()
        return 130


if __name__ == "__main__":
    sys.exit(main())

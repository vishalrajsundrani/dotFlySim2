"""Terminal colouring, shared by the CLI and (later) the TUI's plain-text modes."""

from __future__ import annotations

import os
import sys

_tty = sys.stdout.isatty() and not os.environ.get("NO_COLOR")

RESET = "\033[0m" if _tty else ""
DIM = "\033[2m" if _tty else ""
BOLD = "\033[1m" if _tty else ""
RED = "\033[31m" if _tty else ""
GREEN = "\033[32m" if _tty else ""
YELLOW = "\033[33m" if _tty else ""
BLUE = "\033[34m" if _tty else ""
CYAN = "\033[36m" if _tty else ""

_MARK = {
    "ok":   (GREEN,  "[ ok ]"),
    "warn": (YELLOW, "[warn]"),
    "fail": (RED,    "[fail]"),
    "skip": (DIM,    "[skip]"),
}


def mark(state: str, name: str, detail: str = "") -> str:
    colour, tag = _MARK.get(state, (DIM, "[    ]"))
    return f"  {colour}{tag}{RESET} {name:<24} {DIM if state == 'skip' else ''}{detail}{RESET}"


def step(text: str) -> None:
    print(f"\n{BOLD}{CYAN}==> {text}{RESET}")


def info(text: str) -> None:
    print(f"  {BLUE}•{RESET} {text}")


def ok(text: str) -> None:
    print(f"  {GREEN}✔{RESET} {text}")


def warn(text: str) -> None:
    print(f"  {YELLOW}!{RESET} {text}")


def err(text: str) -> None:
    print(f"  {RED}✖{RESET} {text}", file=sys.stderr)

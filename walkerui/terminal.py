"""
Giving a unit its own terminal window.

THE HOST REALITY THIS WAS WRITTEN AGAINST
-----------------------------------------
GNOME on Wayland, with `ptyxis` as the only installed emulator -- no
gnome-terminal, no xterm, no tmux. So the obvious `gnome-terminal -- cmd` that
most projects hard-code would simply not work here, and a plan that assumed it
would have failed on the first machine it met.

Hence: a detection chain, an environment override, and a fallback that is not
"nothing". A missing emulator costs the separate window and nothing else --
walker shows every unit's log itself, so the simulation still runs.
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class Emulator:
    name: str
    # argv builder: (title, command string) -> argv for Popen
    build: object

    def argv(self, title: str, command: str) -> list[str]:
        return self.build(title, command)  # type: ignore[operator]


# Ordered by preference. ptyxis first because it is what modern Ubuntu/GNOME
# ships; xterm last because it is ugly but almost always works.
_CHAIN: tuple[tuple[str, object], ...] = (
    ("ptyxis",          lambda t, c: ["ptyxis", "--new-window", "--title", t, "--", "bash", "-lc", c]),
    ("gnome-terminal",  lambda t, c: ["gnome-terminal", f"--title={t}", "--", "bash", "-lc", c]),
    ("konsole",         lambda t, c: ["konsole", "-p", f"tabtitle={t}", "-e", "bash", "-lc", c]),
    ("xfce4-terminal",  lambda t, c: ["xfce4-terminal", f"--title={t}", "-x", "bash", "-lc", c]),
    ("kitty",           lambda t, c: ["kitty", "--title", t, "bash", "-lc", c]),
    ("alacritty",       lambda t, c: ["alacritty", "-t", t, "-e", "bash", "-lc", c]),
    ("wezterm",         lambda t, c: ["wezterm", "start", "--", "bash", "-lc", c]),
    ("foot",            lambda t, c: ["foot", "-T", t, "bash", "-lc", c]),
    ("x-terminal-emulator",
                        lambda t, c: ["x-terminal-emulator", "-T", t, "-e", "bash", "-lc", c]),
    ("xterm",           lambda t, c: ["xterm", "-T", t, "-e", "bash", "-lc", c]),
)


def detect() -> Emulator | None:
    """
    The emulator walker will use, or None.

    SIM_TERMINAL overrides the chain entirely, for a machine with something
    unusual. It is a template containing {title} and {cmd}:

        SIM_TERMINAL='myterm --title {title} -e bash -lc {cmd}'

    {cmd} is substituted already quoted, so the template must NOT add quotes.
    """
    override = os.environ.get("SIM_TERMINAL")
    if override:
        def build(t: str, c: str, tpl: str = override) -> list[str]:
            return shlex.split(tpl.replace("{title}", shlex.quote(t)).replace("{cmd}", shlex.quote(c)))
        return Emulator("SIM_TERMINAL", build)

    for name, build in _CHAIN:
        if shutil.which(name):
            return Emulator(name, build)
    return None


def open_window(title: str, command: str) -> tuple[bool, str]:
    """
    Open a terminal running `command`. Returns (opened, note).

    Never raises: failing to open a window must not take a unit down with it.
    """
    emu = detect()
    if emu is None:
        return False, ("no terminal emulator found; run this yourself to get a window:\n"
                       f"    {command}")
    try:
        subprocess.Popen(
            emu.argv(title, command),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,   # survives walker exiting
        )
        return True, f"opened a {emu.name} window for {title}"
    except OSError as e:
        return False, f"{emu.name} failed to start ({e}); run it yourself:\n    {command}"

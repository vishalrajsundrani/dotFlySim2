"""
simty — the PSDK Wrapper <-> dotFlySim bridge, as a set of composable pieces.

Entry point is ../ROS_Bridge_Simty.py. Nothing here imports rclpy at package
import time on purpose: the launcher must be able to set RMW_IMPLEMENTATION and
friends *before* the middleware and any message typesupport get dlopen'd.

Layout
------
The bridge proper:

  settings.py    runtime-tunable settings + JSON profile persistence
  qos.py         named QoS presets and the compatibility rules behind them
  clock.py       PX4 time domain tracking (SITL runs on simulated time)
  state.py       per-route counters/rates and the in-memory log ring
  converters.py  pure message translation functions
  registry.py    the declarative route tables -- add a row to add a bridge
  discovery.py   reads the DDS graph: is the surface up, and is it only ours?
  node.py        the rclpy node that realises the route tables

The control room. In V1 the only thing that could change state was the built-in
console; in V2 that is walker, which reaches the same command queue over
/simty/control:

  control.py     the command queue: one verb per message on /simty/control
  journal.py     every conversion compromise, aggregated by (route, code)
  viz.py         RViz marker boards for flow, services and the aircraft
  diagnostics.py /simty/diagnostics and /simty/conversion_events, for scripts

Removed in Version 2 (about 3 500 lines), because none of it applies to a
simulation that runs entirely inside one container:

  mock.py        synthetic Manifold publishing the whole /wrapper surface
  standin.py     the above plus a stub for every psdk service we do not serve
  remote.py      started the real wrapper on a Manifold over SSH
  provision.py   the escalation policy between those three
  tui.py         the interactive console -- walker is the operator surface now

To add a bridged topic you touch exactly two files: a converter in
converters.py and a row in registry.py. Everything else -- endpoint creation,
QoS, rate limiting, stats, TUI rows, RViz rows, diagnostics, live
enable/disable -- is driven off that row. A converter that has to clamp,
substitute or infer calls ctx.warn(<code>), and the code is documented once in
journal.CODES.
"""

__all__ = [
    "settings",
    "qos",
    "clock",
    "state",
    "converters",
    "registry",
    "discovery",
    "journal",
    "node",
    "viz",
    "diagnostics",
]

__version__ = "2.1.0"

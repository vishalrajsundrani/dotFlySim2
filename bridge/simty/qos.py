"""
Named QoS presets.

THE RULE THAT MATTERS
--------------------
DDS matches a reader to a writer only when the reader's REQUESTED quality is no
stronger than the writer's OFFERED quality. Two consequences drive every choice
in this file:

  * Subscribing with BEST_EFFORT / VOLATILE matches *everything* -- best-effort
    and reliable writers, volatile and transient-local ones. Subscribing with
    RELIABLE silently matches nothing when the writer is best-effort. No error,
    no warning, just zero messages forever.

  * Publishing with RELIABLE satisfies both best-effort and reliable readers.

So: subscribe permissively, publish as strongly as the data rate allows.

This is not academic. PX4's uXRCE-DDS agent publishes every /fmu/out/* topic as
BEST_EFFORT. The previous version of this bridge subscribed to them with a
RELIABLE profile and therefore received nothing at all, while its health
display cheerfully showed the topics as registered.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

_BE = ReliabilityPolicy.BEST_EFFORT
_REL = ReliabilityPolicy.RELIABLE
_VOL = DurabilityPolicy.VOLATILE
_TL = DurabilityPolicy.TRANSIENT_LOCAL


def _profile(reliability, durability, depth):
    return QoSProfile(
        reliability=reliability,
        durability=durability,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# name -> (factory, one-line description)
PRESETS = {
    # Maximum-compatibility subscription. Use this for EVERY subscription
    # unless you have a specific reason not to.
    "compat": (lambda: _profile(_BE, _VOL, 10), "BEST_EFFORT/VOLATILE d10 - subscribes to anything"),
    # What PX4 itself uses on /fmu/in and /fmu/out.
    "px4": (lambda: _profile(_BE, _VOL, 5), "BEST_EFFORT/VOLATILE d5 - PX4 native"),
    # Low-rate telemetry and commands where loss is not acceptable.
    "reliable": (lambda: _profile(_REL, _VOL, 10), "RELIABLE/VOLATILE d10 - low-rate, no loss"),
    # State that a late joiner must see immediately (e.g. RC authority).
    "latched": (lambda: _profile(_REL, _TL, 1), "RELIABLE/TRANSIENT_LOCAL d1 - latched state"),
    # Video frames. BEST_EFFORT because a retransmitted frame arrives too late to
    # be worth anything, and depth 1 because a subscriber that cannot keep up
    # should fall one frame behind rather than queue megabytes: at ~6 MB an
    # uncompressed 1080p frame, a depth-10 image queue is 60 MB of RAM per
    # endpoint and seconds of latency.
    "video": (lambda: _profile(_BE, _VOL, 1), "BEST_EFFORT/VOLATILE d1 - video frames"),
}

# Cycle order for the TUI's "change QoS" key.
ORDER = ["compat", "px4", "reliable", "latched", "video"]

DEFAULT_SUB = "compat"
DEFAULT_PUB = "px4"


def build(name: str) -> QoSProfile:
    """Instantiate a fresh QoSProfile for a preset name."""
    factory, _ = PRESETS.get(name, PRESETS[DEFAULT_SUB])
    return factory()


def describe(name: str) -> str:
    _, text = PRESETS.get(name, (None, "unknown preset"))
    return text


def cycle(name: str) -> str:
    """Next preset in ORDER, wrapping. Used by the TUI."""
    try:
        return ORDER[(ORDER.index(name) + 1) % len(ORDER)]
    except ValueError:
        return DEFAULT_SUB


def warn_if_unsafe_subscription(name: str) -> str:
    """
    Return a human-readable warning when a subscription preset risks silently
    matching nothing, or "" when it is safe.
    """
    factory, _ = PRESETS.get(name, (None, None))
    if factory is None:
        return f"unknown QoS preset '{name}'"
    profile = factory()
    if profile.reliability == _REL:
        return (
            "RELIABLE subscription will NOT match a BEST_EFFORT publisher "
            "(all /fmu/out/* topics are best-effort)"
        )
    return ""

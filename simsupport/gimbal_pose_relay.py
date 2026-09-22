#!/usr/bin/env python3
"""
Gimbal pose relay — subscribes to Gazebo's /world/default/pose/info (via
Gazebo Transport) and republishes the drone's (m4e_0) pose on a ROS2 topic.

Why this is a separate process: a busy gz-transport subscription and
repeated gz-transport service requests (set_pose, in gui/gimbal_stabilizer.py)
starve each other when run in the SAME process — confirmed empirically: an
isolated request loop with no subscription sustained ~150Hz reliably, but
adding a concurrent Pose_V subscription (even on a separate gz.transport13.
Node instance) dropped that to ~40% success. Splitting the "watch gz state"
and "command gz state" roles into separate OS processes, talking over ROS2
in between, avoids the contention entirely.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

# ---------------------------------------------------------------------------
# WHICH WORLD AND WHICH MODEL: read from the composition, not hard-coded.
#
# Version 1 could hard-code "default" and "m4e_0" because there was exactly one
# world and one aircraft, both baked into the image. Version 2 composes a run
# from a chosen drone and a chosen world, so these have to follow that choice
# or this node silently teleports a model that is not there -- and the symptom
# is a payload camera filming the ground where the drone used to be.
#
# compose.json is written by tools/compose_sim.py immediately before the
# simulation starts; the environment overrides it for anyone running this node
# by hand.
# ---------------------------------------------------------------------------
import json as _json
import os as _os


def _composition() -> dict:
    path = _os.environ.get("SIM_COMPOSE",
                           _os.path.expanduser("~/gz_runtime/compose.json"))
    try:
        with open(path, encoding="utf-8") as fh:
            return _json.load(fh)
    except (OSError, ValueError):
        return {}


_COMP = _composition()

WORLD = _os.environ.get("SIM_GZ_WORLD", _COMP.get("gz_world_name", "default"))
# PX4 spawns the vehicle as <model>_<instance>; instance 0 is the only one here.
DRONE_MODEL = _os.environ.get(
    "SIM_DRONE_MODEL", f'{_COMP.get("drone", "m4e")}_0')
TOPIC = "/drone/gz_pose"

# Reject a single pose sample outright if its orientation implies an angular
# rate faster than any real maneuver could produce -- guards against an
# occasional glitched/corrupted sample under CPU load causing a visible
# stuck-rotation artifact downstream (see gui/gimbal_stabilizer.py).
MAX_ATT_RATE = math.radians(400)  # rad/s


class GimbalPoseRelay(Node):
    def __init__(self):
        super().__init__("gimbal_pose_relay_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(PoseStamped, TOPIC, qos)

        self._last_quat = None  # (w, x, y, z)
        self._last_t = None

        self._gz_node = gz_transport.Node()
        self._gz_node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self._pose_cb)
        self.get_logger().info("GimbalPoseRelay ready")

    def _stamp(self, msg: Pose_V, pose) -> Time:
        """
        Gazebo's own sim-time stamp, not the wall clock.

        This is ground truth for anything measuring localization error, and it
        has to share a time base with the data being measured -- which is
        Gazebo's, because that is what bridged camera frames carry and what the
        PSDK bridge now stamps with (see gui/simty/clock.py). Taking
        get_clock().now() here instead, as this did, made the trajectory drift
        against the estimate at a rate of (1 - real_time_factor), which looks
        exactly like estimator error and is not.

        Reading the stamp off the message rather than setting use_sim_time on
        this node is deliberate: with use_sim_time and no /clock yet, rclpy
        hands back time zero, and a pose stamped at the epoch is worse than a
        wall-clock one. Here an unpopulated header simply falls back.

        gz.msgs.Time spells its fields sec/nsec, not sec/nanosec.
        """
        for header in (getattr(pose, "header", None), getattr(msg, "header", None)):
            stamp = getattr(header, "stamp", None)
            if stamp is None:
                continue
            sec, nsec = int(getattr(stamp, "sec", 0)), int(getattr(stamp, "nsec", 0))
            if sec or nsec:
                return Time(sec=sec, nanosec=nsec)
        return self.get_clock().now().to_msg()

    def _pose_cb(self, msg: Pose_V) -> None:
        for p in msg.pose:
            if p.name == DRONE_MODEL:
                q = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                now = time.monotonic()

                if self._last_quat is not None:
                    dt = max(1e-4, now - self._last_t)
                    dot = sum(a * b for a, b in zip(q, self._last_quat))
                    angle = 2 * math.acos(max(-1.0, min(1.0, abs(dot))))
                    if angle > MAX_ATT_RATE * dt:
                        return  # outlier -- drop this sample entirely

                self._last_quat, self._last_t = q, now

                out = PoseStamped()
                out.header.stamp = self._stamp(msg, p)
                out.pose.position.x = p.position.x
                out.pose.position.y = p.position.y
                out.pose.position.z = p.position.z
                out.pose.orientation.w = p.orientation.w
                out.pose.orientation.x = p.orientation.x
                out.pose.orientation.y = p.orientation.y
                out.pose.orientation.z = p.orientation.z
                self._pub.publish(out)
                return


def main():
    rclpy.init()
    node = GimbalPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

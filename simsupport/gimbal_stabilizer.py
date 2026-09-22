#!/usr/bin/env python3
"""
Gimbal stabilizer — kinematically teleports the standalone m4e_camera model
(models/m4e_camera/model.sdf) so its gimbal_link tracks the drone's position
rigidly but cancels the drone's roll/pitch, every tick.

Why a separate model: Gazebo's /world/<world>/set_pose service only sticks
on a free-standing (unconstrained) model. A child link inside an articulated
body's own kinematic tree gets its Cartesian pose recomputed from the joint's
generalized coordinates every physics step regardless of what you write to
it — confirmed empirically (see conversation), for both fixed and revolute
joints. Splitting the camera assembly into its own model with no joint back
to the drone sidesteps that entirely: it's a plain rigid body, and teleporting
it is exactly the supported, standard use of set_pose (same mechanism used to
teleport a whole model/prop in a scene).

This uses:
  - ROS2 for both /fmu/out/vehicle_attitude (roll/pitch) and /drone/gz_pose
    (the drone's live Gazebo pose, republished by gui/gimbal_pose_relay.py
    -- a SEPARATE process, deliberately: a busy gz-transport subscription
    and repeated gz-transport set_pose requests starve each other when run
    in the same process, confirmed empirically. This process has ZERO
    gz-transport subscriptions, only outgoing requests, so it gets full
    throughput).
  - Gazebo Transport (gz.transport13 / gz.msgs10) only for calling
    /world/default/set_pose — not a ROS service, so it can't be reached via
    the ROS bridge.

Correction math: the target orientation is

    R_target = R_drone (live, from Gazebo's own pose stream)
               @ Rot_X(-roll) @ Rot_Y(-pitch)   (cancel roll, then pitch)
               @ Rot_Z(-90deg)                   (fixed camera mount yaw)

and target position = drone_position + R_drone @ LOCAL_OFFSET (rigid
attachment — only orientation is corrected, not translation). Using
Gazebo's own live drone orientation (rather than PX4's quaternion) for the
R_drone term avoids any PX4-NED-vs-Gazebo-world frame mismatch; PX4's
roll/pitch VALUES are still used for the correction angles themselves, since
that mapping onto Gazebo's body-frame X/Y axes was already validated
directionally correct earlier in this project.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from px4_msgs.msg import VehicleAttitude

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# Manual pan/roll/tilt setpoints, applied on top of the auto roll/pitch
# cancellation below. Limits match the real DJI Matrice 4E gimbal's soft
# stops (same values as the old JointPositionController joint limits in
# models/m4e/model.sdf before the physical-joint gimbal was replaced by this
# kinematic-teleport approach).
GIMBAL_LIMITS = {
    "pan":  (-1.047198,  1.047198),   # ±60°
    "roll": (-0.820305,  0.820305),   # ±47°
    "tilt": (-1.570796,  0.610865),   # -90° / +35°
}
GIMBAL_CMD_TOPICS = {
    "pan":  "/drone/gimbal/cmd/pan",
    "roll": "/drone/gimbal/cmd/roll",
    "tilt": "/drone/gimbal/cmd/tilt",
}

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
GZ_POSE_TOPIC = "/drone/gz_pose"
# set_pose only sticks when addressed by the top-level MODEL name -- by link
# name it silently no-ops (confirmed empirically). gimbal_link is this
# model's root/canonical link, so moving the model IS moving gimbal_link.
# The payload model this drone declares as its gimbal attachment.
def _payload_model() -> str:
    staged = _COMP.get("staged_models", [])
    drone = _COMP.get("drone", "m4e")
    for name in staged:
        if name != drone:
            return name
    return "m4e_camera"


TARGET_MODEL = _os.environ.get("SIM_PAYLOAD_MODEL", _payload_model())

# gimbal_link's position offset from base_link, body-frame (matches the old
# gimbal_link <pose> position in the previous single-model design).
LOCAL_OFFSET = np.array([0.1215, 0.0, 0.02])

# Fixed camera mount yaw (see models/m4e_camera/model.sdf) — this is a
# sideways-looking inspection camera, not forward-facing.
MOUNT_YAW = 0.0

SET_POSE_TIMEOUT_MS = 100

# This teleport has zero physical damping (no inertia, unlike a real joint),
# so any noise/timing-jitter in the raw PX4 attitude feed passes straight
# through as visible shake. A light time-constant-based filter (uses actual
# elapsed dt, not a fixed per-message step, since VehicleAttitude arrives
# unevenly) removes that with no lag/oscillation risk -- there's no actuator
# dynamics here for a filter to destabilize, unlike the earlier physical-
# joint attempts.
ATT_TAU = 0.02

# Cap the teleport rate instead of spamming set_pose as fast as possible --
# bursty/irregular call timing was itself a source of jitter.
TELEPORT_HZ = 50.0

# Reject a single attitude sample outright if it implies an angular rate
# faster than any real maneuver could produce -- PX4 running under CPU load
# (this sim shares the host with Gazebo's renderer, physics, etc.) can spit
# out an occasional bad/glitched sample, and an EMA filter has no defense
# against that: it just smooths toward whatever it's given, including one
# bad value, and visibly "sticks" at the wrong angle for a couple hundred ms
# while it chases back out. Confirmed empirically (see conversation) as the
# cause of an intermittent ~45deg stuck-rotation artifact in the video.
MAX_ATT_RATE = math.radians(400)  # rad/s


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _quat_to_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(r):
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


class GimbalStabilizer(Node):
    def __init__(self):
        super().__init__("gimbal_stabilizer_node")

        self._lock = threading.Lock()
        self._drone_pos = None      # np.array([x, y, z])
        self._drone_r = None        # 3x3 rotation matrix
        self._roll = 0.0
        self._pitch = 0.0
        self._att_last_t = None

        # Manual pan/roll/tilt offsets from the GUI sliders, already clamped
        # to GIMBAL_LIMITS on receipt (see _gimbal_cmd_cb).
        self._cmd_pan = 0.0
        self._cmd_roll = 0.0
        self._cmd_tilt = 0.0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._att_cb, px4_qos)
        self.create_subscription(
            PoseStamped, GZ_POSE_TOPIC, self._pose_cb, px4_qos)
        for name, topic in GIMBAL_CMD_TOPICS.items():
            self.create_subscription(
                Float64, topic, lambda msg, n=name: self._gimbal_cmd_cb(n, msg), 1)

        # No gz-transport subscription in this process -- only outgoing
        # set_pose requests. See module docstring for why that split matters.
        self._gz_req_node = gz_transport.Node()

        self._stop = False
        self._teleport_thread = threading.Thread(target=self._teleport_loop, daemon=True)
        self._teleport_thread.start()
        self.get_logger().info("GimbalStabilizer (kinematic) ready")

    def _att_cb(self, msg: VehicleAttitude) -> None:
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))

        now = time.monotonic()
        with self._lock:
            if self._att_last_t is None:
                self._roll, self._pitch, self._att_last_t = roll, pitch, now
                return
            dt = max(1e-4, now - self._att_last_t)
            self._att_last_t = now

            max_jump = MAX_ATT_RATE * dt
            if abs(roll - self._roll) > max_jump or abs(pitch - self._pitch) > max_jump:
                # Outlier -- ignore this sample's value, but the timestamp
                # update above still resets dt so the NEXT sample isn't
                # penalized by an inflated gap.
                return

            alpha = dt / (ATT_TAU + dt)
            self._roll += alpha * (roll - self._roll)
            self._pitch += alpha * (pitch - self._pitch)

    def _gimbal_cmd_cb(self, name: str, msg: Float64) -> None:
        lo, hi = GIMBAL_LIMITS[name]
        val = max(lo, min(hi, float(msg.data)))
        with self._lock:
            if name == "pan":
                self._cmd_pan = val
            elif name == "roll":
                self._cmd_roll = val
            else:
                self._cmd_tilt = val

    def _pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        r = _quat_to_matrix(p.orientation.w, p.orientation.x,
                             p.orientation.y, p.orientation.z)
        with self._lock:
            self._drone_pos, self._drone_r = pos, r

    def _teleport_loop(self) -> None:
        topic = f"/world/{WORLD}/set_pose"
        period = 1.0 / TELEPORT_HZ
        while not self._stop:
            loop_start = time.monotonic()
            with self._lock:
                drone_pos, r_drone = self._drone_pos, self._drone_r
                roll, pitch = self._roll, self._pitch
                cmd_pan, cmd_roll, cmd_tilt = self._cmd_pan, self._cmd_roll, self._cmd_tilt

            if drone_pos is None:
                time.sleep(0.05)
                continue

            # PX4's pitch sign is inverted relative to Gazebo's own pitch for
            # the same physical attitude (NED/FRD vs ENU/FLU) -- confirmed
            # empirically: camera roll was tracking exactly 2x drone pitch
            # (doubling instead of cancelling) until this was flipped to +.
            # Manual pan/roll/tilt setpoints (from the GUI gimbal sliders)
            # are added on top of the auto-level correction terms, same as
            # the old physical gimbal_yaw/roll/tilt joints stacked on top of
            # the fixed mount.
            r_target = (r_drone @ _rot_x(-roll + cmd_roll) @ _rot_y(pitch + cmd_tilt)
                        @ _rot_z(MOUNT_YAW + cmd_pan))
            pos_target = drone_pos + r_drone @ LOCAL_OFFSET
            w, x, y, z = _matrix_to_quat(r_target)

            req = Pose()
            req.name = TARGET_MODEL
            req.position.x, req.position.y, req.position.z = pos_target.tolist()
            req.orientation.w, req.orientation.x = w, x
            req.orientation.y, req.orientation.z = y, z

            self._gz_req_node.request(topic, req, Pose, Boolean, SET_POSE_TIMEOUT_MS)

            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    def destroy_node(self):
        self._stop = True
        self._teleport_thread.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = GimbalStabilizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

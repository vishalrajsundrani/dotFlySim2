#!/usr/bin/env python3
"""
camera_manager — one subscription per camera the operator has switched on.

WHAT IT IS FOR
==============
Rendering in this simulation is demand-driven. A camera sensor is
`always_on=false` in the composed model and its ros_gz_bridge row is
`lazy: true`, so Gazebo renders it only while something is subscribed
(SPIKE-1: 12 payload cameras cost 18.9% CPU switched off against a 17.1%
floor, and 106% with one subscriber).

"Something" is this node. It holds exactly the subscriptions walker has asked
for and no others, which makes "enable a camera" and "subscribe to a camera"
the same act. There is no per-camera switch anywhere in Gazebo to flip.

WHY THE CALLBACK DOES NOTHING
=============================
For most cameras the subscription IS the whole point -- it makes the sensor
render, and RViz (or a perception node, or rosbag) subscribes to the same ROS
topic to actually look at the frames. Copying the image here as well would
double the traffic for no one's benefit.

The exception is the payload lens feeding the wrapper surface: psdk_ros2
publishes ONE main_camera_stream, so whichever payload lens is designated
active is republished onto /drone/camera/active/image_raw, which is what the
bridge's main_camera_out route reads.

CONTROL
=======
    /walker/cameras/enabled   std_msgs/String, a JSON document:
        {"enabled": ["/drone/perception/front/left/image_raw", ...],
         "active":  "/drone/camera/wide/videofhd/image_raw"}

Sent whole rather than as deltas: walker knows the complete desired set, and a
node that rebuilds from the full set cannot drift out of step with the screen
the way an incremental protocol can after one dropped message.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

CONTROL_TOPIC = "/walker/cameras/enabled"
ACTIVE_TOPIC = "/drone/camera/active/image_raw"


class CameraManager(Node):
    def __init__(self) -> None:
        super().__init__("walker_camera_manager")
        self._subs: dict[str, object] = {}
        self._active: str = ""

        # The wrapper's single payload stream. Created once: a publisher with
        # no subscribers costs nothing, and creating it on demand would race
        # the bridge's route coming up.
        self._active_pub = self.create_publisher(Image, ACTIVE_TOPIC,
                                                 qos_profile_sensor_data)

        self.create_subscription(String, CONTROL_TOPIC, self._on_control, 10)
        self.get_logger().info(
            f"camera manager ready; waiting for {CONTROL_TOPIC}")

    # ── control ──────────────────────────────────────────────────────────────

    def _on_control(self, msg: String) -> None:
        try:
            doc = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"ignoring malformed control: {msg.data[:80]}")
            return

        wanted = set(doc.get("enabled", []))
        self._active = doc.get("active", "") or ""

        for topic in list(self._subs):
            if topic not in wanted:
                self.destroy_subscription(self._subs.pop(topic))
                self.get_logger().info(f"off  {topic}")

        for topic in sorted(wanted):
            if topic in self._subs:
                continue
            self._subs[topic] = self.create_subscription(
                Image, topic,
                (lambda m, t=topic: self._on_image(m, t)),
                qos_profile_sensor_data)
            self.get_logger().info(f"on   {topic}")

        self.get_logger().info(
            f"{len(self._subs)} camera(s) rendering"
            + (f"; active payload lens {self._active}" if self._active else ""))

    def _on_image(self, msg: Image, topic: str) -> None:
        # Deliberately almost nothing: see the module docstring.
        if topic and topic == self._active:
            self._active_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

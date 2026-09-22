#!/usr/bin/env python3
"""
M4E camera switcher — activates one camera at a time to reduce compute.

With lazy: true in bridge_config.yaml, ros_gz_bridge only subscribes to a
Gazebo camera topic when a ROS2 subscriber exists. Gazebo then skips rendering
that sensor. This node holds exactly one camera subscription at a time.

Active camera output (raw)       : /drone/camera/active/image_raw
Active camera output (compressed): /drone/camera/active/image_raw/compressed
Switch command                   : ros2 topic pub /drone/camera/select std_msgs/msg/String \
                                     "data: 'wide'" --once
                                   # choices: wide | medium_tele | tele  (default: wide)
Mode command                     : ros2 topic pub /drone/camera/mode std_msgs/msg/String \
                                     "data: 'video_4k'" --once
                                   # choices: photo | video_4k | video_fhd | preview (default: photo)
                                   # photo     -> full-res still sensor, 1 Hz
                                   # video_4k  -> 3840x2160 @ 30fps (DJI spec 4K)
                                   # video_fhd -> 1920x1080 @ 30fps (DJI spec FHD)
                                   # preview   -> 640x360 @ 30fps, lightweight sim-performance tier
Zoom command                     : ros2 topic pub /drone/camera/zoom std_msgs/msg/Float64 \
                                     "data: 5.0" --once
                                   # zoom 1–168; camera switches automatically at 3× and 7×
                                   # within each camera range, the image is digitally cropped

Prefer the /compressed topic for consumers — the raw frames are 60–144 MB each
and will stress DDS transport; JPEG output is typically <1 MB at quality 85.

Gimbal stabilization is handled physically, not here -- see
gui/gimbal_stabilizer.py, which kinematically teleports the standalone
m4e_camera model (models/m4e_camera/model.sdf) to cancel the drone's
roll/pitch before Gazebo renders each frame. This node just relays whatever
Gazebo already rendered.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, String

LENSES_LIST = ('wide', 'medium_tele', 'tele')
MODES_LIST  = ('photo', 'video_4k', 'video_fhd', 'preview')

# Maps a mode to its topic-name tier suffix; 'photo' has no suffix (uses the
# original base topics, unchanged, so existing photo behavior is untouched).
_TIER_BY_MODE = {
    'photo':     None,
    'video_4k':  'video4k',
    'video_fhd': 'videofhd',
    'preview':   'preview',
}

# Photo-mode base topics (unchanged from the original single-tier design).
_PHOTO_TOPICS = {
    'wide':        '/drone/camera/wide/image_raw',
    'medium_tele': '/drone/camera/medium_tele/image_raw',
    'tele':        '/drone/camera/tele/image_raw',
}


def _topic_for(lens: str, mode: str) -> str:
    tier = _TIER_BY_MODE[mode]
    if tier is None:
        return _PHOTO_TOPICS[lens]
    return f'/drone/camera/{lens}/{tier}/image_raw'


# MODES[mode][lens] -> gz-bridged ROS topic for that lens at that resolution tier.
MODES = {
    mode: {lens: _topic_for(lens, mode) for lens in LENSES_LIST}
    for mode in MODES_LIST
}

DEFAULT_LENS = 'wide'
DEFAULT_MODE = 'photo'

# Zoom value at which each lens becomes active (matches GUI breakpoints).
# crop_ratio = ZOOM_BASE[camera] / current_zoom
# Lens-keyed only — zoom/lens-breakpoint behavior is the same regardless of
# which resolution tier (mode) is currently streaming.
ZOOM_BASE = {
    'wide':        1.0,
    'medium_tele': 3.0,
    'tele':        7.0,
}

JPEG_QUALITY = 85


class CameraSwitcher(Node):
    def __init__(self):
        super().__init__('camera_switcher')

        self._active_name = DEFAULT_LENS
        self._active_mode = DEFAULT_MODE
        self._active_sub  = None
        self._zoom        = 1.0
        self._crop_ratio  = 1.0

        self._pub = self.create_publisher(Image, '/drone/camera/active/image_raw', 1)
        self._compressed_pub = self.create_publisher(
            CompressedImage, '/drone/camera/active/image_raw/compressed', 1)
        # Tells GstCameraSystem (selective mode, see server.config) which gz
        # camera topic to stream to QGroundControl on UDP 5600.
        self._gst_select_pub = self.create_publisher(
            String, '/drone/camera/gst_select', 1)

        self._cmd_sub = self.create_subscription(
            String,  '/drone/camera/select', self._on_select, 10)
        self._mode_sub = self.create_subscription(
            String,  '/drone/camera/mode',   self._on_mode,   10)
        self._zoom_sub = self.create_subscription(
            Float64, '/drone/camera/zoom',   self._on_zoom,   10)

        self._resubscribe()
        self.get_logger().info(
            f"Camera switcher ready. Active: lens='{self._active_name}' "
            f"mode='{self._active_mode}'. Zoom: {self._zoom:.1f}×. "
            f"Compressed output on /drone/camera/active/image_raw/compressed.")

    # ── camera / mode selection ──────────────────────────────────────────────

    def _on_select(self, msg: String) -> None:
        name = msg.data.strip()
        if name not in LENSES_LIST:
            self.get_logger().warn(
                f"Unknown lens '{name}'. Choices: {LENSES_LIST}")
            return
        if name == self._active_name:
            return
        self._active_name = name
        self._resubscribe()

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip()
        if mode not in MODES:
            self.get_logger().warn(
                f"Unknown mode '{mode}'. Choices: {MODES_LIST}")
            return
        if mode == self._active_mode:
            return
        self._active_mode = mode
        self._resubscribe()

    def _resubscribe(self) -> None:
        if self._active_sub is not None:
            self.destroy_subscription(self._active_sub)
            self._active_sub = None

        topic = MODES[self._active_mode][self._active_name]
        self._active_sub = self.create_subscription(
            Image, topic, self._relay, 1)
        self._gst_select_pub.publish(String(data=topic))
        self._update_crop()
        self.get_logger().info(
            f"Active → lens={self._active_name} mode={self._active_mode}  "
            f"({topic})  zoom={self._zoom:.1f}×")

    # ── zoom / digital crop ──────────────────────────────────────────────────

    def _on_zoom(self, msg: Float64) -> None:
        self._zoom = max(1.0, float(msg.data))
        self._update_crop()

    def _update_crop(self) -> None:
        base = ZOOM_BASE.get(self._active_name, 1.0)
        self._crop_ratio = min(1.0, base / self._zoom)

    def _apply_zoom(self, msg: Image):
        """Return (numpy_rgb_array, Image_msg) after digital zoom crop."""
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        if self._crop_ratio >= 0.999:
            return arr, msg

        h, w = arr.shape[:2]
        ch = max(1, int(h * self._crop_ratio))
        cw = max(1, int(w * self._crop_ratio))
        y0 = (h - ch) // 2
        x0 = (w - cw) // 2
        cropped = arr[y0:y0 + ch, x0:x0 + cw]
        out_arr = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

        out_msg = Image()
        out_msg.header    = msg.header
        out_msg.height    = h
        out_msg.width     = w
        out_msg.encoding  = msg.encoding
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.step      = w * 3  # R8G8B8
        out_msg.data      = out_arr.tobytes()
        return out_arr, out_msg

    # ── relay ────────────────────────────────────────────────────────────────

    def _relay(self, msg: Image) -> None:
        raw_subs = self._pub.get_subscription_count()
        cmp_subs = self._compressed_pub.get_subscription_count()

        if raw_subs == 0 and cmp_subs == 0:
            return

        arr, out_msg = self._apply_zoom(msg)

        if raw_subs > 0:
            self._pub.publish(out_msg)

        if cmp_subs == 0:
            return

        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            return

        cimg = CompressedImage()
        cimg.header = msg.header
        cimg.format = 'jpeg'
        cimg.data   = buf.tobytes()
        self._compressed_pub.publish(cimg)


def main():
    rclpy.init()
    node = CameraSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

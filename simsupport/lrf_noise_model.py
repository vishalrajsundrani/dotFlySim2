#!/usr/bin/env python3
"""
DJI M4E LRF range-dependent noise model.

Gazebo's SDF only supports fixed Gaussian noise; the real DJI M4E
rangefinder has range-dependent accuracy:

    sigma(D) = 0.2 + 0.0015 x D  metres  (DJI published spec)

models/m4e_camera/model.sdf's lrf sensor has its <noise> block removed so
Gazebo publishes a clean range on /drone/lrf/range (bridged via
bridge.yaml). This node applies the real DJI accuracy model on top of that
clean signal.

Subscribe : /drone/lrf/range        (sensor_msgs/LaserScan, clean from Gazebo)
Publish   : /drone/lrf/range_noisy  (sensor_msgs/LaserScan, DJI M4E noise model)

Any downstream consumer that needs realistic LRF behavior (EKF fusion,
terrain-following, logging) should subscribe to /drone/lrf/range_noisy,
not the raw /drone/lrf/range topic.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# DJI M4E ranging accuracy spec: +/-(NOISE_FIXED + NOISE_RATE x D) metres
_NOISE_FIXED = 0.2     # metres — constant component
_NOISE_RATE  = 0.0015  # metres per metre of range
_BLIND_ZONE  = 1.0     # metres — sensor returns inf below this
_MAX_RANGE   = 1800.0  # metres


class LrfNoiseModel(Node):
    def __init__(self):
        super().__init__('lrf_noise_model')
        self.sub = self.create_subscription(
            LaserScan, '/drone/lrf/range', self._cb, 10)
        self.pub = self.create_publisher(
            LaserScan, '/drone/lrf/range_noisy', 10)
        self.get_logger().info(
            'LRF noise model active  sigma(D) = 0.20 + 0.0015*D  [DJI M4E spec]')

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header          = msg.header
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.intensities     = msg.intensities

        noisy = []
        for r in msg.ranges:
            if not math.isfinite(r) or r < _BLIND_ZONE or r > _MAX_RANGE:
                noisy.append(float('inf'))
                continue
            sigma = _NOISE_FIXED + _NOISE_RATE * r
            r_noisy = r + float(np.random.normal(0.0, sigma))
            noisy.append(float(np.clip(r_noisy, _BLIND_ZONE, _MAX_RANGE)))

        out.ranges = noisy
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LrfNoiseModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

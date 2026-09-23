#!/usr/bin/env python3
"""
Regenerate config/rviz/cameras.rviz from the current composition.

Run this after adding a camera to a drone's model. The file it writes carries
one Image display per camera, all DISABLED -- see the header it generates for
why that matters (an enabled display subscribes, and subscribing is what makes
Gazebo render the lens, so the config doubles as a camera switch).

    python3 tools/gen_rviz_cameras.py > config/rviz/cameras.rviz
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "walkerd"))
sys.path.insert(0, "/home/developer/ws/src/walkerd")
import cameras  # noqa: E402

if __name__ == "__main__":
    print(json.dumps(cameras.catalogue(), indent=2))

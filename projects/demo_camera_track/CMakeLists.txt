cmake_minimum_required(VERSION 3.16)
project(demo_camera_track)

# ─────────────────────────────────────────────────────────────────────────────
# Discovered through the demo/ mount (see demo/README.md). Launch it from the
# HOST -- the launcher builds inside the container when needed, and turns the
# bridge's video route on, which this project cannot do for itself:
#
#     ./demo/run_demo.sh camera_track
#
# By hand, inside the container, that is:
#
#     cd ~/uav_ws && colcon build --packages-select demo_camera_track
#     source ~/uav_ws/install/setup.bash
#     ros2 run demo_camera_track camera_track
#
# The entry point is src/main.cpp -- one file per project, by convention here.
#
# NO OpenCV, NO cv_bridge. The detector walks the sensor_msgs/Image buffer
# itself (see the header comment in src/main.cpp): a colour window and a
# centroid do not need an imaging library, and not depending on one keeps this
# package buildable in an image that never installed vision_opencv.
#
# sensor_msgs carries both halves of this demo -- the Image it reads and the Joy
# it steers with -- and geometry_msgs is only for the PointStamped that reports
# the loop's own input on ~/detection.
# ─────────────────────────────────────────────────────────────────────────────

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# No px4_msgs anywhere: this project is restricted to the PSDK wrapper surface.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(camera_track src/main.cpp)
ament_target_dependencies(camera_track
  rclcpp geometry_msgs psdk_interfaces sensor_msgs std_msgs std_srvs)

install(TARGETS camera_track
  DESTINATION lib/${PROJECT_NAME}
)

# The RViz config ships with the project so `rviz2 -d $(ros2 pkg prefix
# demo_camera_track)/share/demo_camera_track/rviz/camera_track.rviz` works from
# an install space. run_demo.sh opens the SOURCE copy instead -- demo/ is a live
# mount, so editing the config there needs no rebuild.
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

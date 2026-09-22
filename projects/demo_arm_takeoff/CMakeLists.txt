cmake_minimum_required(VERSION 3.16)
project(demo_arm_takeoff)

# ─────────────────────────────────────────────────────────────────────────────
# This package is discovered through the demo/ mount, exactly like the RViz
# panels are discovered through gui/: run.sh maps the host's demo/ directory to
# ~/uav_ws/src/demo, colcon recurses into it, and no Dockerfile edit is needed
# to add or change a demo.
#
# You are not expected to run this build by hand. From the HOST:
#
#     ./demo/run_demo.sh arm_takeoff
#
# which builds this package inside the hardcoded container if the executable is
# missing, then runs it. The equivalent by hand, inside the container, is
#
#     cd ~/uav_ws && colcon build --packages-select demo_arm_takeoff
#     source ~/uav_ws/install/setup.bash
#     ros2 run demo_arm_takeoff arm_takeoff
#
# The entry point is src/main.cpp -- one file per project, by convention here.
# ─────────────────────────────────────────────────────────────────────────────

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# No px4_msgs anywhere: this project is restricted to the PSDK wrapper surface,
# so its only message dependency is the vendored psdk_interfaces.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(arm_takeoff src/main.cpp)
ament_target_dependencies(arm_takeoff
  rclcpp psdk_interfaces std_msgs std_srvs geometry_msgs sensor_msgs)

install(TARGETS arm_takeoff
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

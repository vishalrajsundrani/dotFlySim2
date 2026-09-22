cmake_minimum_required(VERSION 3.16)
project(demo_square_mission)

# ─────────────────────────────────────────────────────────────────────────────
# Discovered through the demo/ mount (see demo/README.md). Launch it from the
# HOST -- the launcher builds inside the container when needed:
#
#     ./demo/run_demo.sh square_mission
#
# By hand, inside the container, that is:
#
#     cd ~/uav_ws && colcon build --packages-select demo_orbit_mission
#     source ~/uav_ws/install/setup.bash
#     ros2 run demo_orbit_mission square_mission
#
# The entry point is src/main.cpp -- one file per project, by convention here.
#
# psdk_interfaces carries every message and service this mission uses. It is
# built into the image already (Dockerfile STEP 10), so nothing extra has to be
# fetched.
# ─────────────────────────────────────────────────────────────────────────────

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# No px4_msgs anywhere: this project is restricted to the PSDK wrapper surface.
# sensor_msgs is here for the Joy sticks, geometry_msgs for the ENU velocity and
# the optional attitude quaternion.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(square_mission src/main.cpp)
ament_target_dependencies(square_mission rclcpp geometry_msgs psdk_interfaces sensor_msgs std_msgs std_srvs)

install(TARGETS square_mission
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

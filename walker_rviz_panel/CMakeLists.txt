cmake_minimum_required(VERSION 3.16)
project(walker_rviz_panel)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# AUTOMOC, because the panel declares Q_OBJECT and includes its own .moc at the
# bottom -- the signal/slot machinery is generated, not written.
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME} SHARED src/walker_panel.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} pluginlib rclcpp rviz_common std_msgs)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()

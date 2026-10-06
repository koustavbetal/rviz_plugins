cmake_minimum_required(VERSION 3.8)
project(polygon_editor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(opennav_coverage_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  include/polygon_editor/polygon_editor_panel.hpp
  src/polygon_store.cpp
  src/polygon_edit_tool.cpp
  src/polygon_editor_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  rclcpp_action
  tf2_ros
  rviz_common
  rviz_rendering
  pluginlib
  interactive_markers
  visualization_msgs
  geometry_msgs
  opennav_coverage_msgs
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

target_compile_definitions(${PROJECT_NAME} PRIVATE "POLYGON_EDITOR_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files (yaml + pgm)
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp rclcpp rclcpp_action rviz_common rviz_rendering pluginlib interactive_markers
  visualization_msgs geometry_msgs opennav_coverage_msgs
)
ament_export_targets(export_${PROJECT_NAME})
ament_package()

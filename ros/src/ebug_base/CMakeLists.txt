cmake_minimum_required(VERSION 3.8)
project(ebug_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Must explicity list sub-dependency for grid_map_rviz_plugin
# ROS doesn't seem to manage this hierarchy properly...
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(image_proc REQUIRED)
find_package(pluginlib REQUIRED)

find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)
find_package(grid_map_visualization REQUIRED)

grid_map_package()

include_directories(include)

rosidl_generate_interfaces (${PROJECT_NAME}
  "srv/ComputeTarget.srv"
  "msg/RobotPose.msg"
  "msg/ControlCommand.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")




add_library(byte_rectifier_component SHARED src/byte_rectifier.cpp)
ament_target_dependencies(byte_rectifier_component rclcpp rclcpp_components sensor_msgs)

add_library(transform_converter_component SHARED src/transform_converter.cpp)
ament_target_dependencies(transform_converter_component rclcpp rclcpp_components sensor_msgs tf2 tf2_msgs)

add_library(jpeg_republisher_component SHARED src/jpeg_republisher.cpp)
ament_target_dependencies(jpeg_republisher_component rclcpp rclcpp_components sensor_msgs image_transport pluginlib)

add_library(gridmap_controller_component SHARED src/gridmap_controller.cpp)
target_link_libraries(gridmap_controller_component "${cpp_typesupport_target}")
ament_target_dependencies(gridmap_controller_component rclcpp rclcpp_components grid_map_cmake_helpers grid_map grid_map_core grid_map_ros grid_map_rviz_plugin grid_map_visualization)


ament_export_dependencies(rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs message_filters image_transport compressed_image_transport image_proc pluginlib grid_map_cmake_helpers grid_map grid_map_core grid_map_ros grid_map_rviz_plugin grid_map_visualization)

rclcpp_components_register_node(
  byte_rectifier_component
  PLUGIN "ebug::ByteRectifier"
  EXECUTABLE byte_rectifier
)

rclcpp_components_register_node(
  transform_converter_component
  PLUGIN "ebug::TransformConverter"
  EXECUTABLE transform_converter
)

rclcpp_components_register_node(
  jpeg_republisher_component
  PLUGIN "ebug::JpegRepublisher"
  EXECUTABLE jpeg_republisher
)

rclcpp_components_register_node(
  gridmap_controller_component
  PLUGIN "ebug::GridmapController"
  EXECUTABLE gridmap_controller
)


# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  byte_rectifier_component
  transform_converter_component
  jpeg_republisher_component
  gridmap_controller_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

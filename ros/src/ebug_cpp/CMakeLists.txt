cmake_minimum_required(VERSION 3.8)
project(ebug_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(image_proc REQUIRED)
find_package(pluginlib REQUIRED)

find_package(ebug_interfaces REQUIRED)

include_directories(include)

add_library(camera_controller_component SHARED  src/camera_controller.cpp)
ament_target_dependencies(camera_controller_component rclcpp rclcpp_components sensor_msgs message_filters)

add_library(byte_rectifier_component SHARED src/byte_rectifier.cpp)
ament_target_dependencies(byte_rectifier_component rclcpp rclcpp_components sensor_msgs)

add_library(jpeg_republisher_component SHARED src/jpeg_republisher.cpp)
ament_target_dependencies(jpeg_republisher_component rclcpp rclcpp_components sensor_msgs image_transport pluginlib)

ament_export_dependencies(rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs message_filters image_transport compressed_image_transport image_proc pluginlib)

rclcpp_components_register_node(
  camera_controller_component
  PLUGIN "ebug::CameraController"
  EXECUTABLE camera_controller
)

rclcpp_components_register_node(
  byte_rectifier_component
  PLUGIN "ebug::ByteRectifier"
  EXECUTABLE byte_rectifier
)

rclcpp_components_register_node(
  jpeg_republisher_component
  PLUGIN "ebug::JpegRepublisher"
  EXECUTABLE jpeg_republisher
)

# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  camera_controller_component
  byte_rectifier_component
  jpeg_republisher_component

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

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces (${PROJECT_NAME}
#   "srv/ComputeTarget.srv"
#   "msg/RobotPose.msg"
#   "msg/ControlCommand.msg"
#   DEPENDENCIES geometry_msgs
# )

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ebug_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(ebug_interfaces REQUIRED)


add_library(camera_controller_component src/camera_controller.cpp)
ament_target_dependencies(camera_controller_component rclcpp rclcpp_components sensor_msgs message_filters)

add_library(byte_rectifier_component src/byte_rectifier.cpp)
ament_target_dependencies(byte_rectifier_component rclcpp rclcpp_components sensor_msgs message_filters)

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

install(TARGETS
  camera_controller_component
  byte_rectifier_component

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

cmake_minimum_required(VERSION 3.8)
project(ebug_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(pluginlib REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)


add_library(pololu_hardware SHARED src/pololu_diff_drive.cpp src/romi_i2c_bridge.cpp)

target_include_directories(
  pololu_hardware
  PRIVATE
  include
)
ament_target_dependencies(
  pololu_hardware

  rclcpp
  pluginlib
  hardware_interface
  controller_manager
)

pluginlib_export_plugin_description_file(hardware_interface ebug_hardware.xml)

ament_export_libraries(
  pololu_hardware
)

ament_export_dependencies(
  rclcpp
  pluginlib
  hardware_interface
  controller_manager
)

ament_package()



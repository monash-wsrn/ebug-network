// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ebug_base:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__ROBOT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define EBUG_BASE__MSG__DETAIL__ROBOT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ebug_base/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ebug_base/msg/detail/robot_pose__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ebug_base
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ebug_base
cdr_serialize(
  const ebug_base::msg::RobotPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ebug_base
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ebug_base::msg::RobotPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ebug_base
get_serialized_size(
  const ebug_base::msg::RobotPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ebug_base
max_serialized_size_RobotPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ebug_base

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ebug_base
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ebug_base, msg, RobotPose)();

#ifdef __cplusplus
}
#endif

#endif  // EBUG_BASE__MSG__DETAIL__ROBOT_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

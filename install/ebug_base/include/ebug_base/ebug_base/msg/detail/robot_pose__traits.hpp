// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ebug_base:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_
#define EBUG_BASE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ebug_base/msg/detail/robot_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace ebug_base
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ebug_base

namespace rosidl_generator_traits
{

[[deprecated("use ebug_base::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ebug_base::msg::RobotPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebug_base::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebug_base::msg::to_yaml() instead")]]
inline std::string to_yaml(const ebug_base::msg::RobotPose & msg)
{
  return ebug_base::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ebug_base::msg::RobotPose>()
{
  return "ebug_base::msg::RobotPose";
}

template<>
inline const char * name<ebug_base::msg::RobotPose>()
{
  return "ebug_base/msg/RobotPose";
}

template<>
struct has_fixed_size<ebug_base::msg::RobotPose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ebug_base::msg::RobotPose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ebug_base::msg::RobotPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EBUG_BASE__MSG__DETAIL__ROBOT_POSE__TRAITS_HPP_

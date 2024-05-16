// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_
#define EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ebug_base/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace ebug_base
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: control
  {
    out << "control: ";
    to_flow_style_yaml(msg.control, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    to_flow_style_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control:\n";
    to_block_style_yaml(msg.control, out, indentation + 2);
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color:\n";
    to_block_style_yaml(msg.color, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlCommand & msg, bool use_flow_style = false)
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
  const ebug_base::msg::ControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebug_base::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebug_base::msg::to_yaml() instead")]]
inline std::string to_yaml(const ebug_base::msg::ControlCommand & msg)
{
  return ebug_base::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ebug_base::msg::ControlCommand>()
{
  return "ebug_base::msg::ControlCommand";
}

template<>
inline const char * name<ebug_base::msg::ControlCommand>()
{
  return "ebug_base/msg/ControlCommand";
}

template<>
struct has_fixed_size<ebug_base::msg::ControlCommand>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<ebug_base::msg::ControlCommand>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<ebug_base::msg::ControlCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_

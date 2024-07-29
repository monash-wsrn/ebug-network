// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__TRAITS_HPP_
#define EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ebug_base/srv/detail/compute_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace ebug_base
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeTarget_Request & msg,
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
  const ComputeTarget_Request & msg,
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

inline std::string to_yaml(const ComputeTarget_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ebug_base

namespace rosidl_generator_traits
{

[[deprecated("use ebug_base::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ebug_base::srv::ComputeTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebug_base::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebug_base::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebug_base::srv::ComputeTarget_Request & msg)
{
  return ebug_base::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebug_base::srv::ComputeTarget_Request>()
{
  return "ebug_base::srv::ComputeTarget_Request";
}

template<>
inline const char * name<ebug_base::srv::ComputeTarget_Request>()
{
  return "ebug_base/srv/ComputeTarget_Request";
}

template<>
struct has_fixed_size<ebug_base::srv::ComputeTarget_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ebug_base::srv::ComputeTarget_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ebug_base::srv::ComputeTarget_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace ebug_base
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeTarget_Response & msg,
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
  const ComputeTarget_Response & msg,
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

inline std::string to_yaml(const ComputeTarget_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ebug_base

namespace rosidl_generator_traits
{

[[deprecated("use ebug_base::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ebug_base::srv::ComputeTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ebug_base::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ebug_base::srv::to_yaml() instead")]]
inline std::string to_yaml(const ebug_base::srv::ComputeTarget_Response & msg)
{
  return ebug_base::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ebug_base::srv::ComputeTarget_Response>()
{
  return "ebug_base::srv::ComputeTarget_Response";
}

template<>
inline const char * name<ebug_base::srv::ComputeTarget_Response>()
{
  return "ebug_base/srv/ComputeTarget_Response";
}

template<>
struct has_fixed_size<ebug_base::srv::ComputeTarget_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<ebug_base::srv::ComputeTarget_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<ebug_base::srv::ComputeTarget_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ebug_base::srv::ComputeTarget>()
{
  return "ebug_base::srv::ComputeTarget";
}

template<>
inline const char * name<ebug_base::srv::ComputeTarget>()
{
  return "ebug_base/srv/ComputeTarget";
}

template<>
struct has_fixed_size<ebug_base::srv::ComputeTarget>
  : std::integral_constant<
    bool,
    has_fixed_size<ebug_base::srv::ComputeTarget_Request>::value &&
    has_fixed_size<ebug_base::srv::ComputeTarget_Response>::value
  >
{
};

template<>
struct has_bounded_size<ebug_base::srv::ComputeTarget>
  : std::integral_constant<
    bool,
    has_bounded_size<ebug_base::srv::ComputeTarget_Request>::value &&
    has_bounded_size<ebug_base::srv::ComputeTarget_Response>::value
  >
{
};

template<>
struct is_service<ebug_base::srv::ComputeTarget>
  : std::true_type
{
};

template<>
struct is_service_request<ebug_base::srv::ComputeTarget_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ebug_base::srv::ComputeTarget_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__BUILDER_HPP_
#define EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ebug_base/srv/detail/compute_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ebug_base
{

namespace srv
{

namespace builder
{

class Init_ComputeTarget_Request_pose
{
public:
  explicit Init_ComputeTarget_Request_pose(::ebug_base::srv::ComputeTarget_Request & msg)
  : msg_(msg)
  {}
  ::ebug_base::srv::ComputeTarget_Request pose(::ebug_base::srv::ComputeTarget_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebug_base::srv::ComputeTarget_Request msg_;
};

class Init_ComputeTarget_Request_robot_id
{
public:
  Init_ComputeTarget_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeTarget_Request_pose robot_id(::ebug_base::srv::ComputeTarget_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_ComputeTarget_Request_pose(msg_);
  }

private:
  ::ebug_base::srv::ComputeTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebug_base::srv::ComputeTarget_Request>()
{
  return ebug_base::srv::builder::Init_ComputeTarget_Request_robot_id();
}

}  // namespace ebug_base


namespace ebug_base
{

namespace srv
{

namespace builder
{

class Init_ComputeTarget_Response_color
{
public:
  explicit Init_ComputeTarget_Response_color(::ebug_base::srv::ComputeTarget_Response & msg)
  : msg_(msg)
  {}
  ::ebug_base::srv::ComputeTarget_Response color(::ebug_base::srv::ComputeTarget_Response::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebug_base::srv::ComputeTarget_Response msg_;
};

class Init_ComputeTarget_Response_control
{
public:
  Init_ComputeTarget_Response_control()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeTarget_Response_color control(::ebug_base::srv::ComputeTarget_Response::_control_type arg)
  {
    msg_.control = std::move(arg);
    return Init_ComputeTarget_Response_color(msg_);
  }

private:
  ::ebug_base::srv::ComputeTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebug_base::srv::ComputeTarget_Response>()
{
  return ebug_base::srv::builder::Init_ComputeTarget_Response_control();
}

}  // namespace ebug_base

#endif  // EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ebug_base:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_
#define EBUG_BASE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ebug_base/msg/detail/robot_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ebug_base
{

namespace msg
{

namespace builder
{

class Init_RobotPose_pose
{
public:
  explicit Init_RobotPose_pose(::ebug_base::msg::RobotPose & msg)
  : msg_(msg)
  {}
  ::ebug_base::msg::RobotPose pose(::ebug_base::msg::RobotPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebug_base::msg::RobotPose msg_;
};

class Init_RobotPose_robot_id
{
public:
  Init_RobotPose_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPose_pose robot_id(::ebug_base::msg::RobotPose::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotPose_pose(msg_);
  }

private:
  ::ebug_base::msg::RobotPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebug_base::msg::RobotPose>()
{
  return ebug_base::msg::builder::Init_RobotPose_robot_id();
}

}  // namespace ebug_base

#endif  // EBUG_BASE__MSG__DETAIL__ROBOT_POSE__BUILDER_HPP_

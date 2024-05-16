// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
#define EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ebug_base/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ebug_base
{

namespace msg
{

namespace builder
{

class Init_ControlCommand_color
{
public:
  explicit Init_ControlCommand_color(::ebug_base::msg::ControlCommand & msg)
  : msg_(msg)
  {}
  ::ebug_base::msg::ControlCommand color(::ebug_base::msg::ControlCommand::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ebug_base::msg::ControlCommand msg_;
};

class Init_ControlCommand_control
{
public:
  Init_ControlCommand_control()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_color control(::ebug_base::msg::ControlCommand::_control_type arg)
  {
    msg_.control = std::move(arg);
    return Init_ControlCommand_color(msg_);
  }

private:
  ::ebug_base::msg::ControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ebug_base::msg::ControlCommand>()
{
  return ebug_base::msg::builder::Init_ControlCommand_control();
}

}  // namespace ebug_base

#endif  // EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

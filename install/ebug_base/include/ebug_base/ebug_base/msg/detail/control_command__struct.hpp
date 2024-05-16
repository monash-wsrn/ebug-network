// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_
#define EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ebug_base__msg__ControlCommand __attribute__((deprecated))
#else
# define DEPRECATED__ebug_base__msg__ControlCommand __declspec(deprecated)
#endif

namespace ebug_base
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlCommand_
{
  using Type = ControlCommand_<ContainerAllocator>;

  explicit ControlCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control(_init),
    color(_init)
  {
    (void)_init;
  }

  explicit ControlCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control(_alloc, _init),
    color(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _control_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _control_type control;
  using _color_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _color_type color;

  // setters for named parameter idiom
  Type & set__control(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->control = _arg;
    return *this;
  }
  Type & set__color(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ebug_base::msg::ControlCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebug_base::msg::ControlCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebug_base::msg::ControlCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebug_base::msg::ControlCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebug_base__msg__ControlCommand
    std::shared_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebug_base__msg__ControlCommand
    std::shared_ptr<ebug_base::msg::ControlCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlCommand_ & other) const
  {
    if (this->control != other.control) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlCommand_

// alias to use template instance with default allocator
using ControlCommand =
  ebug_base::msg::ControlCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ebug_base

#endif  // EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_

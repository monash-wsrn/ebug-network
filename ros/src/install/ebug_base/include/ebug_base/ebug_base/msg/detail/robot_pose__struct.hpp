// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ebug_base:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_HPP_
#define EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ebug_base__msg__RobotPose __attribute__((deprecated))
#else
# define DEPRECATED__ebug_base__msg__RobotPose __declspec(deprecated)
#endif

namespace ebug_base
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPose_
{
  using Type = RobotPose_<ContainerAllocator>;

  explicit RobotPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  explicit RobotPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ebug_base::msg::RobotPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebug_base::msg::RobotPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebug_base::msg::RobotPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebug_base::msg::RobotPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebug_base::msg::RobotPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebug_base::msg::RobotPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebug_base::msg::RobotPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebug_base::msg::RobotPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebug_base::msg::RobotPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebug_base::msg::RobotPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebug_base__msg__RobotPose
    std::shared_ptr<ebug_base::msg::RobotPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebug_base__msg__RobotPose
    std::shared_ptr<ebug_base::msg::RobotPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPose_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPose_

// alias to use template instance with default allocator
using RobotPose =
  ebug_base::msg::RobotPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ebug_base

#endif  // EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_HPP_
#define EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_HPP_

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
# define DEPRECATED__ebug_base__srv__ComputeTarget_Request __attribute__((deprecated))
#else
# define DEPRECATED__ebug_base__srv__ComputeTarget_Request __declspec(deprecated)
#endif

namespace ebug_base
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeTarget_Request_
{
  using Type = ComputeTarget_Request_<ContainerAllocator>;

  explicit ComputeTarget_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
    }
  }

  explicit ComputeTarget_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebug_base__srv__ComputeTarget_Request
    std::shared_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebug_base__srv__ComputeTarget_Request
    std::shared_ptr<ebug_base::srv::ComputeTarget_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeTarget_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeTarget_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeTarget_Request_

// alias to use template instance with default allocator
using ComputeTarget_Request =
  ebug_base::srv::ComputeTarget_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebug_base


// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ebug_base__srv__ComputeTarget_Response __attribute__((deprecated))
#else
# define DEPRECATED__ebug_base__srv__ComputeTarget_Response __declspec(deprecated)
#endif

namespace ebug_base
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeTarget_Response_
{
  using Type = ComputeTarget_Response_<ContainerAllocator>;

  explicit ComputeTarget_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : control(_init),
    color(_init)
  {
    (void)_init;
  }

  explicit ComputeTarget_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ebug_base__srv__ComputeTarget_Response
    std::shared_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ebug_base__srv__ComputeTarget_Response
    std::shared_ptr<ebug_base::srv::ComputeTarget_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeTarget_Response_ & other) const
  {
    if (this->control != other.control) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeTarget_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeTarget_Response_

// alias to use template instance with default allocator
using ComputeTarget_Response =
  ebug_base::srv::ComputeTarget_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ebug_base

namespace ebug_base
{

namespace srv
{

struct ComputeTarget
{
  using Request = ebug_base::srv::ComputeTarget_Request;
  using Response = ebug_base::srv::ComputeTarget_Response;
};

}  // namespace srv

}  // namespace ebug_base

#endif  // EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_HPP_

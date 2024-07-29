// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ebug_base:msg/RobotPose.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_H_
#define EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"

/// Struct defined in msg/RobotPose in the package ebug_base.
typedef struct ebug_base__msg__RobotPose
{
  rosidl_runtime_c__String robot_id;
  geometry_msgs__msg__PoseWithCovariance pose;
} ebug_base__msg__RobotPose;

// Struct for a sequence of ebug_base__msg__RobotPose.
typedef struct ebug_base__msg__RobotPose__Sequence
{
  ebug_base__msg__RobotPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebug_base__msg__RobotPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EBUG_BASE__MSG__DETAIL__ROBOT_POSE__STRUCT_H_

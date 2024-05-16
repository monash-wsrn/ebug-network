// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_H_
#define EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_H_

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

/// Struct defined in srv/ComputeTarget in the package ebug_base.
typedef struct ebug_base__srv__ComputeTarget_Request
{
  rosidl_runtime_c__String robot_id;
  geometry_msgs__msg__PoseWithCovariance pose;
} ebug_base__srv__ComputeTarget_Request;

// Struct for a sequence of ebug_base__srv__ComputeTarget_Request.
typedef struct ebug_base__srv__ComputeTarget_Request__Sequence
{
  ebug_base__srv__ComputeTarget_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebug_base__srv__ComputeTarget_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/ComputeTarget in the package ebug_base.
typedef struct ebug_base__srv__ComputeTarget_Response
{
  geometry_msgs__msg__Twist control;
  geometry_msgs__msg__Vector3 color;
} ebug_base__srv__ComputeTarget_Response;

// Struct for a sequence of ebug_base__srv__ComputeTarget_Response.
typedef struct ebug_base__srv__ComputeTarget_Response__Sequence
{
  ebug_base__srv__ComputeTarget_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebug_base__srv__ComputeTarget_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__STRUCT_H_

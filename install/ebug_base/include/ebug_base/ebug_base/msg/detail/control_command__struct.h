// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
#define EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'control'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'color'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ControlCommand in the package ebug_base.
typedef struct ebug_base__msg__ControlCommand
{
  geometry_msgs__msg__Twist control;
  geometry_msgs__msg__Vector3 color;
} ebug_base__msg__ControlCommand;

// Struct for a sequence of ebug_base__msg__ControlCommand.
typedef struct ebug_base__msg__ControlCommand__Sequence
{
  ebug_base__msg__ControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ebug_base__msg__ControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EBUG_BASE__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_

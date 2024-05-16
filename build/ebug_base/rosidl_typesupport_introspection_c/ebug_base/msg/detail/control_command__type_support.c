// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ebug_base/msg/detail/control_command__rosidl_typesupport_introspection_c.h"
#include "ebug_base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ebug_base/msg/detail/control_command__functions.h"
#include "ebug_base/msg/detail/control_command__struct.h"


// Include directives for member types
// Member `control`
#include "geometry_msgs/msg/twist.h"
// Member `control`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `color`
#include "geometry_msgs/msg/vector3.h"
// Member `color`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ebug_base__msg__ControlCommand__init(message_memory);
}

void ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_fini_function(void * message_memory)
{
  ebug_base__msg__ControlCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_member_array[2] = {
  {
    "control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebug_base__msg__ControlCommand, control),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "color",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebug_base__msg__ControlCommand, color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_members = {
  "ebug_base__msg",  // message namespace
  "ControlCommand",  // message name
  2,  // number of fields
  sizeof(ebug_base__msg__ControlCommand),
  ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_member_array,  // message members
  ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_type_support_handle = {
  0,
  &ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebug_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, msg, ControlCommand)() {
  ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_type_support_handle.typesupport_identifier) {
    ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ebug_base__msg__ControlCommand__rosidl_typesupport_introspection_c__ControlCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

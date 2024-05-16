// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ebug_base/srv/detail/compute_target__rosidl_typesupport_introspection_c.h"
#include "ebug_base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ebug_base/srv/detail/compute_target__functions.h"
#include "ebug_base/srv/detail/compute_target__struct.h"


// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose_with_covariance.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ebug_base__srv__ComputeTarget_Request__init(message_memory);
}

void ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_fini_function(void * message_memory)
{
  ebug_base__srv__ComputeTarget_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_member_array[2] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebug_base__srv__ComputeTarget_Request, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebug_base__srv__ComputeTarget_Request, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_members = {
  "ebug_base__srv",  // message namespace
  "ComputeTarget_Request",  // message name
  2,  // number of fields
  sizeof(ebug_base__srv__ComputeTarget_Request),
  ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_member_array,  // message members
  ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_type_support_handle = {
  0,
  &ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebug_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Request)() {
  ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovariance)();
  if (!ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_type_support_handle.typesupport_identifier) {
    ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ebug_base__srv__ComputeTarget_Request__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ebug_base/srv/detail/compute_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ebug_base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ebug_base/srv/detail/compute_target__functions.h"
// already included above
// #include "ebug_base/srv/detail/compute_target__struct.h"


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

void ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ebug_base__srv__ComputeTarget_Response__init(message_memory);
}

void ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_fini_function(void * message_memory)
{
  ebug_base__srv__ComputeTarget_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_member_array[2] = {
  {
    "control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ebug_base__srv__ComputeTarget_Response, control),  // bytes offset in struct
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
    offsetof(ebug_base__srv__ComputeTarget_Response, color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_members = {
  "ebug_base__srv",  // message namespace
  "ComputeTarget_Response",  // message name
  2,  // number of fields
  sizeof(ebug_base__srv__ComputeTarget_Response),
  ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_member_array,  // message members
  ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_type_support_handle = {
  0,
  &ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebug_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Response)() {
  ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_type_support_handle.typesupport_identifier) {
    ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ebug_base__srv__ComputeTarget_Response__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ebug_base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ebug_base/srv/detail/compute_target__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_members = {
  "ebug_base__srv",  // service namespace
  "ComputeTarget",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_Request_message_type_support_handle,
  NULL  // response message
  // ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_Response_message_type_support_handle
};

static rosidl_service_type_support_t ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_type_support_handle = {
  0,
  &ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ebug_base
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget)() {
  if (!ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_type_support_handle.typesupport_identifier) {
    ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ebug_base, srv, ComputeTarget_Response)()->data;
  }

  return &ebug_base__srv__detail__compute_target__rosidl_typesupport_introspection_c__ComputeTarget_service_type_support_handle;
}

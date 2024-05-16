// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice
#include "ebug_base/srv/detail/compute_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"

bool
ebug_base__srv__ComputeTarget_Request__init(ebug_base__srv__ComputeTarget_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    ebug_base__srv__ComputeTarget_Request__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->pose)) {
    ebug_base__srv__ComputeTarget_Request__fini(msg);
    return false;
  }
  return true;
}

void
ebug_base__srv__ComputeTarget_Request__fini(ebug_base__srv__ComputeTarget_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // pose
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->pose);
}

bool
ebug_base__srv__ComputeTarget_Request__are_equal(const ebug_base__srv__ComputeTarget_Request * lhs, const ebug_base__srv__ComputeTarget_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
ebug_base__srv__ComputeTarget_Request__copy(
  const ebug_base__srv__ComputeTarget_Request * input,
  ebug_base__srv__ComputeTarget_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

ebug_base__srv__ComputeTarget_Request *
ebug_base__srv__ComputeTarget_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Request * msg = (ebug_base__srv__ComputeTarget_Request *)allocator.allocate(sizeof(ebug_base__srv__ComputeTarget_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ebug_base__srv__ComputeTarget_Request));
  bool success = ebug_base__srv__ComputeTarget_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ebug_base__srv__ComputeTarget_Request__destroy(ebug_base__srv__ComputeTarget_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ebug_base__srv__ComputeTarget_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ebug_base__srv__ComputeTarget_Request__Sequence__init(ebug_base__srv__ComputeTarget_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Request * data = NULL;

  if (size) {
    data = (ebug_base__srv__ComputeTarget_Request *)allocator.zero_allocate(size, sizeof(ebug_base__srv__ComputeTarget_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ebug_base__srv__ComputeTarget_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ebug_base__srv__ComputeTarget_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ebug_base__srv__ComputeTarget_Request__Sequence__fini(ebug_base__srv__ComputeTarget_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ebug_base__srv__ComputeTarget_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ebug_base__srv__ComputeTarget_Request__Sequence *
ebug_base__srv__ComputeTarget_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Request__Sequence * array = (ebug_base__srv__ComputeTarget_Request__Sequence *)allocator.allocate(sizeof(ebug_base__srv__ComputeTarget_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ebug_base__srv__ComputeTarget_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ebug_base__srv__ComputeTarget_Request__Sequence__destroy(ebug_base__srv__ComputeTarget_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ebug_base__srv__ComputeTarget_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ebug_base__srv__ComputeTarget_Request__Sequence__are_equal(const ebug_base__srv__ComputeTarget_Request__Sequence * lhs, const ebug_base__srv__ComputeTarget_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ebug_base__srv__ComputeTarget_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ebug_base__srv__ComputeTarget_Request__Sequence__copy(
  const ebug_base__srv__ComputeTarget_Request__Sequence * input,
  ebug_base__srv__ComputeTarget_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ebug_base__srv__ComputeTarget_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ebug_base__srv__ComputeTarget_Request * data =
      (ebug_base__srv__ComputeTarget_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ebug_base__srv__ComputeTarget_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ebug_base__srv__ComputeTarget_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ebug_base__srv__ComputeTarget_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `control`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `color`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
ebug_base__srv__ComputeTarget_Response__init(ebug_base__srv__ComputeTarget_Response * msg)
{
  if (!msg) {
    return false;
  }
  // control
  if (!geometry_msgs__msg__Twist__init(&msg->control)) {
    ebug_base__srv__ComputeTarget_Response__fini(msg);
    return false;
  }
  // color
  if (!geometry_msgs__msg__Vector3__init(&msg->color)) {
    ebug_base__srv__ComputeTarget_Response__fini(msg);
    return false;
  }
  return true;
}

void
ebug_base__srv__ComputeTarget_Response__fini(ebug_base__srv__ComputeTarget_Response * msg)
{
  if (!msg) {
    return;
  }
  // control
  geometry_msgs__msg__Twist__fini(&msg->control);
  // color
  geometry_msgs__msg__Vector3__fini(&msg->color);
}

bool
ebug_base__srv__ComputeTarget_Response__are_equal(const ebug_base__srv__ComputeTarget_Response * lhs, const ebug_base__srv__ComputeTarget_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // control
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->control), &(rhs->control)))
  {
    return false;
  }
  // color
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
ebug_base__srv__ComputeTarget_Response__copy(
  const ebug_base__srv__ComputeTarget_Response * input,
  ebug_base__srv__ComputeTarget_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // control
  if (!geometry_msgs__msg__Twist__copy(
      &(input->control), &(output->control)))
  {
    return false;
  }
  // color
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

ebug_base__srv__ComputeTarget_Response *
ebug_base__srv__ComputeTarget_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Response * msg = (ebug_base__srv__ComputeTarget_Response *)allocator.allocate(sizeof(ebug_base__srv__ComputeTarget_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ebug_base__srv__ComputeTarget_Response));
  bool success = ebug_base__srv__ComputeTarget_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ebug_base__srv__ComputeTarget_Response__destroy(ebug_base__srv__ComputeTarget_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ebug_base__srv__ComputeTarget_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ebug_base__srv__ComputeTarget_Response__Sequence__init(ebug_base__srv__ComputeTarget_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Response * data = NULL;

  if (size) {
    data = (ebug_base__srv__ComputeTarget_Response *)allocator.zero_allocate(size, sizeof(ebug_base__srv__ComputeTarget_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ebug_base__srv__ComputeTarget_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ebug_base__srv__ComputeTarget_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ebug_base__srv__ComputeTarget_Response__Sequence__fini(ebug_base__srv__ComputeTarget_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ebug_base__srv__ComputeTarget_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ebug_base__srv__ComputeTarget_Response__Sequence *
ebug_base__srv__ComputeTarget_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__srv__ComputeTarget_Response__Sequence * array = (ebug_base__srv__ComputeTarget_Response__Sequence *)allocator.allocate(sizeof(ebug_base__srv__ComputeTarget_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ebug_base__srv__ComputeTarget_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ebug_base__srv__ComputeTarget_Response__Sequence__destroy(ebug_base__srv__ComputeTarget_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ebug_base__srv__ComputeTarget_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ebug_base__srv__ComputeTarget_Response__Sequence__are_equal(const ebug_base__srv__ComputeTarget_Response__Sequence * lhs, const ebug_base__srv__ComputeTarget_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ebug_base__srv__ComputeTarget_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ebug_base__srv__ComputeTarget_Response__Sequence__copy(
  const ebug_base__srv__ComputeTarget_Response__Sequence * input,
  ebug_base__srv__ComputeTarget_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ebug_base__srv__ComputeTarget_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ebug_base__srv__ComputeTarget_Response * data =
      (ebug_base__srv__ComputeTarget_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ebug_base__srv__ComputeTarget_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ebug_base__srv__ComputeTarget_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ebug_base__srv__ComputeTarget_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ebug_base:msg/ControlCommand.idl
// generated code does not contain a copyright notice
#include "ebug_base/msg/detail/control_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `control`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `color`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
ebug_base__msg__ControlCommand__init(ebug_base__msg__ControlCommand * msg)
{
  if (!msg) {
    return false;
  }
  // control
  if (!geometry_msgs__msg__Twist__init(&msg->control)) {
    ebug_base__msg__ControlCommand__fini(msg);
    return false;
  }
  // color
  if (!geometry_msgs__msg__Vector3__init(&msg->color)) {
    ebug_base__msg__ControlCommand__fini(msg);
    return false;
  }
  return true;
}

void
ebug_base__msg__ControlCommand__fini(ebug_base__msg__ControlCommand * msg)
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
ebug_base__msg__ControlCommand__are_equal(const ebug_base__msg__ControlCommand * lhs, const ebug_base__msg__ControlCommand * rhs)
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
ebug_base__msg__ControlCommand__copy(
  const ebug_base__msg__ControlCommand * input,
  ebug_base__msg__ControlCommand * output)
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

ebug_base__msg__ControlCommand *
ebug_base__msg__ControlCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__msg__ControlCommand * msg = (ebug_base__msg__ControlCommand *)allocator.allocate(sizeof(ebug_base__msg__ControlCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ebug_base__msg__ControlCommand));
  bool success = ebug_base__msg__ControlCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ebug_base__msg__ControlCommand__destroy(ebug_base__msg__ControlCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ebug_base__msg__ControlCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ebug_base__msg__ControlCommand__Sequence__init(ebug_base__msg__ControlCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__msg__ControlCommand * data = NULL;

  if (size) {
    data = (ebug_base__msg__ControlCommand *)allocator.zero_allocate(size, sizeof(ebug_base__msg__ControlCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ebug_base__msg__ControlCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ebug_base__msg__ControlCommand__fini(&data[i - 1]);
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
ebug_base__msg__ControlCommand__Sequence__fini(ebug_base__msg__ControlCommand__Sequence * array)
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
      ebug_base__msg__ControlCommand__fini(&array->data[i]);
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

ebug_base__msg__ControlCommand__Sequence *
ebug_base__msg__ControlCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ebug_base__msg__ControlCommand__Sequence * array = (ebug_base__msg__ControlCommand__Sequence *)allocator.allocate(sizeof(ebug_base__msg__ControlCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ebug_base__msg__ControlCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ebug_base__msg__ControlCommand__Sequence__destroy(ebug_base__msg__ControlCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ebug_base__msg__ControlCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ebug_base__msg__ControlCommand__Sequence__are_equal(const ebug_base__msg__ControlCommand__Sequence * lhs, const ebug_base__msg__ControlCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ebug_base__msg__ControlCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ebug_base__msg__ControlCommand__Sequence__copy(
  const ebug_base__msg__ControlCommand__Sequence * input,
  ebug_base__msg__ControlCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ebug_base__msg__ControlCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ebug_base__msg__ControlCommand * data =
      (ebug_base__msg__ControlCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ebug_base__msg__ControlCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ebug_base__msg__ControlCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ebug_base__msg__ControlCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

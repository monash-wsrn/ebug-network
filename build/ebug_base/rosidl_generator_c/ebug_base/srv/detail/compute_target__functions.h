// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ebug_base:srv/ComputeTarget.idl
// generated code does not contain a copyright notice

#ifndef EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__FUNCTIONS_H_
#define EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ebug_base/msg/rosidl_generator_c__visibility_control.h"

#include "ebug_base/srv/detail/compute_target__struct.h"

/// Initialize srv/ComputeTarget message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ebug_base__srv__ComputeTarget_Request
 * )) before or use
 * ebug_base__srv__ComputeTarget_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__init(ebug_base__srv__ComputeTarget_Request * msg);

/// Finalize srv/ComputeTarget message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Request__fini(ebug_base__srv__ComputeTarget_Request * msg);

/// Create srv/ComputeTarget message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ebug_base__srv__ComputeTarget_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
ebug_base__srv__ComputeTarget_Request *
ebug_base__srv__ComputeTarget_Request__create();

/// Destroy srv/ComputeTarget message.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Request__destroy(ebug_base__srv__ComputeTarget_Request * msg);

/// Check for srv/ComputeTarget message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__are_equal(const ebug_base__srv__ComputeTarget_Request * lhs, const ebug_base__srv__ComputeTarget_Request * rhs);

/// Copy a srv/ComputeTarget message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__copy(
  const ebug_base__srv__ComputeTarget_Request * input,
  ebug_base__srv__ComputeTarget_Request * output);

/// Initialize array of srv/ComputeTarget messages.
/**
 * It allocates the memory for the number of elements and calls
 * ebug_base__srv__ComputeTarget_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__Sequence__init(ebug_base__srv__ComputeTarget_Request__Sequence * array, size_t size);

/// Finalize array of srv/ComputeTarget messages.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Request__Sequence__fini(ebug_base__srv__ComputeTarget_Request__Sequence * array);

/// Create array of srv/ComputeTarget messages.
/**
 * It allocates the memory for the array and calls
 * ebug_base__srv__ComputeTarget_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
ebug_base__srv__ComputeTarget_Request__Sequence *
ebug_base__srv__ComputeTarget_Request__Sequence__create(size_t size);

/// Destroy array of srv/ComputeTarget messages.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Request__Sequence__destroy(ebug_base__srv__ComputeTarget_Request__Sequence * array);

/// Check for srv/ComputeTarget message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__Sequence__are_equal(const ebug_base__srv__ComputeTarget_Request__Sequence * lhs, const ebug_base__srv__ComputeTarget_Request__Sequence * rhs);

/// Copy an array of srv/ComputeTarget messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Request__Sequence__copy(
  const ebug_base__srv__ComputeTarget_Request__Sequence * input,
  ebug_base__srv__ComputeTarget_Request__Sequence * output);

/// Initialize srv/ComputeTarget message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ebug_base__srv__ComputeTarget_Response
 * )) before or use
 * ebug_base__srv__ComputeTarget_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__init(ebug_base__srv__ComputeTarget_Response * msg);

/// Finalize srv/ComputeTarget message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Response__fini(ebug_base__srv__ComputeTarget_Response * msg);

/// Create srv/ComputeTarget message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ebug_base__srv__ComputeTarget_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
ebug_base__srv__ComputeTarget_Response *
ebug_base__srv__ComputeTarget_Response__create();

/// Destroy srv/ComputeTarget message.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Response__destroy(ebug_base__srv__ComputeTarget_Response * msg);

/// Check for srv/ComputeTarget message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__are_equal(const ebug_base__srv__ComputeTarget_Response * lhs, const ebug_base__srv__ComputeTarget_Response * rhs);

/// Copy a srv/ComputeTarget message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__copy(
  const ebug_base__srv__ComputeTarget_Response * input,
  ebug_base__srv__ComputeTarget_Response * output);

/// Initialize array of srv/ComputeTarget messages.
/**
 * It allocates the memory for the number of elements and calls
 * ebug_base__srv__ComputeTarget_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__Sequence__init(ebug_base__srv__ComputeTarget_Response__Sequence * array, size_t size);

/// Finalize array of srv/ComputeTarget messages.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Response__Sequence__fini(ebug_base__srv__ComputeTarget_Response__Sequence * array);

/// Create array of srv/ComputeTarget messages.
/**
 * It allocates the memory for the array and calls
 * ebug_base__srv__ComputeTarget_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
ebug_base__srv__ComputeTarget_Response__Sequence *
ebug_base__srv__ComputeTarget_Response__Sequence__create(size_t size);

/// Destroy array of srv/ComputeTarget messages.
/**
 * It calls
 * ebug_base__srv__ComputeTarget_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
void
ebug_base__srv__ComputeTarget_Response__Sequence__destroy(ebug_base__srv__ComputeTarget_Response__Sequence * array);

/// Check for srv/ComputeTarget message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__Sequence__are_equal(const ebug_base__srv__ComputeTarget_Response__Sequence * lhs, const ebug_base__srv__ComputeTarget_Response__Sequence * rhs);

/// Copy an array of srv/ComputeTarget messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ebug_base
bool
ebug_base__srv__ComputeTarget_Response__Sequence__copy(
  const ebug_base__srv__ComputeTarget_Response__Sequence * input,
  ebug_base__srv__ComputeTarget_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EBUG_BASE__SRV__DETAIL__COMPUTE_TARGET__FUNCTIONS_H_

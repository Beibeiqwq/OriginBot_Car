// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ai_msgs:msg/DBResult.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__DB_RESULT__FUNCTIONS_H_
#define AI_MSGS__MSG__DETAIL__DB_RESULT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ai_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ai_msgs/msg/detail/db_result__struct.h"

/// Initialize msg/DBResult message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ai_msgs__msg__DBResult
 * )) before or use
 * ai_msgs__msg__DBResult__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__init(ai_msgs__msg__DBResult * msg);

/// Finalize msg/DBResult message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__DBResult__fini(ai_msgs__msg__DBResult * msg);

/// Create msg/DBResult message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ai_msgs__msg__DBResult__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
ai_msgs__msg__DBResult *
ai_msgs__msg__DBResult__create();

/// Destroy msg/DBResult message.
/**
 * It calls
 * ai_msgs__msg__DBResult__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__DBResult__destroy(ai_msgs__msg__DBResult * msg);

/// Check for msg/DBResult message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__are_equal(const ai_msgs__msg__DBResult * lhs, const ai_msgs__msg__DBResult * rhs);

/// Copy a msg/DBResult message.
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
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__copy(
  const ai_msgs__msg__DBResult * input,
  ai_msgs__msg__DBResult * output);

/// Initialize array of msg/DBResult messages.
/**
 * It allocates the memory for the number of elements and calls
 * ai_msgs__msg__DBResult__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__Sequence__init(ai_msgs__msg__DBResult__Sequence * array, size_t size);

/// Finalize array of msg/DBResult messages.
/**
 * It calls
 * ai_msgs__msg__DBResult__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__DBResult__Sequence__fini(ai_msgs__msg__DBResult__Sequence * array);

/// Create array of msg/DBResult messages.
/**
 * It allocates the memory for the array and calls
 * ai_msgs__msg__DBResult__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
ai_msgs__msg__DBResult__Sequence *
ai_msgs__msg__DBResult__Sequence__create(size_t size);

/// Destroy array of msg/DBResult messages.
/**
 * It calls
 * ai_msgs__msg__DBResult__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__DBResult__Sequence__destroy(ai_msgs__msg__DBResult__Sequence * array);

/// Check for msg/DBResult message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__Sequence__are_equal(const ai_msgs__msg__DBResult__Sequence * lhs, const ai_msgs__msg__DBResult__Sequence * rhs);

/// Copy an array of msg/DBResult messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__DBResult__Sequence__copy(
  const ai_msgs__msg__DBResult__Sequence * input,
  ai_msgs__msg__DBResult__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__DB_RESULT__FUNCTIONS_H_

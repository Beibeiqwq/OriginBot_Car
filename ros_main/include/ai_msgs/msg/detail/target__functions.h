// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__TARGET__FUNCTIONS_H_
#define AI_MSGS__MSG__DETAIL__TARGET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ai_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ai_msgs/msg/detail/target__struct.h"

/// Initialize msg/Target message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ai_msgs__msg__Target
 * )) before or use
 * ai_msgs__msg__Target__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__Target__init(ai_msgs__msg__Target * msg);

/// Finalize msg/Target message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__Target__fini(ai_msgs__msg__Target * msg);

/// Create msg/Target message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ai_msgs__msg__Target__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
ai_msgs__msg__Target *
ai_msgs__msg__Target__create();

/// Destroy msg/Target message.
/**
 * It calls
 * ai_msgs__msg__Target__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__Target__destroy(ai_msgs__msg__Target * msg);

/// Check for msg/Target message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__Target__are_equal(const ai_msgs__msg__Target * lhs, const ai_msgs__msg__Target * rhs);

/// Copy a msg/Target message.
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
ai_msgs__msg__Target__copy(
  const ai_msgs__msg__Target * input,
  ai_msgs__msg__Target * output);

/// Initialize array of msg/Target messages.
/**
 * It allocates the memory for the number of elements and calls
 * ai_msgs__msg__Target__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__Target__Sequence__init(ai_msgs__msg__Target__Sequence * array, size_t size);

/// Finalize array of msg/Target messages.
/**
 * It calls
 * ai_msgs__msg__Target__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__Target__Sequence__fini(ai_msgs__msg__Target__Sequence * array);

/// Create array of msg/Target messages.
/**
 * It allocates the memory for the array and calls
 * ai_msgs__msg__Target__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
ai_msgs__msg__Target__Sequence *
ai_msgs__msg__Target__Sequence__create(size_t size);

/// Destroy array of msg/Target messages.
/**
 * It calls
 * ai_msgs__msg__Target__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
void
ai_msgs__msg__Target__Sequence__destroy(ai_msgs__msg__Target__Sequence * array);

/// Check for msg/Target message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ai_msgs
bool
ai_msgs__msg__Target__Sequence__are_equal(const ai_msgs__msg__Target__Sequence * lhs, const ai_msgs__msg__Target__Sequence * rhs);

/// Copy an array of msg/Target messages.
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
ai_msgs__msg__Target__Sequence__copy(
  const ai_msgs__msg__Target__Sequence * input,
  ai_msgs__msg__Target__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__TARGET__FUNCTIONS_H_

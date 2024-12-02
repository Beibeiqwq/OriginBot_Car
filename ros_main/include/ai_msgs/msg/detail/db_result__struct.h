// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/DBResult.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'db_type'
// Member 'match_id'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/DBResult in the package ai_msgs.
typedef struct ai_msgs__msg__DBResult
{
  rosidl_runtime_c__String db_type;
  rosidl_runtime_c__String match_id;
  float distance;
  float similarity;
} ai_msgs__msg__DBResult;

// Struct for a sequence of ai_msgs__msg__DBResult.
typedef struct ai_msgs__msg__DBResult__Sequence
{
  ai_msgs__msg__DBResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__DBResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_H_

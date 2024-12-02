// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/Perf.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERF__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__PERF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/string.h"
// Member 'stamp_start'
// Member 'stamp_end'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in msg/Perf in the package ai_msgs.
typedef struct ai_msgs__msg__Perf
{
  rosidl_runtime_c__String type;
  builtin_interfaces__msg__Time stamp_start;
  builtin_interfaces__msg__Time stamp_end;
  double time_ms_duration;
} ai_msgs__msg__Perf;

// Struct for a sequence of ai_msgs__msg__Perf.
typedef struct ai_msgs__msg__Perf__Sequence
{
  ai_msgs__msg__Perf * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__Perf__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__PERF__STRUCT_H_

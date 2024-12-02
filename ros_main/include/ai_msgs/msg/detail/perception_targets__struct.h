// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/PerceptionTargets.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'perfs'
#include "ai_msgs/msg/detail/perf__struct.h"
// Member 'targets'
// Member 'disappeared_targets'
#include "ai_msgs/msg/detail/target__struct.h"

// Struct defined in msg/PerceptionTargets in the package ai_msgs.
typedef struct ai_msgs__msg__PerceptionTargets
{
  std_msgs__msg__Header header;
  int16_t fps;
  ai_msgs__msg__Perf__Sequence perfs;
  ai_msgs__msg__Target__Sequence targets;
  ai_msgs__msg__Target__Sequence disappeared_targets;
} ai_msgs__msg__PerceptionTargets;

// Struct for a sequence of ai_msgs__msg__PerceptionTargets.
typedef struct ai_msgs__msg__PerceptionTargets__Sequence
{
  ai_msgs__msg__PerceptionTargets * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__PerceptionTargets__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_H_
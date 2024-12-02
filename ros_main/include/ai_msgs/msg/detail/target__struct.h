// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__TARGET__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__TARGET__STRUCT_H_

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
// Member 'rois'
#include "ai_msgs/msg/detail/roi__struct.h"
// Member 'attributes'
#include "ai_msgs/msg/detail/attribute__struct.h"
// Member 'points'
#include "ai_msgs/msg/detail/point__struct.h"
// Member 'captures'
#include "ai_msgs/msg/detail/capture__struct.h"

// Struct defined in msg/Target in the package ai_msgs.
typedef struct ai_msgs__msg__Target
{
  rosidl_runtime_c__String type;
  uint64_t track_id;
  ai_msgs__msg__Roi__Sequence rois;
  ai_msgs__msg__Attribute__Sequence attributes;
  ai_msgs__msg__Point__Sequence points;
  ai_msgs__msg__Capture__Sequence captures;
} ai_msgs__msg__Target;

// Struct for a sequence of ai_msgs__msg__Target.
typedef struct ai_msgs__msg__Target__Sequence
{
  ai_msgs__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__TARGET__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__POINT__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__POINT__STRUCT_H_

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
// Member 'point'
#include "geometry_msgs/msg/detail/point32__struct.h"
// Member 'confidence'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Point in the package ai_msgs.
typedef struct ai_msgs__msg__Point
{
  rosidl_runtime_c__String type;
  geometry_msgs__msg__Point32__Sequence point;
  rosidl_runtime_c__float__Sequence confidence;
} ai_msgs__msg__Point;

// Struct for a sequence of ai_msgs__msg__Point.
typedef struct ai_msgs__msg__Point__Sequence
{
  ai_msgs__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__POINT__STRUCT_H_

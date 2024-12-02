// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/Capture.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_H_

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
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'features'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'db_result'
#include "ai_msgs/msg/detail/db_result__struct.h"

// Struct defined in msg/Capture in the package ai_msgs.
typedef struct ai_msgs__msg__Capture
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__Image img;
  rosidl_runtime_c__float__Sequence features;
  ai_msgs__msg__DBResult db_result;
} ai_msgs__msg__Capture;

// Struct for a sequence of ai_msgs__msg__Capture.
typedef struct ai_msgs__msg__Capture__Sequence
{
  ai_msgs__msg__Capture * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__Capture__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_H_

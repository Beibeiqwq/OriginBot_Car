// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ai_msgs:msg/Roi.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__ROI__STRUCT_H_
#define AI_MSGS__MSG__DETAIL__ROI__STRUCT_H_

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
// Member 'rect'
#include "sensor_msgs/msg/detail/region_of_interest__struct.h"

// Struct defined in msg/Roi in the package ai_msgs.
typedef struct ai_msgs__msg__Roi
{
  rosidl_runtime_c__String type;
  sensor_msgs__msg__RegionOfInterest rect;
  float confidence;
} ai_msgs__msg__Roi;

// Struct for a sequence of ai_msgs__msg__Roi.
typedef struct ai_msgs__msg__Roi__Sequence
{
  ai_msgs__msg__Roi * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ai_msgs__msg__Roi__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AI_MSGS__MSG__DETAIL__ROI__STRUCT_H_

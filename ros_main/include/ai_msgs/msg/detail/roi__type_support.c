// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ai_msgs:msg/Roi.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ai_msgs/msg/detail/roi__rosidl_typesupport_introspection_c.h"
#include "ai_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ai_msgs/msg/detail/roi__functions.h"
#include "ai_msgs/msg/detail/roi__struct.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"
// Member `rect`
#include "sensor_msgs/msg/region_of_interest.h"
// Member `rect`
#include "sensor_msgs/msg/detail/region_of_interest__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Roi__rosidl_typesupport_introspection_c__Roi_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ai_msgs__msg__Roi__init(message_memory);
}

void Roi__rosidl_typesupport_introspection_c__Roi_fini_function(void * message_memory)
{
  ai_msgs__msg__Roi__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Roi__rosidl_typesupport_introspection_c__Roi_message_member_array[3] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__Roi, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rect",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__Roi, rect),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__Roi, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Roi__rosidl_typesupport_introspection_c__Roi_message_members = {
  "ai_msgs__msg",  // message namespace
  "Roi",  // message name
  3,  // number of fields
  sizeof(ai_msgs__msg__Roi),
  Roi__rosidl_typesupport_introspection_c__Roi_message_member_array,  // message members
  Roi__rosidl_typesupport_introspection_c__Roi_init_function,  // function to initialize message memory (memory has to be allocated)
  Roi__rosidl_typesupport_introspection_c__Roi_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Roi__rosidl_typesupport_introspection_c__Roi_message_type_support_handle = {
  0,
  &Roi__rosidl_typesupport_introspection_c__Roi_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ai_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, Roi)() {
  Roi__rosidl_typesupport_introspection_c__Roi_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, RegionOfInterest)();
  if (!Roi__rosidl_typesupport_introspection_c__Roi_message_type_support_handle.typesupport_identifier) {
    Roi__rosidl_typesupport_introspection_c__Roi_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Roi__rosidl_typesupport_introspection_c__Roi_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

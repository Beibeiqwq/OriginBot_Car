// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ai_msgs:msg/Attribute.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ai_msgs/msg/detail/attribute__rosidl_typesupport_introspection_c.h"
#include "ai_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ai_msgs/msg/detail/attribute__functions.h"
#include "ai_msgs/msg/detail/attribute__struct.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Attribute__rosidl_typesupport_introspection_c__Attribute_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ai_msgs__msg__Attribute__init(message_memory);
}

void Attribute__rosidl_typesupport_introspection_c__Attribute_fini_function(void * message_memory)
{
  ai_msgs__msg__Attribute__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Attribute__rosidl_typesupport_introspection_c__Attribute_message_member_array[3] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__Attribute, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__Attribute, value),  // bytes offset in struct
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
    offsetof(ai_msgs__msg__Attribute, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Attribute__rosidl_typesupport_introspection_c__Attribute_message_members = {
  "ai_msgs__msg",  // message namespace
  "Attribute",  // message name
  3,  // number of fields
  sizeof(ai_msgs__msg__Attribute),
  Attribute__rosidl_typesupport_introspection_c__Attribute_message_member_array,  // message members
  Attribute__rosidl_typesupport_introspection_c__Attribute_init_function,  // function to initialize message memory (memory has to be allocated)
  Attribute__rosidl_typesupport_introspection_c__Attribute_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Attribute__rosidl_typesupport_introspection_c__Attribute_message_type_support_handle = {
  0,
  &Attribute__rosidl_typesupport_introspection_c__Attribute_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ai_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, Attribute)() {
  if (!Attribute__rosidl_typesupport_introspection_c__Attribute_message_type_support_handle.typesupport_identifier) {
    Attribute__rosidl_typesupport_introspection_c__Attribute_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Attribute__rosidl_typesupport_introspection_c__Attribute_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

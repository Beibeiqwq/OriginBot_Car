// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ai_msgs:msg/PerceptionTargets.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ai_msgs/msg/detail/perception_targets__rosidl_typesupport_introspection_c.h"
#include "ai_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ai_msgs/msg/detail/perception_targets__functions.h"
#include "ai_msgs/msg/detail/perception_targets__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `perfs`
#include "ai_msgs/msg/perf.h"
// Member `perfs`
#include "ai_msgs/msg/detail/perf__rosidl_typesupport_introspection_c.h"
// Member `targets`
// Member `disappeared_targets`
#include "ai_msgs/msg/target.h"
// Member `targets`
// Member `disappeared_targets`
#include "ai_msgs/msg/detail/target__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ai_msgs__msg__PerceptionTargets__init(message_memory);
}

void PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_fini_function(void * message_memory)
{
  ai_msgs__msg__PerceptionTargets__fini(message_memory);
}

size_t PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Perf__perfs(
  const void * untyped_member)
{
  const ai_msgs__msg__Perf__Sequence * member =
    (const ai_msgs__msg__Perf__Sequence *)(untyped_member);
  return member->size;
}

const void * PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Perf__perfs(
  const void * untyped_member, size_t index)
{
  const ai_msgs__msg__Perf__Sequence * member =
    (const ai_msgs__msg__Perf__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Perf__perfs(
  void * untyped_member, size_t index)
{
  ai_msgs__msg__Perf__Sequence * member =
    (ai_msgs__msg__Perf__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Perf__perfs(
  void * untyped_member, size_t size)
{
  ai_msgs__msg__Perf__Sequence * member =
    (ai_msgs__msg__Perf__Sequence *)(untyped_member);
  ai_msgs__msg__Perf__Sequence__fini(member);
  return ai_msgs__msg__Perf__Sequence__init(member, size);
}

size_t PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Target__targets(
  const void * untyped_member)
{
  const ai_msgs__msg__Target__Sequence * member =
    (const ai_msgs__msg__Target__Sequence *)(untyped_member);
  return member->size;
}

const void * PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Target__targets(
  const void * untyped_member, size_t index)
{
  const ai_msgs__msg__Target__Sequence * member =
    (const ai_msgs__msg__Target__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Target__targets(
  void * untyped_member, size_t index)
{
  ai_msgs__msg__Target__Sequence * member =
    (ai_msgs__msg__Target__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Target__targets(
  void * untyped_member, size_t size)
{
  ai_msgs__msg__Target__Sequence * member =
    (ai_msgs__msg__Target__Sequence *)(untyped_member);
  ai_msgs__msg__Target__Sequence__fini(member);
  return ai_msgs__msg__Target__Sequence__init(member, size);
}

size_t PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Target__disappeared_targets(
  const void * untyped_member)
{
  const ai_msgs__msg__Target__Sequence * member =
    (const ai_msgs__msg__Target__Sequence *)(untyped_member);
  return member->size;
}

const void * PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Target__disappeared_targets(
  const void * untyped_member, size_t index)
{
  const ai_msgs__msg__Target__Sequence * member =
    (const ai_msgs__msg__Target__Sequence *)(untyped_member);
  return &member->data[index];
}

void * PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Target__disappeared_targets(
  void * untyped_member, size_t index)
{
  ai_msgs__msg__Target__Sequence * member =
    (ai_msgs__msg__Target__Sequence *)(untyped_member);
  return &member->data[index];
}

bool PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Target__disappeared_targets(
  void * untyped_member, size_t size)
{
  ai_msgs__msg__Target__Sequence * member =
    (ai_msgs__msg__Target__Sequence *)(untyped_member);
  ai_msgs__msg__Target__Sequence__fini(member);
  return ai_msgs__msg__Target__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__PerceptionTargets, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__PerceptionTargets, fps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "perfs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__PerceptionTargets, perfs),  // bytes offset in struct
    NULL,  // default value
    PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Perf__perfs,  // size() function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Perf__perfs,  // get_const(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Perf__perfs,  // get(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Perf__perfs  // resize(index) function pointer
  },
  {
    "targets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__PerceptionTargets, targets),  // bytes offset in struct
    NULL,  // default value
    PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Target__targets,  // size() function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Target__targets,  // get_const(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Target__targets,  // get(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Target__targets  // resize(index) function pointer
  },
  {
    "disappeared_targets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs__msg__PerceptionTargets, disappeared_targets),  // bytes offset in struct
    NULL,  // default value
    PerceptionTargets__rosidl_typesupport_introspection_c__size_function__Target__disappeared_targets,  // size() function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_const_function__Target__disappeared_targets,  // get_const(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__get_function__Target__disappeared_targets,  // get(index) function pointer
    PerceptionTargets__rosidl_typesupport_introspection_c__resize_function__Target__disappeared_targets  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_members = {
  "ai_msgs__msg",  // message namespace
  "PerceptionTargets",  // message name
  5,  // number of fields
  sizeof(ai_msgs__msg__PerceptionTargets),
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array,  // message members
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_init_function,  // function to initialize message memory (memory has to be allocated)
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_type_support_handle = {
  0,
  &PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ai_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, PerceptionTargets)() {
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, Perf)();
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, Target)();
  PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ai_msgs, msg, Target)();
  if (!PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_type_support_handle.typesupport_identifier) {
    PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PerceptionTargets__rosidl_typesupport_introspection_c__PerceptionTargets_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

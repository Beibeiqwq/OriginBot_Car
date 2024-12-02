// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ai_msgs:msg/CaptureTargets.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ai_msgs/msg/detail/capture_targets__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ai_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CaptureTargets_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ai_msgs::msg::CaptureTargets(_init);
}

void CaptureTargets_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ai_msgs::msg::CaptureTargets *>(message_memory);
  typed_message->~CaptureTargets();
}

size_t size_function__CaptureTargets__perfs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Perf> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CaptureTargets__perfs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Perf> *>(untyped_member);
  return &member[index];
}

void * get_function__CaptureTargets__perfs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Perf> *>(untyped_member);
  return &member[index];
}

void resize_function__CaptureTargets__perfs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Perf> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CaptureTargets__targets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Target> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CaptureTargets__targets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Target> *>(untyped_member);
  return &member[index];
}

void * get_function__CaptureTargets__targets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Target> *>(untyped_member);
  return &member[index];
}

void resize_function__CaptureTargets__targets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Target> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CaptureTargets_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::CaptureTargets, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "perfs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Perf>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::CaptureTargets, perfs),  // bytes offset in struct
    nullptr,  // default value
    size_function__CaptureTargets__perfs,  // size() function pointer
    get_const_function__CaptureTargets__perfs,  // get_const(index) function pointer
    get_function__CaptureTargets__perfs,  // get(index) function pointer
    resize_function__CaptureTargets__perfs  // resize(index) function pointer
  },
  {
    "targets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Target>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::CaptureTargets, targets),  // bytes offset in struct
    nullptr,  // default value
    size_function__CaptureTargets__targets,  // size() function pointer
    get_const_function__CaptureTargets__targets,  // get_const(index) function pointer
    get_function__CaptureTargets__targets,  // get(index) function pointer
    resize_function__CaptureTargets__targets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CaptureTargets_message_members = {
  "ai_msgs::msg",  // message namespace
  "CaptureTargets",  // message name
  3,  // number of fields
  sizeof(ai_msgs::msg::CaptureTargets),
  CaptureTargets_message_member_array,  // message members
  CaptureTargets_init_function,  // function to initialize message memory (memory has to be allocated)
  CaptureTargets_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CaptureTargets_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CaptureTargets_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ai_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ai_msgs::msg::CaptureTargets>()
{
  return &::ai_msgs::msg::rosidl_typesupport_introspection_cpp::CaptureTargets_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ai_msgs, msg, CaptureTargets)() {
  return &::ai_msgs::msg::rosidl_typesupport_introspection_cpp::CaptureTargets_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ai_msgs/msg/detail/target__struct.hpp"
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

void Target_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ai_msgs::msg::Target(_init);
}

void Target_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ai_msgs::msg::Target *>(message_memory);
  typed_message->~Target();
}

size_t size_function__Target__rois(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Roi> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Target__rois(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Roi> *>(untyped_member);
  return &member[index];
}

void * get_function__Target__rois(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Roi> *>(untyped_member);
  return &member[index];
}

void resize_function__Target__rois(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Roi> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Target__attributes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Attribute> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Target__attributes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Attribute> *>(untyped_member);
  return &member[index];
}

void * get_function__Target__attributes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Attribute> *>(untyped_member);
  return &member[index];
}

void resize_function__Target__attributes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Attribute> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Target__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Target__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__Target__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void resize_function__Target__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Target__captures(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ai_msgs::msg::Capture> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Target__captures(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ai_msgs::msg::Capture> *>(untyped_member);
  return &member[index];
}

void * get_function__Target__captures(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ai_msgs::msg::Capture> *>(untyped_member);
  return &member[index];
}

void resize_function__Target__captures(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ai_msgs::msg::Capture> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Target_message_member_array[6] = {
  {
    "type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "track_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, track_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rois",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Roi>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, rois),  // bytes offset in struct
    nullptr,  // default value
    size_function__Target__rois,  // size() function pointer
    get_const_function__Target__rois,  // get_const(index) function pointer
    get_function__Target__rois,  // get(index) function pointer
    resize_function__Target__rois  // resize(index) function pointer
  },
  {
    "attributes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Attribute>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, attributes),  // bytes offset in struct
    nullptr,  // default value
    size_function__Target__attributes,  // size() function pointer
    get_const_function__Target__attributes,  // get_const(index) function pointer
    get_function__Target__attributes,  // get(index) function pointer
    resize_function__Target__attributes  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__Target__points,  // size() function pointer
    get_const_function__Target__points,  // get_const(index) function pointer
    get_function__Target__points,  // get(index) function pointer
    resize_function__Target__points  // resize(index) function pointer
  },
  {
    "captures",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ai_msgs::msg::Capture>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ai_msgs::msg::Target, captures),  // bytes offset in struct
    nullptr,  // default value
    size_function__Target__captures,  // size() function pointer
    get_const_function__Target__captures,  // get_const(index) function pointer
    get_function__Target__captures,  // get(index) function pointer
    resize_function__Target__captures  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Target_message_members = {
  "ai_msgs::msg",  // message namespace
  "Target",  // message name
  6,  // number of fields
  sizeof(ai_msgs::msg::Target),
  Target_message_member_array,  // message members
  Target_init_function,  // function to initialize message memory (memory has to be allocated)
  Target_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Target_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Target_message_members,
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
get_message_type_support_handle<ai_msgs::msg::Target>()
{
  return &::ai_msgs::msg::rosidl_typesupport_introspection_cpp::Target_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ai_msgs, msg, Target)() {
  return &::ai_msgs::msg::rosidl_typesupport_introspection_cpp::Target_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

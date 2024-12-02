// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Attribute.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__ATTRIBUTE__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__ATTRIBUTE__BUILDER_HPP_

#include "ai_msgs/msg/detail/attribute__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Attribute_confidence
{
public:
  explicit Init_Attribute_confidence(::ai_msgs::msg::Attribute & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Attribute confidence(::ai_msgs::msg::Attribute::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Attribute msg_;
};

class Init_Attribute_value
{
public:
  explicit Init_Attribute_value(::ai_msgs::msg::Attribute & msg)
  : msg_(msg)
  {}
  Init_Attribute_confidence value(::ai_msgs::msg::Attribute::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_Attribute_confidence(msg_);
  }

private:
  ::ai_msgs::msg::Attribute msg_;
};

class Init_Attribute_type
{
public:
  Init_Attribute_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Attribute_value type(::ai_msgs::msg::Attribute::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Attribute_value(msg_);
  }

private:
  ::ai_msgs::msg::Attribute msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Attribute>()
{
  return ai_msgs::msg::builder::Init_Attribute_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__ATTRIBUTE__BUILDER_HPP_

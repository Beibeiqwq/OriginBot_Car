// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Roi.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__ROI__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__ROI__BUILDER_HPP_

#include "ai_msgs/msg/detail/roi__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Roi_confidence
{
public:
  explicit Init_Roi_confidence(::ai_msgs::msg::Roi & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Roi confidence(::ai_msgs::msg::Roi::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Roi msg_;
};

class Init_Roi_rect
{
public:
  explicit Init_Roi_rect(::ai_msgs::msg::Roi & msg)
  : msg_(msg)
  {}
  Init_Roi_confidence rect(::ai_msgs::msg::Roi::_rect_type arg)
  {
    msg_.rect = std::move(arg);
    return Init_Roi_confidence(msg_);
  }

private:
  ::ai_msgs::msg::Roi msg_;
};

class Init_Roi_type
{
public:
  Init_Roi_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Roi_rect type(::ai_msgs::msg::Roi::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Roi_rect(msg_);
  }

private:
  ::ai_msgs::msg::Roi msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Roi>()
{
  return ai_msgs::msg::builder::Init_Roi_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__ROI__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/CaptureTargets.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__CAPTURE_TARGETS__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__CAPTURE_TARGETS__BUILDER_HPP_

#include "ai_msgs/msg/detail/capture_targets__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_CaptureTargets_targets
{
public:
  explicit Init_CaptureTargets_targets(::ai_msgs::msg::CaptureTargets & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::CaptureTargets targets(::ai_msgs::msg::CaptureTargets::_targets_type arg)
  {
    msg_.targets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::CaptureTargets msg_;
};

class Init_CaptureTargets_perfs
{
public:
  explicit Init_CaptureTargets_perfs(::ai_msgs::msg::CaptureTargets & msg)
  : msg_(msg)
  {}
  Init_CaptureTargets_targets perfs(::ai_msgs::msg::CaptureTargets::_perfs_type arg)
  {
    msg_.perfs = std::move(arg);
    return Init_CaptureTargets_targets(msg_);
  }

private:
  ::ai_msgs::msg::CaptureTargets msg_;
};

class Init_CaptureTargets_header
{
public:
  Init_CaptureTargets_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureTargets_perfs header(::ai_msgs::msg::CaptureTargets::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CaptureTargets_perfs(msg_);
  }

private:
  ::ai_msgs::msg::CaptureTargets msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::CaptureTargets>()
{
  return ai_msgs::msg::builder::Init_CaptureTargets_header();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__CAPTURE_TARGETS__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/PerceptionTargets.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__BUILDER_HPP_

#include "ai_msgs/msg/detail/perception_targets__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_PerceptionTargets_disappeared_targets
{
public:
  explicit Init_PerceptionTargets_disappeared_targets(::ai_msgs::msg::PerceptionTargets & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::PerceptionTargets disappeared_targets(::ai_msgs::msg::PerceptionTargets::_disappeared_targets_type arg)
  {
    msg_.disappeared_targets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::PerceptionTargets msg_;
};

class Init_PerceptionTargets_targets
{
public:
  explicit Init_PerceptionTargets_targets(::ai_msgs::msg::PerceptionTargets & msg)
  : msg_(msg)
  {}
  Init_PerceptionTargets_disappeared_targets targets(::ai_msgs::msg::PerceptionTargets::_targets_type arg)
  {
    msg_.targets = std::move(arg);
    return Init_PerceptionTargets_disappeared_targets(msg_);
  }

private:
  ::ai_msgs::msg::PerceptionTargets msg_;
};

class Init_PerceptionTargets_perfs
{
public:
  explicit Init_PerceptionTargets_perfs(::ai_msgs::msg::PerceptionTargets & msg)
  : msg_(msg)
  {}
  Init_PerceptionTargets_targets perfs(::ai_msgs::msg::PerceptionTargets::_perfs_type arg)
  {
    msg_.perfs = std::move(arg);
    return Init_PerceptionTargets_targets(msg_);
  }

private:
  ::ai_msgs::msg::PerceptionTargets msg_;
};

class Init_PerceptionTargets_fps
{
public:
  explicit Init_PerceptionTargets_fps(::ai_msgs::msg::PerceptionTargets & msg)
  : msg_(msg)
  {}
  Init_PerceptionTargets_perfs fps(::ai_msgs::msg::PerceptionTargets::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return Init_PerceptionTargets_perfs(msg_);
  }

private:
  ::ai_msgs::msg::PerceptionTargets msg_;
};

class Init_PerceptionTargets_header
{
public:
  Init_PerceptionTargets_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerceptionTargets_fps header(::ai_msgs::msg::PerceptionTargets::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PerceptionTargets_fps(msg_);
  }

private:
  ::ai_msgs::msg::PerceptionTargets msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::PerceptionTargets>()
{
  return ai_msgs::msg::builder::Init_PerceptionTargets_header();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__BUILDER_HPP_

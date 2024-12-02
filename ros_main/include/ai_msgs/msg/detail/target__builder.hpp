// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_

#include "ai_msgs/msg/detail/target__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Target_captures
{
public:
  explicit Init_Target_captures(::ai_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Target captures(::ai_msgs::msg::Target::_captures_type arg)
  {
    msg_.captures = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

class Init_Target_points
{
public:
  explicit Init_Target_points(::ai_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_captures points(::ai_msgs::msg::Target::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_Target_captures(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

class Init_Target_attributes
{
public:
  explicit Init_Target_attributes(::ai_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_points attributes(::ai_msgs::msg::Target::_attributes_type arg)
  {
    msg_.attributes = std::move(arg);
    return Init_Target_points(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

class Init_Target_rois
{
public:
  explicit Init_Target_rois(::ai_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_attributes rois(::ai_msgs::msg::Target::_rois_type arg)
  {
    msg_.rois = std::move(arg);
    return Init_Target_attributes(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

class Init_Target_track_id
{
public:
  explicit Init_Target_track_id(::ai_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_rois track_id(::ai_msgs::msg::Target::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_Target_rois(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

class Init_Target_type
{
public:
  Init_Target_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_track_id type(::ai_msgs::msg::Target::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Target_track_id(msg_);
  }

private:
  ::ai_msgs::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Target>()
{
  return ai_msgs::msg::builder::Init_Target_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_

#include "ai_msgs/msg/detail/point__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Point_confidence
{
public:
  explicit Init_Point_confidence(::ai_msgs::msg::Point & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Point confidence(::ai_msgs::msg::Point::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Point msg_;
};

class Init_Point_point
{
public:
  explicit Init_Point_point(::ai_msgs::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_confidence point(::ai_msgs::msg::Point::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_Point_confidence(msg_);
  }

private:
  ::ai_msgs::msg::Point msg_;
};

class Init_Point_type
{
public:
  Init_Point_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_point type(::ai_msgs::msg::Point::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Point_point(msg_);
  }

private:
  ::ai_msgs::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Point>()
{
  return ai_msgs::msg::builder::Init_Point_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_

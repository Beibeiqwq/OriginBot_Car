// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Capture.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__CAPTURE__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__CAPTURE__BUILDER_HPP_

#include "ai_msgs/msg/detail/capture__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Capture_db_result
{
public:
  explicit Init_Capture_db_result(::ai_msgs::msg::Capture & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Capture db_result(::ai_msgs::msg::Capture::_db_result_type arg)
  {
    msg_.db_result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Capture msg_;
};

class Init_Capture_features
{
public:
  explicit Init_Capture_features(::ai_msgs::msg::Capture & msg)
  : msg_(msg)
  {}
  Init_Capture_db_result features(::ai_msgs::msg::Capture::_features_type arg)
  {
    msg_.features = std::move(arg);
    return Init_Capture_db_result(msg_);
  }

private:
  ::ai_msgs::msg::Capture msg_;
};

class Init_Capture_img
{
public:
  explicit Init_Capture_img(::ai_msgs::msg::Capture & msg)
  : msg_(msg)
  {}
  Init_Capture_features img(::ai_msgs::msg::Capture::_img_type arg)
  {
    msg_.img = std::move(arg);
    return Init_Capture_features(msg_);
  }

private:
  ::ai_msgs::msg::Capture msg_;
};

class Init_Capture_header
{
public:
  Init_Capture_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Capture_img header(::ai_msgs::msg::Capture::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Capture_img(msg_);
  }

private:
  ::ai_msgs::msg::Capture msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Capture>()
{
  return ai_msgs::msg::builder::Init_Capture_header();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__CAPTURE__BUILDER_HPP_

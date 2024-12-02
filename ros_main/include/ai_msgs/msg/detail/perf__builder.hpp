// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/Perf.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERF__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__PERF__BUILDER_HPP_

#include "ai_msgs/msg/detail/perf__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_Perf_time_ms_duration
{
public:
  explicit Init_Perf_time_ms_duration(::ai_msgs::msg::Perf & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::Perf time_ms_duration(::ai_msgs::msg::Perf::_time_ms_duration_type arg)
  {
    msg_.time_ms_duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::Perf msg_;
};

class Init_Perf_stamp_end
{
public:
  explicit Init_Perf_stamp_end(::ai_msgs::msg::Perf & msg)
  : msg_(msg)
  {}
  Init_Perf_time_ms_duration stamp_end(::ai_msgs::msg::Perf::_stamp_end_type arg)
  {
    msg_.stamp_end = std::move(arg);
    return Init_Perf_time_ms_duration(msg_);
  }

private:
  ::ai_msgs::msg::Perf msg_;
};

class Init_Perf_stamp_start
{
public:
  explicit Init_Perf_stamp_start(::ai_msgs::msg::Perf & msg)
  : msg_(msg)
  {}
  Init_Perf_stamp_end stamp_start(::ai_msgs::msg::Perf::_stamp_start_type arg)
  {
    msg_.stamp_start = std::move(arg);
    return Init_Perf_stamp_end(msg_);
  }

private:
  ::ai_msgs::msg::Perf msg_;
};

class Init_Perf_type
{
public:
  Init_Perf_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Perf_stamp_start type(::ai_msgs::msg::Perf::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Perf_stamp_start(msg_);
  }

private:
  ::ai_msgs::msg::Perf msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::Perf>()
{
  return ai_msgs::msg::builder::Init_Perf_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__PERF__BUILDER_HPP_

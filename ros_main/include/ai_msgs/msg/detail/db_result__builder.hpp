// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ai_msgs:msg/DBResult.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__DB_RESULT__BUILDER_HPP_
#define AI_MSGS__MSG__DETAIL__DB_RESULT__BUILDER_HPP_

#include "ai_msgs/msg/detail/db_result__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ai_msgs
{

namespace msg
{

namespace builder
{

class Init_DBResult_similarity
{
public:
  explicit Init_DBResult_similarity(::ai_msgs::msg::DBResult & msg)
  : msg_(msg)
  {}
  ::ai_msgs::msg::DBResult similarity(::ai_msgs::msg::DBResult::_similarity_type arg)
  {
    msg_.similarity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ai_msgs::msg::DBResult msg_;
};

class Init_DBResult_distance
{
public:
  explicit Init_DBResult_distance(::ai_msgs::msg::DBResult & msg)
  : msg_(msg)
  {}
  Init_DBResult_similarity distance(::ai_msgs::msg::DBResult::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_DBResult_similarity(msg_);
  }

private:
  ::ai_msgs::msg::DBResult msg_;
};

class Init_DBResult_match_id
{
public:
  explicit Init_DBResult_match_id(::ai_msgs::msg::DBResult & msg)
  : msg_(msg)
  {}
  Init_DBResult_distance match_id(::ai_msgs::msg::DBResult::_match_id_type arg)
  {
    msg_.match_id = std::move(arg);
    return Init_DBResult_distance(msg_);
  }

private:
  ::ai_msgs::msg::DBResult msg_;
};

class Init_DBResult_db_type
{
public:
  Init_DBResult_db_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DBResult_match_id db_type(::ai_msgs::msg::DBResult::_db_type_type arg)
  {
    msg_.db_type = std::move(arg);
    return Init_DBResult_match_id(msg_);
  }

private:
  ::ai_msgs::msg::DBResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ai_msgs::msg::DBResult>()
{
  return ai_msgs::msg::builder::Init_DBResult_db_type();
}

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__DB_RESULT__BUILDER_HPP_

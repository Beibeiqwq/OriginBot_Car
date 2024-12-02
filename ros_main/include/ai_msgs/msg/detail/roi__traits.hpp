// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ai_msgs:msg/Roi.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__ROI__TRAITS_HPP_
#define AI_MSGS__MSG__DETAIL__ROI__TRAITS_HPP_

#include "ai_msgs/msg/detail/roi__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'rect'
#include "sensor_msgs/msg/detail/region_of_interest__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ai_msgs::msg::Roi>()
{
  return "ai_msgs::msg::Roi";
}

template<>
inline const char * name<ai_msgs::msg::Roi>()
{
  return "ai_msgs/msg/Roi";
}

template<>
struct has_fixed_size<ai_msgs::msg::Roi>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ai_msgs::msg::Roi>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ai_msgs::msg::Roi>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AI_MSGS__MSG__DETAIL__ROI__TRAITS_HPP_

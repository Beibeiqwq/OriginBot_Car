// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/Perf.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERF__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__PERF__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'stamp_start'
// Member 'stamp_end'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__Perf __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__Perf __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Perf_
{
  using Type = Perf_<ContainerAllocator>;

  explicit Perf_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp_start(_init),
    stamp_end(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->time_ms_duration = 0.0;
    }
  }

  explicit Perf_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc),
    stamp_start(_alloc, _init),
    stamp_end(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->time_ms_duration = 0.0;
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _type_type type;
  using _stamp_start_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_start_type stamp_start;
  using _stamp_end_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_end_type stamp_end;
  using _time_ms_duration_type =
    double;
  _time_ms_duration_type time_ms_duration;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__stamp_start(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp_start = _arg;
    return *this;
  }
  Type & set__stamp_end(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp_end = _arg;
    return *this;
  }
  Type & set__time_ms_duration(
    const double & _arg)
  {
    this->time_ms_duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::Perf_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::Perf_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::Perf_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::Perf_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Perf_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Perf_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Perf_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Perf_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::Perf_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::Perf_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__Perf
    std::shared_ptr<ai_msgs::msg::Perf_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__Perf
    std::shared_ptr<ai_msgs::msg::Perf_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Perf_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->stamp_start != other.stamp_start) {
      return false;
    }
    if (this->stamp_end != other.stamp_end) {
      return false;
    }
    if (this->time_ms_duration != other.time_ms_duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const Perf_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Perf_

// alias to use template instance with default allocator
using Perf =
  ai_msgs::msg::Perf_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__PERF__STRUCT_HPP_

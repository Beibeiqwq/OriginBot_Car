// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/Capture.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'db_result'
#include "ai_msgs/msg/detail/db_result__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__Capture __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__Capture __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Capture_
{
  using Type = Capture_<ContainerAllocator>;

  explicit Capture_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    img(_init),
    db_result(_init)
  {
    (void)_init;
  }

  explicit Capture_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    img(_alloc, _init),
    db_result(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _img_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _img_type img;
  using _features_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _features_type features;
  using _db_result_type =
    ai_msgs::msg::DBResult_<ContainerAllocator>;
  _db_result_type db_result;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__img(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->img = _arg;
    return *this;
  }
  Type & set__features(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->features = _arg;
    return *this;
  }
  Type & set__db_result(
    const ai_msgs::msg::DBResult_<ContainerAllocator> & _arg)
  {
    this->db_result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::Capture_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::Capture_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::Capture_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::Capture_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Capture_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Capture_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Capture_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Capture_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::Capture_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::Capture_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__Capture
    std::shared_ptr<ai_msgs::msg::Capture_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__Capture
    std::shared_ptr<ai_msgs::msg::Capture_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Capture_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->img != other.img) {
      return false;
    }
    if (this->features != other.features) {
      return false;
    }
    if (this->db_result != other.db_result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Capture_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Capture_

// alias to use template instance with default allocator
using Capture =
  ai_msgs::msg::Capture_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__CAPTURE__STRUCT_HPP_

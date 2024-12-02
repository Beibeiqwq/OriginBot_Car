// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/Roi.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__ROI__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__ROI__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'rect'
#include "sensor_msgs/msg/detail/region_of_interest__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__Roi __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__Roi __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Roi_
{
  using Type = Roi_<ContainerAllocator>;

  explicit Roi_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rect(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->confidence = 0.0f;
    }
  }

  explicit Roi_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc),
    rect(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _type_type type;
  using _rect_type =
    sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>;
  _rect_type rect;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__rect(
    const sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> & _arg)
  {
    this->rect = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::Roi_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::Roi_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::Roi_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::Roi_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Roi_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Roi_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Roi_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Roi_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::Roi_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::Roi_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__Roi
    std::shared_ptr<ai_msgs::msg::Roi_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__Roi
    std::shared_ptr<ai_msgs::msg::Roi_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Roi_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->rect != other.rect) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Roi_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Roi_

// alias to use template instance with default allocator
using Roi =
  ai_msgs::msg::Roi_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__ROI__STRUCT_HPP_

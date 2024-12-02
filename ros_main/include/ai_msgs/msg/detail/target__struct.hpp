// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'rois'
#include "ai_msgs/msg/detail/roi__struct.hpp"
// Member 'attributes'
#include "ai_msgs/msg/detail/attribute__struct.hpp"
// Member 'points'
#include "ai_msgs/msg/detail/point__struct.hpp"
// Member 'captures'
#include "ai_msgs/msg/detail/capture__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__Target __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__Target __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Target_
{
  using Type = Target_<ContainerAllocator>;

  explicit Target_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->track_id = 0ull;
    }
  }

  explicit Target_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = "";
      this->track_id = 0ull;
    }
  }

  // field types and members
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _type_type type;
  using _track_id_type =
    uint64_t;
  _track_id_type track_id;
  using _rois_type =
    std::vector<ai_msgs::msg::Roi_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Roi_<ContainerAllocator>>::other>;
  _rois_type rois;
  using _attributes_type =
    std::vector<ai_msgs::msg::Attribute_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Attribute_<ContainerAllocator>>::other>;
  _attributes_type attributes;
  using _points_type =
    std::vector<ai_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Point_<ContainerAllocator>>::other>;
  _points_type points;
  using _captures_type =
    std::vector<ai_msgs::msg::Capture_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Capture_<ContainerAllocator>>::other>;
  _captures_type captures;

  // setters for named parameter idiom
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__track_id(
    const uint64_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }
  Type & set__rois(
    const std::vector<ai_msgs::msg::Roi_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Roi_<ContainerAllocator>>::other> & _arg)
  {
    this->rois = _arg;
    return *this;
  }
  Type & set__attributes(
    const std::vector<ai_msgs::msg::Attribute_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Attribute_<ContainerAllocator>>::other> & _arg)
  {
    this->attributes = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<ai_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__captures(
    const std::vector<ai_msgs::msg::Capture_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Capture_<ContainerAllocator>>::other> & _arg)
  {
    this->captures = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::Target_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::Target_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::Target_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::Target_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Target_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Target_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::Target_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::Target_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::Target_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::Target_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__Target
    std::shared_ptr<ai_msgs::msg::Target_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__Target
    std::shared_ptr<ai_msgs::msg::Target_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Target_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    if (this->rois != other.rois) {
      return false;
    }
    if (this->attributes != other.attributes) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->captures != other.captures) {
      return false;
    }
    return true;
  }
  bool operator!=(const Target_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Target_

// alias to use template instance with default allocator
using Target =
  ai_msgs::msg::Target_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__TARGET__STRUCT_HPP_

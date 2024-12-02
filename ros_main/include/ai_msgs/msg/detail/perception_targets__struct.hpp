// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/PerceptionTargets.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_HPP_

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
// Member 'perfs'
#include "ai_msgs/msg/detail/perf__struct.hpp"
// Member 'targets'
// Member 'disappeared_targets'
#include "ai_msgs/msg/detail/target__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__PerceptionTargets __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__PerceptionTargets __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PerceptionTargets_
{
  using Type = PerceptionTargets_<ContainerAllocator>;

  explicit PerceptionTargets_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fps = 0;
    }
  }

  explicit PerceptionTargets_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fps = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fps_type =
    int16_t;
  _fps_type fps;
  using _perfs_type =
    std::vector<ai_msgs::msg::Perf_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Perf_<ContainerAllocator>>::other>;
  _perfs_type perfs;
  using _targets_type =
    std::vector<ai_msgs::msg::Target_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Target_<ContainerAllocator>>::other>;
  _targets_type targets;
  using _disappeared_targets_type =
    std::vector<ai_msgs::msg::Target_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Target_<ContainerAllocator>>::other>;
  _disappeared_targets_type disappeared_targets;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fps(
    const int16_t & _arg)
  {
    this->fps = _arg;
    return *this;
  }
  Type & set__perfs(
    const std::vector<ai_msgs::msg::Perf_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Perf_<ContainerAllocator>>::other> & _arg)
  {
    this->perfs = _arg;
    return *this;
  }
  Type & set__targets(
    const std::vector<ai_msgs::msg::Target_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Target_<ContainerAllocator>>::other> & _arg)
  {
    this->targets = _arg;
    return *this;
  }
  Type & set__disappeared_targets(
    const std::vector<ai_msgs::msg::Target_<ContainerAllocator>, typename ContainerAllocator::template rebind<ai_msgs::msg::Target_<ContainerAllocator>>::other> & _arg)
  {
    this->disappeared_targets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::PerceptionTargets_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::PerceptionTargets_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::PerceptionTargets_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::PerceptionTargets_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__PerceptionTargets
    std::shared_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__PerceptionTargets
    std::shared_ptr<ai_msgs::msg::PerceptionTargets_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PerceptionTargets_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    if (this->perfs != other.perfs) {
      return false;
    }
    if (this->targets != other.targets) {
      return false;
    }
    if (this->disappeared_targets != other.disappeared_targets) {
      return false;
    }
    return true;
  }
  bool operator!=(const PerceptionTargets_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PerceptionTargets_

// alias to use template instance with default allocator
using PerceptionTargets =
  ai_msgs::msg::PerceptionTargets_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__PERCEPTION_TARGETS__STRUCT_HPP_
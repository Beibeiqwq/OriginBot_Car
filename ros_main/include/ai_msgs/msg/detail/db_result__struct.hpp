// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ai_msgs:msg/DBResult.idl
// generated code does not contain a copyright notice

#ifndef AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_HPP_
#define AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__ai_msgs__msg__DBResult __attribute__((deprecated))
#else
# define DEPRECATED__ai_msgs__msg__DBResult __declspec(deprecated)
#endif

namespace ai_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DBResult_
{
  using Type = DBResult_<ContainerAllocator>;

  explicit DBResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->db_type = "";
      this->match_id = "";
      this->distance = 0.0f;
      this->similarity = 0.0f;
    }
  }

  explicit DBResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : db_type(_alloc),
    match_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->db_type = "";
      this->match_id = "";
      this->distance = 0.0f;
      this->similarity = 0.0f;
    }
  }

  // field types and members
  using _db_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _db_type_type db_type;
  using _match_id_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _match_id_type match_id;
  using _distance_type =
    float;
  _distance_type distance;
  using _similarity_type =
    float;
  _similarity_type similarity;

  // setters for named parameter idiom
  Type & set__db_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->db_type = _arg;
    return *this;
  }
  Type & set__match_id(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->match_id = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__similarity(
    const float & _arg)
  {
    this->similarity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ai_msgs::msg::DBResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const ai_msgs::msg::DBResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ai_msgs::msg::DBResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ai_msgs::msg::DBResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::DBResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::DBResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ai_msgs::msg::DBResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ai_msgs::msg::DBResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ai_msgs::msg::DBResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ai_msgs::msg::DBResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ai_msgs__msg__DBResult
    std::shared_ptr<ai_msgs::msg::DBResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ai_msgs__msg__DBResult
    std::shared_ptr<ai_msgs::msg::DBResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DBResult_ & other) const
  {
    if (this->db_type != other.db_type) {
      return false;
    }
    if (this->match_id != other.match_id) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->similarity != other.similarity) {
      return false;
    }
    return true;
  }
  bool operator!=(const DBResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DBResult_

// alias to use template instance with default allocator
using DBResult =
  ai_msgs::msg::DBResult_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ai_msgs

#endif  // AI_MSGS__MSG__DETAIL__DB_RESULT__STRUCT_HPP_

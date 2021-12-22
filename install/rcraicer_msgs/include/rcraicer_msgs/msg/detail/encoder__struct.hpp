// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/Encoder.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__Encoder __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__Encoder __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Encoder_
{
  using Type = Encoder_<ContainerAllocator>;

  explicit Encoder_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_rear = 0l;
      this->left_front = 0l;
      this->right_front = 0l;
      this->right_rear = 0l;
    }
  }

  explicit Encoder_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_rear = 0l;
      this->left_front = 0l;
      this->right_front = 0l;
      this->right_rear = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_rear_type =
    int32_t;
  _left_rear_type left_rear;
  using _left_front_type =
    int32_t;
  _left_front_type left_front;
  using _right_front_type =
    int32_t;
  _right_front_type right_front;
  using _right_rear_type =
    int32_t;
  _right_rear_type right_rear;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_rear(
    const int32_t & _arg)
  {
    this->left_rear = _arg;
    return *this;
  }
  Type & set__left_front(
    const int32_t & _arg)
  {
    this->left_front = _arg;
    return *this;
  }
  Type & set__right_front(
    const int32_t & _arg)
  {
    this->right_front = _arg;
    return *this;
  }
  Type & set__right_rear(
    const int32_t & _arg)
  {
    this->right_rear = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::Encoder_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::Encoder_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::Encoder_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::Encoder_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__Encoder
    std::shared_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__Encoder
    std::shared_ptr<rcraicer_msgs::msg::Encoder_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Encoder_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_rear != other.left_rear) {
      return false;
    }
    if (this->left_front != other.left_front) {
      return false;
    }
    if (this->right_front != other.right_front) {
      return false;
    }
    if (this->right_rear != other.right_rear) {
      return false;
    }
    return true;
  }
  bool operator!=(const Encoder_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Encoder_

// alias to use template instance with default allocator
using Encoder =
  rcraicer_msgs::msg::Encoder_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_HPP_

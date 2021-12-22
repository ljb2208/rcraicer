// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/ChassisState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__ChassisState __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__ChassisState __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChassisState_
{
  using Type = ChassisState_<ContainerAllocator>;

  explicit ChassisState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->throttle = 0.0;
      this->steer = 0.0;
      this->steer_angle = 0.0;
    }
  }

  explicit ChassisState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->throttle = 0.0;
      this->steer = 0.0;
      this->steer_angle = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _armed_type =
    bool;
  _armed_type armed;
  using _throttle_type =
    double;
  _throttle_type throttle;
  using _steer_type =
    double;
  _steer_type steer;
  using _steer_angle_type =
    double;
  _steer_angle_type steer_angle;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__throttle(
    const double & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__steer(
    const double & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__steer_angle(
    const double & _arg)
  {
    this->steer_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::ChassisState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::ChassisState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ChassisState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ChassisState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__ChassisState
    std::shared_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__ChassisState
    std::shared_ptr<rcraicer_msgs::msg::ChassisState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChassisState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->armed != other.armed) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    if (this->steer_angle != other.steer_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChassisState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChassisState_

// alias to use template instance with default allocator
using ChassisState =
  rcraicer_msgs::msg::ChassisState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_HPP_

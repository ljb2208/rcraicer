// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/LapStats.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__LapStats __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__LapStats __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LapStats_
{
  using Type = LapStats_<ContainerAllocator>;

  explicit LapStats_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_number = 0ll;
      this->lap_time = 0.0;
      this->max_speed = 0.0;
      this->max_slip = 0.0;
    }
  }

  explicit LapStats_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lap_number = 0ll;
      this->lap_time = 0.0;
      this->max_speed = 0.0;
      this->max_slip = 0.0;
    }
  }

  // field types and members
  using _lap_number_type =
    int64_t;
  _lap_number_type lap_number;
  using _lap_time_type =
    double;
  _lap_time_type lap_time;
  using _max_speed_type =
    double;
  _max_speed_type max_speed;
  using _max_slip_type =
    double;
  _max_slip_type max_slip;

  // setters for named parameter idiom
  Type & set__lap_number(
    const int64_t & _arg)
  {
    this->lap_number = _arg;
    return *this;
  }
  Type & set__lap_time(
    const double & _arg)
  {
    this->lap_time = _arg;
    return *this;
  }
  Type & set__max_speed(
    const double & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__max_slip(
    const double & _arg)
  {
    this->max_slip = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::LapStats_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::LapStats_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::LapStats_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::LapStats_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__LapStats
    std::shared_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__LapStats
    std::shared_ptr<rcraicer_msgs::msg::LapStats_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LapStats_ & other) const
  {
    if (this->lap_number != other.lap_number) {
      return false;
    }
    if (this->lap_time != other.lap_time) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->max_slip != other.max_slip) {
      return false;
    }
    return true;
  }
  bool operator!=(const LapStats_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LapStats_

// alias to use template instance with default allocator
using LapStats =
  rcraicer_msgs::msg::LapStats_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/GPSSurvey.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__GPSSurvey __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__GPSSurvey __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSSurvey_
{
  using Type = GPSSurvey_<ContainerAllocator>;

  explicit GPSSurvey_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mean_x = 0l;
      this->mean_y = 0l;
      this->mean_z = 0l;
      this->observations = 0l;
      this->duration = 0l;
      this->accuracy = 0.0f;
      this->valid = 0;
      this->active = 0;
    }
  }

  explicit GPSSurvey_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mean_x = 0l;
      this->mean_y = 0l;
      this->mean_z = 0l;
      this->observations = 0l;
      this->duration = 0l;
      this->accuracy = 0.0f;
      this->valid = 0;
      this->active = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mean_x_type =
    int32_t;
  _mean_x_type mean_x;
  using _mean_y_type =
    int32_t;
  _mean_y_type mean_y;
  using _mean_z_type =
    int32_t;
  _mean_z_type mean_z;
  using _observations_type =
    int32_t;
  _observations_type observations;
  using _duration_type =
    int32_t;
  _duration_type duration;
  using _accuracy_type =
    float;
  _accuracy_type accuracy;
  using _valid_type =
    int16_t;
  _valid_type valid;
  using _active_type =
    int16_t;
  _active_type active;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mean_x(
    const int32_t & _arg)
  {
    this->mean_x = _arg;
    return *this;
  }
  Type & set__mean_y(
    const int32_t & _arg)
  {
    this->mean_y = _arg;
    return *this;
  }
  Type & set__mean_z(
    const int32_t & _arg)
  {
    this->mean_z = _arg;
    return *this;
  }
  Type & set__observations(
    const int32_t & _arg)
  {
    this->observations = _arg;
    return *this;
  }
  Type & set__duration(
    const int32_t & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__accuracy(
    const float & _arg)
  {
    this->accuracy = _arg;
    return *this;
  }
  Type & set__valid(
    const int16_t & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__active(
    const int16_t & _arg)
  {
    this->active = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__GPSSurvey
    std::shared_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__GPSSurvey
    std::shared_ptr<rcraicer_msgs::msg::GPSSurvey_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSSurvey_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mean_x != other.mean_x) {
      return false;
    }
    if (this->mean_y != other.mean_y) {
      return false;
    }
    if (this->mean_z != other.mean_z) {
      return false;
    }
    if (this->observations != other.observations) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->accuracy != other.accuracy) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSSurvey_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSSurvey_

// alias to use template instance with default allocator
using GPSSurvey =
  rcraicer_msgs::msg::GPSSurvey_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_HPP_

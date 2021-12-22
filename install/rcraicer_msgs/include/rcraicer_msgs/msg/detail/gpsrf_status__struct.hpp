// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/GPSRFStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__GPSRFStatus __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__GPSRFStatus __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSRFStatus_
{
  using Type = GPSRFStatus_<ContainerAllocator>;

  explicit GPSRFStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->block1_jamming = 0;
      this->block1_antenna_status = 0;
      this->block1_antenna_power = 0;
      this->block1_noise = 0l;
      this->block1_jamming_ind = 0;
      this->block2_jamming = 0;
      this->block2_antenna_status = 0;
      this->block2_antenna_power = 0;
      this->block2_noise = 0l;
      this->block2_jamming_ind = 0;
    }
  }

  explicit GPSRFStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->block1_jamming = 0;
      this->block1_antenna_status = 0;
      this->block1_antenna_power = 0;
      this->block1_noise = 0l;
      this->block1_jamming_ind = 0;
      this->block2_jamming = 0;
      this->block2_antenna_status = 0;
      this->block2_antenna_power = 0;
      this->block2_noise = 0l;
      this->block2_jamming_ind = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _block1_jamming_type =
    int16_t;
  _block1_jamming_type block1_jamming;
  using _block1_antenna_status_type =
    int16_t;
  _block1_antenna_status_type block1_antenna_status;
  using _block1_antenna_power_type =
    int16_t;
  _block1_antenna_power_type block1_antenna_power;
  using _block1_noise_type =
    int32_t;
  _block1_noise_type block1_noise;
  using _block1_jamming_ind_type =
    int16_t;
  _block1_jamming_ind_type block1_jamming_ind;
  using _block2_jamming_type =
    int16_t;
  _block2_jamming_type block2_jamming;
  using _block2_antenna_status_type =
    int16_t;
  _block2_antenna_status_type block2_antenna_status;
  using _block2_antenna_power_type =
    int16_t;
  _block2_antenna_power_type block2_antenna_power;
  using _block2_noise_type =
    int32_t;
  _block2_noise_type block2_noise;
  using _block2_jamming_ind_type =
    int16_t;
  _block2_jamming_ind_type block2_jamming_ind;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__block1_jamming(
    const int16_t & _arg)
  {
    this->block1_jamming = _arg;
    return *this;
  }
  Type & set__block1_antenna_status(
    const int16_t & _arg)
  {
    this->block1_antenna_status = _arg;
    return *this;
  }
  Type & set__block1_antenna_power(
    const int16_t & _arg)
  {
    this->block1_antenna_power = _arg;
    return *this;
  }
  Type & set__block1_noise(
    const int32_t & _arg)
  {
    this->block1_noise = _arg;
    return *this;
  }
  Type & set__block1_jamming_ind(
    const int16_t & _arg)
  {
    this->block1_jamming_ind = _arg;
    return *this;
  }
  Type & set__block2_jamming(
    const int16_t & _arg)
  {
    this->block2_jamming = _arg;
    return *this;
  }
  Type & set__block2_antenna_status(
    const int16_t & _arg)
  {
    this->block2_antenna_status = _arg;
    return *this;
  }
  Type & set__block2_antenna_power(
    const int16_t & _arg)
  {
    this->block2_antenna_power = _arg;
    return *this;
  }
  Type & set__block2_noise(
    const int32_t & _arg)
  {
    this->block2_noise = _arg;
    return *this;
  }
  Type & set__block2_jamming_ind(
    const int16_t & _arg)
  {
    this->block2_jamming_ind = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__GPSRFStatus
    std::shared_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__GPSRFStatus
    std::shared_ptr<rcraicer_msgs::msg::GPSRFStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSRFStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->block1_jamming != other.block1_jamming) {
      return false;
    }
    if (this->block1_antenna_status != other.block1_antenna_status) {
      return false;
    }
    if (this->block1_antenna_power != other.block1_antenna_power) {
      return false;
    }
    if (this->block1_noise != other.block1_noise) {
      return false;
    }
    if (this->block1_jamming_ind != other.block1_jamming_ind) {
      return false;
    }
    if (this->block2_jamming != other.block2_jamming) {
      return false;
    }
    if (this->block2_antenna_status != other.block2_antenna_status) {
      return false;
    }
    if (this->block2_antenna_power != other.block2_antenna_power) {
      return false;
    }
    if (this->block2_noise != other.block2_noise) {
      return false;
    }
    if (this->block2_jamming_ind != other.block2_jamming_ind) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSRFStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSRFStatus_

// alias to use template instance with default allocator
using GPSRFStatus =
  rcraicer_msgs::msg::GPSRFStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_HPP_

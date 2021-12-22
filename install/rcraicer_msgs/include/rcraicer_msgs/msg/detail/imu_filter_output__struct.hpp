// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/ImuFilterOutput.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_HPP_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'bias'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__ImuFilterOutput __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__ImuFilterOutput __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuFilterOutput_
{
  using Type = ImuFilterOutput_<ContainerAllocator>;

  explicit ImuFilterOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    bias(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quat_status = 0;
      this->bias_status = 0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->orientation_covariance.begin(), this->orientation_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->bias_covariance.begin(), this->bias_covariance.end(), 0.0);
      this->bias_covariance_status = 0;
      this->orientation_covariance_status = 0;
    }
  }

  explicit ImuFilterOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    orientation_covariance(_alloc),
    bias(_alloc, _init),
    bias_covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quat_status = 0;
      this->bias_status = 0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->orientation_covariance.begin(), this->orientation_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->bias_covariance.begin(), this->bias_covariance.end(), 0.0);
      this->bias_covariance_status = 0;
      this->orientation_covariance_status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quat_status_type =
    uint16_t;
  _quat_status_type quat_status;
  using _bias_status_type =
    uint16_t;
  _bias_status_type bias_status;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _orientation_covariance_type =
    std::array<double, 9>;
  _orientation_covariance_type orientation_covariance;
  using _bias_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _bias_type bias;
  using _bias_covariance_type =
    std::array<double, 9>;
  _bias_covariance_type bias_covariance;
  using _bias_covariance_status_type =
    uint16_t;
  _bias_covariance_status_type bias_covariance_status;
  using _orientation_covariance_status_type =
    uint16_t;
  _orientation_covariance_status_type orientation_covariance_status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quat_status(
    const uint16_t & _arg)
  {
    this->quat_status = _arg;
    return *this;
  }
  Type & set__bias_status(
    const uint16_t & _arg)
  {
    this->bias_status = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__orientation_covariance(
    const std::array<double, 9> & _arg)
  {
    this->orientation_covariance = _arg;
    return *this;
  }
  Type & set__bias(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->bias = _arg;
    return *this;
  }
  Type & set__bias_covariance(
    const std::array<double, 9> & _arg)
  {
    this->bias_covariance = _arg;
    return *this;
  }
  Type & set__bias_covariance_status(
    const uint16_t & _arg)
  {
    this->bias_covariance_status = _arg;
    return *this;
  }
  Type & set__orientation_covariance_status(
    const uint16_t & _arg)
  {
    this->orientation_covariance_status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t STATUS_INVALID =
    0u;
  static constexpr uint16_t STATUS_VALID =
    1u;
  static constexpr uint16_t STATUS_VALID_REFERENCED =
    2u;

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__ImuFilterOutput
    std::shared_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__ImuFilterOutput
    std::shared_ptr<rcraicer_msgs::msg::ImuFilterOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuFilterOutput_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quat_status != other.quat_status) {
      return false;
    }
    if (this->bias_status != other.bias_status) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->orientation_covariance != other.orientation_covariance) {
      return false;
    }
    if (this->bias != other.bias) {
      return false;
    }
    if (this->bias_covariance != other.bias_covariance) {
      return false;
    }
    if (this->bias_covariance_status != other.bias_covariance_status) {
      return false;
    }
    if (this->orientation_covariance_status != other.orientation_covariance_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuFilterOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuFilterOutput_

// alias to use template instance with default allocator
using ImuFilterOutput =
  rcraicer_msgs::msg::ImuFilterOutput_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint16_t ImuFilterOutput_<ContainerAllocator>::STATUS_INVALID;
template<typename ContainerAllocator>
constexpr uint16_t ImuFilterOutput_<ContainerAllocator>::STATUS_VALID;
template<typename ContainerAllocator>
constexpr uint16_t ImuFilterOutput_<ContainerAllocator>::STATUS_VALID_REFERENCED;

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_HPP_

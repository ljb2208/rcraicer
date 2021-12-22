// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/ArduinoStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__ArduinoStatus __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__ArduinoStatus __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArduinoStatus_
{
  using Type = ArduinoStatus_<ContainerAllocator>;

  explicit ArduinoStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_update_count = 0;
      this->encoder_msg_count = 0;
      this->main_loop_count = 0;
      this->main_loop_max = 0;
      this->armed = false;
      this->status = 0;
      this->invalid_crc_arduino = 0;
      this->unknown_msg_arduino = 0;
      this->invalid_crc = 0;
      this->unknown_msg = 0;
    }
  }

  explicit ArduinoStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_update_count = 0;
      this->encoder_msg_count = 0;
      this->main_loop_count = 0;
      this->main_loop_max = 0;
      this->armed = false;
      this->status = 0;
      this->invalid_crc_arduino = 0;
      this->unknown_msg_arduino = 0;
      this->invalid_crc = 0;
      this->unknown_msg = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _servo_update_count_type =
    uint16_t;
  _servo_update_count_type servo_update_count;
  using _encoder_msg_count_type =
    uint16_t;
  _encoder_msg_count_type encoder_msg_count;
  using _main_loop_count_type =
    uint16_t;
  _main_loop_count_type main_loop_count;
  using _main_loop_max_type =
    uint16_t;
  _main_loop_max_type main_loop_max;
  using _armed_type =
    bool;
  _armed_type armed;
  using _status_type =
    uint8_t;
  _status_type status;
  using _invalid_crc_arduino_type =
    uint16_t;
  _invalid_crc_arduino_type invalid_crc_arduino;
  using _unknown_msg_arduino_type =
    uint16_t;
  _unknown_msg_arduino_type unknown_msg_arduino;
  using _invalid_crc_type =
    uint16_t;
  _invalid_crc_type invalid_crc;
  using _unknown_msg_type =
    uint16_t;
  _unknown_msg_type unknown_msg;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__servo_update_count(
    const uint16_t & _arg)
  {
    this->servo_update_count = _arg;
    return *this;
  }
  Type & set__encoder_msg_count(
    const uint16_t & _arg)
  {
    this->encoder_msg_count = _arg;
    return *this;
  }
  Type & set__main_loop_count(
    const uint16_t & _arg)
  {
    this->main_loop_count = _arg;
    return *this;
  }
  Type & set__main_loop_max(
    const uint16_t & _arg)
  {
    this->main_loop_max = _arg;
    return *this;
  }
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__invalid_crc_arduino(
    const uint16_t & _arg)
  {
    this->invalid_crc_arduino = _arg;
    return *this;
  }
  Type & set__unknown_msg_arduino(
    const uint16_t & _arg)
  {
    this->unknown_msg_arduino = _arg;
    return *this;
  }
  Type & set__invalid_crc(
    const uint16_t & _arg)
  {
    this->invalid_crc = _arg;
    return *this;
  }
  Type & set__unknown_msg(
    const uint16_t & _arg)
  {
    this->unknown_msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__ArduinoStatus
    std::shared_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__ArduinoStatus
    std::shared_ptr<rcraicer_msgs::msg::ArduinoStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArduinoStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->servo_update_count != other.servo_update_count) {
      return false;
    }
    if (this->encoder_msg_count != other.encoder_msg_count) {
      return false;
    }
    if (this->main_loop_count != other.main_loop_count) {
      return false;
    }
    if (this->main_loop_max != other.main_loop_max) {
      return false;
    }
    if (this->armed != other.armed) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->invalid_crc_arduino != other.invalid_crc_arduino) {
      return false;
    }
    if (this->unknown_msg_arduino != other.unknown_msg_arduino) {
      return false;
    }
    if (this->invalid_crc != other.invalid_crc) {
      return false;
    }
    if (this->unknown_msg != other.unknown_msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArduinoStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArduinoStatus_

// alias to use template instance with default allocator
using ArduinoStatus =
  rcraicer_msgs::msg::ArduinoStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_HPP_

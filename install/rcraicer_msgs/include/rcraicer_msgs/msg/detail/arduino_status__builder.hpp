// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/ArduinoStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/arduino_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_ArduinoStatus_unknown_msg
{
public:
  explicit Init_ArduinoStatus_unknown_msg(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::ArduinoStatus unknown_msg(::rcraicer_msgs::msg::ArduinoStatus::_unknown_msg_type arg)
  {
    msg_.unknown_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_invalid_crc
{
public:
  explicit Init_ArduinoStatus_invalid_crc(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_unknown_msg invalid_crc(::rcraicer_msgs::msg::ArduinoStatus::_invalid_crc_type arg)
  {
    msg_.invalid_crc = std::move(arg);
    return Init_ArduinoStatus_unknown_msg(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_unknown_msg_arduino
{
public:
  explicit Init_ArduinoStatus_unknown_msg_arduino(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_invalid_crc unknown_msg_arduino(::rcraicer_msgs::msg::ArduinoStatus::_unknown_msg_arduino_type arg)
  {
    msg_.unknown_msg_arduino = std::move(arg);
    return Init_ArduinoStatus_invalid_crc(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_invalid_crc_arduino
{
public:
  explicit Init_ArduinoStatus_invalid_crc_arduino(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_unknown_msg_arduino invalid_crc_arduino(::rcraicer_msgs::msg::ArduinoStatus::_invalid_crc_arduino_type arg)
  {
    msg_.invalid_crc_arduino = std::move(arg);
    return Init_ArduinoStatus_unknown_msg_arduino(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_status
{
public:
  explicit Init_ArduinoStatus_status(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_invalid_crc_arduino status(::rcraicer_msgs::msg::ArduinoStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ArduinoStatus_invalid_crc_arduino(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_armed
{
public:
  explicit Init_ArduinoStatus_armed(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_status armed(::rcraicer_msgs::msg::ArduinoStatus::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_ArduinoStatus_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_main_loop_max
{
public:
  explicit Init_ArduinoStatus_main_loop_max(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_armed main_loop_max(::rcraicer_msgs::msg::ArduinoStatus::_main_loop_max_type arg)
  {
    msg_.main_loop_max = std::move(arg);
    return Init_ArduinoStatus_armed(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_main_loop_count
{
public:
  explicit Init_ArduinoStatus_main_loop_count(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_main_loop_max main_loop_count(::rcraicer_msgs::msg::ArduinoStatus::_main_loop_count_type arg)
  {
    msg_.main_loop_count = std::move(arg);
    return Init_ArduinoStatus_main_loop_max(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_encoder_msg_count
{
public:
  explicit Init_ArduinoStatus_encoder_msg_count(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_main_loop_count encoder_msg_count(::rcraicer_msgs::msg::ArduinoStatus::_encoder_msg_count_type arg)
  {
    msg_.encoder_msg_count = std::move(arg);
    return Init_ArduinoStatus_main_loop_count(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_servo_update_count
{
public:
  explicit Init_ArduinoStatus_servo_update_count(::rcraicer_msgs::msg::ArduinoStatus & msg)
  : msg_(msg)
  {}
  Init_ArduinoStatus_encoder_msg_count servo_update_count(::rcraicer_msgs::msg::ArduinoStatus::_servo_update_count_type arg)
  {
    msg_.servo_update_count = std::move(arg);
    return Init_ArduinoStatus_encoder_msg_count(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

class Init_ArduinoStatus_header
{
public:
  Init_ArduinoStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArduinoStatus_servo_update_count header(::rcraicer_msgs::msg::ArduinoStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArduinoStatus_servo_update_count(msg_);
  }

private:
  ::rcraicer_msgs::msg::ArduinoStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::ArduinoStatus>()
{
  return rcraicer_msgs::msg::builder::Init_ArduinoStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__BUILDER_HPP_

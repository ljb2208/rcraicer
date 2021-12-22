// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/ChassisCommand.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/chassis_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_ChassisCommand_sender
{
public:
  explicit Init_ChassisCommand_sender(::rcraicer_msgs::msg::ChassisCommand & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::ChassisCommand sender(::rcraicer_msgs::msg::ChassisCommand::_sender_type arg)
  {
    msg_.sender = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisCommand msg_;
};

class Init_ChassisCommand_steer
{
public:
  explicit Init_ChassisCommand_steer(::rcraicer_msgs::msg::ChassisCommand & msg)
  : msg_(msg)
  {}
  Init_ChassisCommand_sender steer(::rcraicer_msgs::msg::ChassisCommand::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_ChassisCommand_sender(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisCommand msg_;
};

class Init_ChassisCommand_throttle
{
public:
  explicit Init_ChassisCommand_throttle(::rcraicer_msgs::msg::ChassisCommand & msg)
  : msg_(msg)
  {}
  Init_ChassisCommand_steer throttle(::rcraicer_msgs::msg::ChassisCommand::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_ChassisCommand_steer(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisCommand msg_;
};

class Init_ChassisCommand_header
{
public:
  Init_ChassisCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisCommand_throttle header(::rcraicer_msgs::msg::ChassisCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChassisCommand_throttle(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::ChassisCommand>()
{
  return rcraicer_msgs::msg::builder::Init_ChassisCommand_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__BUILDER_HPP_

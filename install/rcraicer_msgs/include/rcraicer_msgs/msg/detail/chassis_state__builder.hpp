// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/ChassisState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/chassis_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_ChassisState_steer_angle
{
public:
  explicit Init_ChassisState_steer_angle(::rcraicer_msgs::msg::ChassisState & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::ChassisState steer_angle(::rcraicer_msgs::msg::ChassisState::_steer_angle_type arg)
  {
    msg_.steer_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisState msg_;
};

class Init_ChassisState_steer
{
public:
  explicit Init_ChassisState_steer(::rcraicer_msgs::msg::ChassisState & msg)
  : msg_(msg)
  {}
  Init_ChassisState_steer_angle steer(::rcraicer_msgs::msg::ChassisState::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_ChassisState_steer_angle(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisState msg_;
};

class Init_ChassisState_throttle
{
public:
  explicit Init_ChassisState_throttle(::rcraicer_msgs::msg::ChassisState & msg)
  : msg_(msg)
  {}
  Init_ChassisState_steer throttle(::rcraicer_msgs::msg::ChassisState::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_ChassisState_steer(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisState msg_;
};

class Init_ChassisState_armed
{
public:
  explicit Init_ChassisState_armed(::rcraicer_msgs::msg::ChassisState & msg)
  : msg_(msg)
  {}
  Init_ChassisState_throttle armed(::rcraicer_msgs::msg::ChassisState::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_ChassisState_throttle(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisState msg_;
};

class Init_ChassisState_header
{
public:
  Init_ChassisState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisState_armed header(::rcraicer_msgs::msg::ChassisState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChassisState_armed(msg_);
  }

private:
  ::rcraicer_msgs::msg::ChassisState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::ChassisState>()
{
  return rcraicer_msgs::msg::builder::Init_ChassisState_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__BUILDER_HPP_

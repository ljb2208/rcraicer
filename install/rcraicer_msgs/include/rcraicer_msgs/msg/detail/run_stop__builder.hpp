// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/RunStop.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__RUN_STOP__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__RUN_STOP__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/run_stop__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_RunStop_motion_enabled
{
public:
  explicit Init_RunStop_motion_enabled(::rcraicer_msgs::msg::RunStop & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::RunStop motion_enabled(::rcraicer_msgs::msg::RunStop::_motion_enabled_type arg)
  {
    msg_.motion_enabled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::RunStop msg_;
};

class Init_RunStop_sender
{
public:
  explicit Init_RunStop_sender(::rcraicer_msgs::msg::RunStop & msg)
  : msg_(msg)
  {}
  Init_RunStop_motion_enabled sender(::rcraicer_msgs::msg::RunStop::_sender_type arg)
  {
    msg_.sender = std::move(arg);
    return Init_RunStop_motion_enabled(msg_);
  }

private:
  ::rcraicer_msgs::msg::RunStop msg_;
};

class Init_RunStop_header
{
public:
  Init_RunStop_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RunStop_sender header(::rcraicer_msgs::msg::RunStop::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RunStop_sender(msg_);
  }

private:
  ::rcraicer_msgs::msg::RunStop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::RunStop>()
{
  return rcraicer_msgs::msg::builder::Init_RunStop_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__RUN_STOP__BUILDER_HPP_

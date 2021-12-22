// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/PathIntegralStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/path_integral_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_PathIntegralStatus_status
{
public:
  explicit Init_PathIntegralStatus_status(::rcraicer_msgs::msg::PathIntegralStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::PathIntegralStatus status(::rcraicer_msgs::msg::PathIntegralStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStatus msg_;
};

class Init_PathIntegralStatus_info
{
public:
  explicit Init_PathIntegralStatus_info(::rcraicer_msgs::msg::PathIntegralStatus & msg)
  : msg_(msg)
  {}
  Init_PathIntegralStatus_status info(::rcraicer_msgs::msg::PathIntegralStatus::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PathIntegralStatus_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStatus msg_;
};

class Init_PathIntegralStatus_header
{
public:
  Init_PathIntegralStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathIntegralStatus_info header(::rcraicer_msgs::msg::PathIntegralStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PathIntegralStatus_info(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::PathIntegralStatus>()
{
  return rcraicer_msgs::msg::builder::Init_PathIntegralStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__BUILDER_HPP_

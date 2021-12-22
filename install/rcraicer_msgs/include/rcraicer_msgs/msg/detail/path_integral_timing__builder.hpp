// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/PathIntegralTiming.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_TIMING__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_TIMING__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/path_integral_timing__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_PathIntegralTiming_average_sleep_time
{
public:
  explicit Init_PathIntegralTiming_average_sleep_time(::rcraicer_msgs::msg::PathIntegralTiming & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::PathIntegralTiming average_sleep_time(::rcraicer_msgs::msg::PathIntegralTiming::_average_sleep_time_type arg)
  {
    msg_.average_sleep_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralTiming msg_;
};

class Init_PathIntegralTiming_average_optimization_cycle_time
{
public:
  explicit Init_PathIntegralTiming_average_optimization_cycle_time(::rcraicer_msgs::msg::PathIntegralTiming & msg)
  : msg_(msg)
  {}
  Init_PathIntegralTiming_average_sleep_time average_optimization_cycle_time(::rcraicer_msgs::msg::PathIntegralTiming::_average_optimization_cycle_time_type arg)
  {
    msg_.average_optimization_cycle_time = std::move(arg);
    return Init_PathIntegralTiming_average_sleep_time(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralTiming msg_;
};

class Init_PathIntegralTiming_average_time_between_poses
{
public:
  explicit Init_PathIntegralTiming_average_time_between_poses(::rcraicer_msgs::msg::PathIntegralTiming & msg)
  : msg_(msg)
  {}
  Init_PathIntegralTiming_average_optimization_cycle_time average_time_between_poses(::rcraicer_msgs::msg::PathIntegralTiming::_average_time_between_poses_type arg)
  {
    msg_.average_time_between_poses = std::move(arg);
    return Init_PathIntegralTiming_average_optimization_cycle_time(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralTiming msg_;
};

class Init_PathIntegralTiming_header
{
public:
  Init_PathIntegralTiming_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathIntegralTiming_average_time_between_poses header(::rcraicer_msgs::msg::PathIntegralTiming::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PathIntegralTiming_average_time_between_poses(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralTiming msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::PathIntegralTiming>()
{
  return rcraicer_msgs::msg::builder::Init_PathIntegralTiming_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_TIMING__BUILDER_HPP_

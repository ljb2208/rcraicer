// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/LapStats.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/lap_stats__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_LapStats_max_slip
{
public:
  explicit Init_LapStats_max_slip(::rcraicer_msgs::msg::LapStats & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::LapStats max_slip(::rcraicer_msgs::msg::LapStats::_max_slip_type arg)
  {
    msg_.max_slip = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::LapStats msg_;
};

class Init_LapStats_max_speed
{
public:
  explicit Init_LapStats_max_speed(::rcraicer_msgs::msg::LapStats & msg)
  : msg_(msg)
  {}
  Init_LapStats_max_slip max_speed(::rcraicer_msgs::msg::LapStats::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_LapStats_max_slip(msg_);
  }

private:
  ::rcraicer_msgs::msg::LapStats msg_;
};

class Init_LapStats_lap_time
{
public:
  explicit Init_LapStats_lap_time(::rcraicer_msgs::msg::LapStats & msg)
  : msg_(msg)
  {}
  Init_LapStats_max_speed lap_time(::rcraicer_msgs::msg::LapStats::_lap_time_type arg)
  {
    msg_.lap_time = std::move(arg);
    return Init_LapStats_max_speed(msg_);
  }

private:
  ::rcraicer_msgs::msg::LapStats msg_;
};

class Init_LapStats_lap_number
{
public:
  Init_LapStats_lap_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LapStats_lap_time lap_number(::rcraicer_msgs::msg::LapStats::_lap_number_type arg)
  {
    msg_.lap_number = std::move(arg);
    return Init_LapStats_lap_time(msg_);
  }

private:
  ::rcraicer_msgs::msg::LapStats msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::LapStats>()
{
  return rcraicer_msgs::msg::builder::Init_LapStats_lap_number();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__BUILDER_HPP_

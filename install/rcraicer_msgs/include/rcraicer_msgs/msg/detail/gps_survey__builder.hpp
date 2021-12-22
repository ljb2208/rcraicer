// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/GPSSurvey.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/gps_survey__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSSurvey_active
{
public:
  explicit Init_GPSSurvey_active(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::GPSSurvey active(::rcraicer_msgs::msg::GPSSurvey::_active_type arg)
  {
    msg_.active = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_valid
{
public:
  explicit Init_GPSSurvey_valid(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_active valid(::rcraicer_msgs::msg::GPSSurvey::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_GPSSurvey_active(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_accuracy
{
public:
  explicit Init_GPSSurvey_accuracy(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_valid accuracy(::rcraicer_msgs::msg::GPSSurvey::_accuracy_type arg)
  {
    msg_.accuracy = std::move(arg);
    return Init_GPSSurvey_valid(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_duration
{
public:
  explicit Init_GPSSurvey_duration(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_accuracy duration(::rcraicer_msgs::msg::GPSSurvey::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_GPSSurvey_accuracy(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_observations
{
public:
  explicit Init_GPSSurvey_observations(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_duration observations(::rcraicer_msgs::msg::GPSSurvey::_observations_type arg)
  {
    msg_.observations = std::move(arg);
    return Init_GPSSurvey_duration(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_mean_z
{
public:
  explicit Init_GPSSurvey_mean_z(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_observations mean_z(::rcraicer_msgs::msg::GPSSurvey::_mean_z_type arg)
  {
    msg_.mean_z = std::move(arg);
    return Init_GPSSurvey_observations(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_mean_y
{
public:
  explicit Init_GPSSurvey_mean_y(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_mean_z mean_y(::rcraicer_msgs::msg::GPSSurvey::_mean_y_type arg)
  {
    msg_.mean_y = std::move(arg);
    return Init_GPSSurvey_mean_z(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_mean_x
{
public:
  explicit Init_GPSSurvey_mean_x(::rcraicer_msgs::msg::GPSSurvey & msg)
  : msg_(msg)
  {}
  Init_GPSSurvey_mean_y mean_x(::rcraicer_msgs::msg::GPSSurvey::_mean_x_type arg)
  {
    msg_.mean_x = std::move(arg);
    return Init_GPSSurvey_mean_y(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

class Init_GPSSurvey_header
{
public:
  Init_GPSSurvey_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSSurvey_mean_x header(::rcraicer_msgs::msg::GPSSurvey::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSSurvey_mean_x(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSSurvey msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::GPSSurvey>()
{
  return rcraicer_msgs::msg::builder::Init_GPSSurvey_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__BUILDER_HPP_

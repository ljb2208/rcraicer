// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/ImuFilterOutput.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/imu_filter_output__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_ImuFilterOutput_orientation_covariance_status
{
public:
  explicit Init_ImuFilterOutput_orientation_covariance_status(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::ImuFilterOutput orientation_covariance_status(::rcraicer_msgs::msg::ImuFilterOutput::_orientation_covariance_status_type arg)
  {
    msg_.orientation_covariance_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_bias_covariance_status
{
public:
  explicit Init_ImuFilterOutput_bias_covariance_status(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_orientation_covariance_status bias_covariance_status(::rcraicer_msgs::msg::ImuFilterOutput::_bias_covariance_status_type arg)
  {
    msg_.bias_covariance_status = std::move(arg);
    return Init_ImuFilterOutput_orientation_covariance_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_bias_covariance
{
public:
  explicit Init_ImuFilterOutput_bias_covariance(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_bias_covariance_status bias_covariance(::rcraicer_msgs::msg::ImuFilterOutput::_bias_covariance_type arg)
  {
    msg_.bias_covariance = std::move(arg);
    return Init_ImuFilterOutput_bias_covariance_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_bias
{
public:
  explicit Init_ImuFilterOutput_bias(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_bias_covariance bias(::rcraicer_msgs::msg::ImuFilterOutput::_bias_type arg)
  {
    msg_.bias = std::move(arg);
    return Init_ImuFilterOutput_bias_covariance(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_orientation_covariance
{
public:
  explicit Init_ImuFilterOutput_orientation_covariance(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_bias orientation_covariance(::rcraicer_msgs::msg::ImuFilterOutput::_orientation_covariance_type arg)
  {
    msg_.orientation_covariance = std::move(arg);
    return Init_ImuFilterOutput_bias(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_orientation
{
public:
  explicit Init_ImuFilterOutput_orientation(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_orientation_covariance orientation(::rcraicer_msgs::msg::ImuFilterOutput::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_ImuFilterOutput_orientation_covariance(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_bias_status
{
public:
  explicit Init_ImuFilterOutput_bias_status(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_orientation bias_status(::rcraicer_msgs::msg::ImuFilterOutput::_bias_status_type arg)
  {
    msg_.bias_status = std::move(arg);
    return Init_ImuFilterOutput_orientation(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_quat_status
{
public:
  explicit Init_ImuFilterOutput_quat_status(::rcraicer_msgs::msg::ImuFilterOutput & msg)
  : msg_(msg)
  {}
  Init_ImuFilterOutput_bias_status quat_status(::rcraicer_msgs::msg::ImuFilterOutput::_quat_status_type arg)
  {
    msg_.quat_status = std::move(arg);
    return Init_ImuFilterOutput_bias_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

class Init_ImuFilterOutput_header
{
public:
  Init_ImuFilterOutput_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuFilterOutput_quat_status header(::rcraicer_msgs::msg::ImuFilterOutput::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ImuFilterOutput_quat_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::ImuFilterOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::ImuFilterOutput>()
{
  return rcraicer_msgs::msg::builder::Init_ImuFilterOutput_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__BUILDER_HPP_

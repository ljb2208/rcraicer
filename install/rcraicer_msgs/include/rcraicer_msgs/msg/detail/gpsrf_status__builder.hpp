// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/GPSRFStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/gpsrf_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSRFStatus_block2_jamming_ind
{
public:
  explicit Init_GPSRFStatus_block2_jamming_ind(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::GPSRFStatus block2_jamming_ind(::rcraicer_msgs::msg::GPSRFStatus::_block2_jamming_ind_type arg)
  {
    msg_.block2_jamming_ind = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block2_noise
{
public:
  explicit Init_GPSRFStatus_block2_noise(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block2_jamming_ind block2_noise(::rcraicer_msgs::msg::GPSRFStatus::_block2_noise_type arg)
  {
    msg_.block2_noise = std::move(arg);
    return Init_GPSRFStatus_block2_jamming_ind(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block2_antenna_power
{
public:
  explicit Init_GPSRFStatus_block2_antenna_power(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block2_noise block2_antenna_power(::rcraicer_msgs::msg::GPSRFStatus::_block2_antenna_power_type arg)
  {
    msg_.block2_antenna_power = std::move(arg);
    return Init_GPSRFStatus_block2_noise(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block2_antenna_status
{
public:
  explicit Init_GPSRFStatus_block2_antenna_status(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block2_antenna_power block2_antenna_status(::rcraicer_msgs::msg::GPSRFStatus::_block2_antenna_status_type arg)
  {
    msg_.block2_antenna_status = std::move(arg);
    return Init_GPSRFStatus_block2_antenna_power(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block2_jamming
{
public:
  explicit Init_GPSRFStatus_block2_jamming(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block2_antenna_status block2_jamming(::rcraicer_msgs::msg::GPSRFStatus::_block2_jamming_type arg)
  {
    msg_.block2_jamming = std::move(arg);
    return Init_GPSRFStatus_block2_antenna_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block1_jamming_ind
{
public:
  explicit Init_GPSRFStatus_block1_jamming_ind(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block2_jamming block1_jamming_ind(::rcraicer_msgs::msg::GPSRFStatus::_block1_jamming_ind_type arg)
  {
    msg_.block1_jamming_ind = std::move(arg);
    return Init_GPSRFStatus_block2_jamming(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block1_noise
{
public:
  explicit Init_GPSRFStatus_block1_noise(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block1_jamming_ind block1_noise(::rcraicer_msgs::msg::GPSRFStatus::_block1_noise_type arg)
  {
    msg_.block1_noise = std::move(arg);
    return Init_GPSRFStatus_block1_jamming_ind(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block1_antenna_power
{
public:
  explicit Init_GPSRFStatus_block1_antenna_power(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block1_noise block1_antenna_power(::rcraicer_msgs::msg::GPSRFStatus::_block1_antenna_power_type arg)
  {
    msg_.block1_antenna_power = std::move(arg);
    return Init_GPSRFStatus_block1_noise(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block1_antenna_status
{
public:
  explicit Init_GPSRFStatus_block1_antenna_status(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block1_antenna_power block1_antenna_status(::rcraicer_msgs::msg::GPSRFStatus::_block1_antenna_status_type arg)
  {
    msg_.block1_antenna_status = std::move(arg);
    return Init_GPSRFStatus_block1_antenna_power(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_block1_jamming
{
public:
  explicit Init_GPSRFStatus_block1_jamming(::rcraicer_msgs::msg::GPSRFStatus & msg)
  : msg_(msg)
  {}
  Init_GPSRFStatus_block1_antenna_status block1_jamming(::rcraicer_msgs::msg::GPSRFStatus::_block1_jamming_type arg)
  {
    msg_.block1_jamming = std::move(arg);
    return Init_GPSRFStatus_block1_antenna_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

class Init_GPSRFStatus_header
{
public:
  Init_GPSRFStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSRFStatus_block1_jamming header(::rcraicer_msgs::msg::GPSRFStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSRFStatus_block1_jamming(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSRFStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::GPSRFStatus>()
{
  return rcraicer_msgs::msg::builder::Init_GPSRFStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__BUILDER_HPP_

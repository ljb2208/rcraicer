// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/ComputeStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/compute_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_ComputeStatus_power_level
{
public:
  explicit Init_ComputeStatus_power_level(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::ComputeStatus power_level(::rcraicer_msgs::msg::ComputeStatus::_power_level_type arg)
  {
    msg_.power_level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_link_quality
{
public:
  explicit Init_ComputeStatus_link_quality(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_power_level link_quality(::rcraicer_msgs::msg::ComputeStatus::_link_quality_type arg)
  {
    msg_.link_quality = std::move(arg);
    return Init_ComputeStatus_power_level(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_bit_rate
{
public:
  explicit Init_ComputeStatus_bit_rate(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_link_quality bit_rate(::rcraicer_msgs::msg::ComputeStatus::_bit_rate_type arg)
  {
    msg_.bit_rate = std::move(arg);
    return Init_ComputeStatus_link_quality(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_ssid
{
public:
  explicit Init_ComputeStatus_ssid(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_bit_rate ssid(::rcraicer_msgs::msg::ComputeStatus::_ssid_type arg)
  {
    msg_.ssid = std::move(arg);
    return Init_ComputeStatus_bit_rate(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_cpu3
{
public:
  explicit Init_ComputeStatus_cpu3(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_ssid cpu3(::rcraicer_msgs::msg::ComputeStatus::_cpu3_type arg)
  {
    msg_.cpu3 = std::move(arg);
    return Init_ComputeStatus_ssid(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_cpu2
{
public:
  explicit Init_ComputeStatus_cpu2(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_cpu3 cpu2(::rcraicer_msgs::msg::ComputeStatus::_cpu2_type arg)
  {
    msg_.cpu2 = std::move(arg);
    return Init_ComputeStatus_cpu3(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_cpu1
{
public:
  explicit Init_ComputeStatus_cpu1(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_cpu2 cpu1(::rcraicer_msgs::msg::ComputeStatus::_cpu1_type arg)
  {
    msg_.cpu1 = std::move(arg);
    return Init_ComputeStatus_cpu2(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_stat_time
{
public:
  explicit Init_ComputeStatus_stat_time(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_cpu1 stat_time(::rcraicer_msgs::msg::ComputeStatus::_stat_time_type arg)
  {
    msg_.stat_time = std::move(arg);
    return Init_ComputeStatus_cpu1(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_mem_pct
{
public:
  explicit Init_ComputeStatus_mem_pct(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_stat_time mem_pct(::rcraicer_msgs::msg::ComputeStatus::_mem_pct_type arg)
  {
    msg_.mem_pct = std::move(arg);
    return Init_ComputeStatus_stat_time(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_mem_free
{
public:
  explicit Init_ComputeStatus_mem_free(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_mem_pct mem_free(::rcraicer_msgs::msg::ComputeStatus::_mem_free_type arg)
  {
    msg_.mem_free = std::move(arg);
    return Init_ComputeStatus_mem_pct(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_mem_total
{
public:
  explicit Init_ComputeStatus_mem_total(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_mem_free mem_total(::rcraicer_msgs::msg::ComputeStatus::_mem_total_type arg)
  {
    msg_.mem_total = std::move(arg);
    return Init_ComputeStatus_mem_free(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_host_name
{
public:
  explicit Init_ComputeStatus_host_name(::rcraicer_msgs::msg::ComputeStatus & msg)
  : msg_(msg)
  {}
  Init_ComputeStatus_mem_total host_name(::rcraicer_msgs::msg::ComputeStatus::_host_name_type arg)
  {
    msg_.host_name = std::move(arg);
    return Init_ComputeStatus_mem_total(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

class Init_ComputeStatus_header
{
public:
  Init_ComputeStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeStatus_host_name header(::rcraicer_msgs::msg::ComputeStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ComputeStatus_host_name(msg_);
  }

private:
  ::rcraicer_msgs::msg::ComputeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::ComputeStatus>()
{
  return rcraicer_msgs::msg::builder::Init_ComputeStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__BUILDER_HPP_

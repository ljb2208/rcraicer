// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/PathIntegralStats.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/path_integral_stats__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_PathIntegralStats_stats
{
public:
  explicit Init_PathIntegralStats_stats(::rcraicer_msgs::msg::PathIntegralStats & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::PathIntegralStats stats(::rcraicer_msgs::msg::PathIntegralStats::_stats_type arg)
  {
    msg_.stats = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStats msg_;
};

class Init_PathIntegralStats_params
{
public:
  explicit Init_PathIntegralStats_params(::rcraicer_msgs::msg::PathIntegralStats & msg)
  : msg_(msg)
  {}
  Init_PathIntegralStats_stats params(::rcraicer_msgs::msg::PathIntegralStats::_params_type arg)
  {
    msg_.params = std::move(arg);
    return Init_PathIntegralStats_stats(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStats msg_;
};

class Init_PathIntegralStats_tag
{
public:
  explicit Init_PathIntegralStats_tag(::rcraicer_msgs::msg::PathIntegralStats & msg)
  : msg_(msg)
  {}
  Init_PathIntegralStats_params tag(::rcraicer_msgs::msg::PathIntegralStats::_tag_type arg)
  {
    msg_.tag = std::move(arg);
    return Init_PathIntegralStats_params(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStats msg_;
};

class Init_PathIntegralStats_header
{
public:
  Init_PathIntegralStats_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathIntegralStats_tag header(::rcraicer_msgs::msg::PathIntegralStats::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PathIntegralStats_tag(msg_);
  }

private:
  ::rcraicer_msgs::msg::PathIntegralStats msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::PathIntegralStats>()
{
  return rcraicer_msgs::msg::builder::Init_PathIntegralStats_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__BUILDER_HPP_

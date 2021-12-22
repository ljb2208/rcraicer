// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/StateEstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/state_estimator_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_StateEstimatorStatus_status
{
public:
  explicit Init_StateEstimatorStatus_status(::rcraicer_msgs::msg::StateEstimatorStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::StateEstimatorStatus status(::rcraicer_msgs::msg::StateEstimatorStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::StateEstimatorStatus msg_;
};

class Init_StateEstimatorStatus_header
{
public:
  Init_StateEstimatorStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateEstimatorStatus_status header(::rcraicer_msgs::msg::StateEstimatorStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StateEstimatorStatus_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::StateEstimatorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::StateEstimatorStatus>()
{
  return rcraicer_msgs::msg::builder::Init_StateEstimatorStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__BUILDER_HPP_

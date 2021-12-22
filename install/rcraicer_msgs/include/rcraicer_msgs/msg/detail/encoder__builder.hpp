// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/Encoder.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ENCODER__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__ENCODER__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/encoder__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_Encoder_right_rear
{
public:
  explicit Init_Encoder_right_rear(::rcraicer_msgs::msg::Encoder & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::Encoder right_rear(::rcraicer_msgs::msg::Encoder::_right_rear_type arg)
  {
    msg_.right_rear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::Encoder msg_;
};

class Init_Encoder_right_front
{
public:
  explicit Init_Encoder_right_front(::rcraicer_msgs::msg::Encoder & msg)
  : msg_(msg)
  {}
  Init_Encoder_right_rear right_front(::rcraicer_msgs::msg::Encoder::_right_front_type arg)
  {
    msg_.right_front = std::move(arg);
    return Init_Encoder_right_rear(msg_);
  }

private:
  ::rcraicer_msgs::msg::Encoder msg_;
};

class Init_Encoder_left_front
{
public:
  explicit Init_Encoder_left_front(::rcraicer_msgs::msg::Encoder & msg)
  : msg_(msg)
  {}
  Init_Encoder_right_front left_front(::rcraicer_msgs::msg::Encoder::_left_front_type arg)
  {
    msg_.left_front = std::move(arg);
    return Init_Encoder_right_front(msg_);
  }

private:
  ::rcraicer_msgs::msg::Encoder msg_;
};

class Init_Encoder_left_rear
{
public:
  explicit Init_Encoder_left_rear(::rcraicer_msgs::msg::Encoder & msg)
  : msg_(msg)
  {}
  Init_Encoder_left_front left_rear(::rcraicer_msgs::msg::Encoder::_left_rear_type arg)
  {
    msg_.left_rear = std::move(arg);
    return Init_Encoder_left_front(msg_);
  }

private:
  ::rcraicer_msgs::msg::Encoder msg_;
};

class Init_Encoder_header
{
public:
  Init_Encoder_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Encoder_left_rear header(::rcraicer_msgs::msg::Encoder::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Encoder_left_rear(msg_);
  }

private:
  ::rcraicer_msgs::msg::Encoder msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::Encoder>()
{
  return rcraicer_msgs::msg::builder::Init_Encoder_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__ENCODER__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/neural_net_layer__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_NeuralNetLayer_bias
{
public:
  explicit Init_NeuralNetLayer_bias(::rcraicer_msgs::msg::NeuralNetLayer & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::NeuralNetLayer bias(::rcraicer_msgs::msg::NeuralNetLayer::_bias_type arg)
  {
    msg_.bias = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetLayer msg_;
};

class Init_NeuralNetLayer_weight
{
public:
  explicit Init_NeuralNetLayer_weight(::rcraicer_msgs::msg::NeuralNetLayer & msg)
  : msg_(msg)
  {}
  Init_NeuralNetLayer_bias weight(::rcraicer_msgs::msg::NeuralNetLayer::_weight_type arg)
  {
    msg_.weight = std::move(arg);
    return Init_NeuralNetLayer_bias(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetLayer msg_;
};

class Init_NeuralNetLayer_name
{
public:
  Init_NeuralNetLayer_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NeuralNetLayer_weight name(::rcraicer_msgs::msg::NeuralNetLayer::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_NeuralNetLayer_weight(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetLayer msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::NeuralNetLayer>()
{
  return rcraicer_msgs::msg::builder::Init_NeuralNetLayer_name();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__BUILDER_HPP_

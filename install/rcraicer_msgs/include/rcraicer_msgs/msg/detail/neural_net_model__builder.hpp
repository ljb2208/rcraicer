// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/neural_net_model__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_NeuralNetModel_structure
{
public:
  explicit Init_NeuralNetModel_structure(::rcraicer_msgs::msg::NeuralNetModel & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::NeuralNetModel structure(::rcraicer_msgs::msg::NeuralNetModel::_structure_type arg)
  {
    msg_.structure = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetModel msg_;
};

class Init_NeuralNetModel_network
{
public:
  explicit Init_NeuralNetModel_network(::rcraicer_msgs::msg::NeuralNetModel & msg)
  : msg_(msg)
  {}
  Init_NeuralNetModel_structure network(::rcraicer_msgs::msg::NeuralNetModel::_network_type arg)
  {
    msg_.network = std::move(arg);
    return Init_NeuralNetModel_structure(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetModel msg_;
};

class Init_NeuralNetModel_num_layers
{
public:
  explicit Init_NeuralNetModel_num_layers(::rcraicer_msgs::msg::NeuralNetModel & msg)
  : msg_(msg)
  {}
  Init_NeuralNetModel_network num_layers(::rcraicer_msgs::msg::NeuralNetModel::_num_layers_type arg)
  {
    msg_.num_layers = std::move(arg);
    return Init_NeuralNetModel_network(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetModel msg_;
};

class Init_NeuralNetModel_header
{
public:
  Init_NeuralNetModel_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NeuralNetModel_num_layers header(::rcraicer_msgs::msg::NeuralNetModel::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NeuralNetModel_num_layers(msg_);
  }

private:
  ::rcraicer_msgs::msg::NeuralNetModel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::NeuralNetModel>()
{
  return rcraicer_msgs::msg::builder::Init_NeuralNetModel_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__BUILDER_HPP_

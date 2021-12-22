// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__TRAITS_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__TRAITS_HPP_

#include "rcraicer_msgs/msg/detail/neural_net_layer__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcraicer_msgs::msg::NeuralNetLayer>()
{
  return "rcraicer_msgs::msg::NeuralNetLayer";
}

template<>
inline const char * name<rcraicer_msgs::msg::NeuralNetLayer>()
{
  return "rcraicer_msgs/msg/NeuralNetLayer";
}

template<>
struct has_fixed_size<rcraicer_msgs::msg::NeuralNetLayer>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcraicer_msgs::msg::NeuralNetLayer>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcraicer_msgs::msg::NeuralNetLayer>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__TRAITS_HPP_

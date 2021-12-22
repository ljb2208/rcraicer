// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcraicer_msgs:msg/PathIntegralParams.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_PARAMS__TRAITS_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_PARAMS__TRAITS_HPP_

#include "rcraicer_msgs/msg/detail/path_integral_params__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcraicer_msgs::msg::PathIntegralParams>()
{
  return "rcraicer_msgs::msg::PathIntegralParams";
}

template<>
inline const char * name<rcraicer_msgs::msg::PathIntegralParams>()
{
  return "rcraicer_msgs/msg/PathIntegralParams";
}

template<>
struct has_fixed_size<rcraicer_msgs::msg::PathIntegralParams>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcraicer_msgs::msg::PathIntegralParams>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcraicer_msgs::msg::PathIntegralParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_PARAMS__TRAITS_HPP_

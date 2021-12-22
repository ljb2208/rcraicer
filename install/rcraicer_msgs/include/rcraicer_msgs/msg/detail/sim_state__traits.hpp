// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcraicer_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__TRAITS_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__TRAITS_HPP_

#include "rcraicer_msgs/msg/detail/sim_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'env_position'
// Member 'gravity'
// Member 'position'
// Member 'linear_velocity'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'angular_acceleration'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcraicer_msgs::msg::SimState>()
{
  return "rcraicer_msgs::msg::SimState";
}

template<>
inline const char * name<rcraicer_msgs::msg::SimState>()
{
  return "rcraicer_msgs/msg/SimState";
}

template<>
struct has_fixed_size<rcraicer_msgs::msg::SimState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rcraicer_msgs::msg::SimState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rcraicer_msgs::msg::SimState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__TRAITS_HPP_
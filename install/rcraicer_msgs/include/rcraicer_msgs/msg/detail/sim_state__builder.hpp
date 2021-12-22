// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/sim_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_SimState_angular_acceleration
{
public:
  explicit Init_SimState_angular_acceleration(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::SimState angular_acceleration(::rcraicer_msgs::msg::SimState::_angular_acceleration_type arg)
  {
    msg_.angular_acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_angular_velocity
{
public:
  explicit Init_SimState_angular_velocity(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_angular_acceleration angular_velocity(::rcraicer_msgs::msg::SimState::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_SimState_angular_acceleration(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_linear_acceleration
{
public:
  explicit Init_SimState_linear_acceleration(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_angular_velocity linear_acceleration(::rcraicer_msgs::msg::SimState::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_SimState_angular_velocity(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_linear_velocity
{
public:
  explicit Init_SimState_linear_velocity(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_linear_acceleration linear_velocity(::rcraicer_msgs::msg::SimState::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_SimState_linear_acceleration(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_orientation
{
public:
  explicit Init_SimState_orientation(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_linear_velocity orientation(::rcraicer_msgs::msg::SimState::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_SimState_linear_velocity(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_position
{
public:
  explicit Init_SimState_position(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_orientation position(::rcraicer_msgs::msg::SimState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_SimState_orientation(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_air_density
{
public:
  explicit Init_SimState_air_density(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_position air_density(::rcraicer_msgs::msg::SimState::_air_density_type arg)
  {
    msg_.air_density = std::move(arg);
    return Init_SimState_position(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_temperature
{
public:
  explicit Init_SimState_temperature(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_air_density temperature(::rcraicer_msgs::msg::SimState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_SimState_air_density(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_air_pressure
{
public:
  explicit Init_SimState_air_pressure(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_temperature air_pressure(::rcraicer_msgs::msg::SimState::_air_pressure_type arg)
  {
    msg_.air_pressure = std::move(arg);
    return Init_SimState_temperature(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_gravity
{
public:
  explicit Init_SimState_gravity(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_air_pressure gravity(::rcraicer_msgs::msg::SimState::_gravity_type arg)
  {
    msg_.gravity = std::move(arg);
    return Init_SimState_air_pressure(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_altitude
{
public:
  explicit Init_SimState_altitude(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_gravity altitude(::rcraicer_msgs::msg::SimState::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_SimState_gravity(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_longitude
{
public:
  explicit Init_SimState_longitude(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_altitude longitude(::rcraicer_msgs::msg::SimState::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_SimState_altitude(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_latitude
{
public:
  explicit Init_SimState_latitude(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_longitude latitude(::rcraicer_msgs::msg::SimState::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_SimState_longitude(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_env_position
{
public:
  explicit Init_SimState_env_position(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_latitude env_position(::rcraicer_msgs::msg::SimState::_env_position_type arg)
  {
    msg_.env_position = std::move(arg);
    return Init_SimState_latitude(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_brake
{
public:
  explicit Init_SimState_brake(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_env_position brake(::rcraicer_msgs::msg::SimState::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_SimState_env_position(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_steering
{
public:
  explicit Init_SimState_steering(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_brake steering(::rcraicer_msgs::msg::SimState::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_SimState_brake(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_throttle
{
public:
  explicit Init_SimState_throttle(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_steering throttle(::rcraicer_msgs::msg::SimState::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_SimState_steering(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_speed
{
public:
  explicit Init_SimState_speed(::rcraicer_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_throttle speed(::rcraicer_msgs::msg::SimState::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_SimState_throttle(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

class Init_SimState_header
{
public:
  Init_SimState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SimState_speed header(::rcraicer_msgs::msg::SimState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SimState_speed(msg_);
  }

private:
  ::rcraicer_msgs::msg::SimState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::SimState>()
{
  return rcraicer_msgs::msg::builder::Init_SimState_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_

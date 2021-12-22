// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'env_position'
// Member 'gravity'
// Member 'position'
// Member 'linear_velocity'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'angular_acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__SimState __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__SimState __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SimState_
{
  using Type = SimState_<ContainerAllocator>;

  explicit SimState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    env_position(_init),
    gravity(_init),
    position(_init),
    orientation(_init),
    linear_velocity(_init),
    linear_acceleration(_init),
    angular_velocity(_init),
    angular_acceleration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->throttle = 0.0;
      this->steering = 0.0;
      this->brake = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->air_pressure = 0.0;
      this->temperature = 0.0;
      this->air_density = 0.0;
    }
  }

  explicit SimState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    env_position(_alloc, _init),
    gravity(_alloc, _init),
    position(_alloc, _init),
    orientation(_alloc, _init),
    linear_velocity(_alloc, _init),
    linear_acceleration(_alloc, _init),
    angular_velocity(_alloc, _init),
    angular_acceleration(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->throttle = 0.0;
      this->steering = 0.0;
      this->brake = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->air_pressure = 0.0;
      this->temperature = 0.0;
      this->air_density = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _speed_type =
    double;
  _speed_type speed;
  using _throttle_type =
    double;
  _throttle_type throttle;
  using _steering_type =
    double;
  _steering_type steering;
  using _brake_type =
    double;
  _brake_type brake;
  using _env_position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _env_position_type env_position;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _gravity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gravity_type gravity;
  using _air_pressure_type =
    double;
  _air_pressure_type air_pressure;
  using _temperature_type =
    double;
  _temperature_type temperature;
  using _air_density_type =
    double;
  _air_density_type air_density;
  using _position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_type position;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _linear_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_velocity_type linear_velocity;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _angular_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_acceleration_type angular_acceleration;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__throttle(
    const double & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__steering(
    const double & _arg)
  {
    this->steering = _arg;
    return *this;
  }
  Type & set__brake(
    const double & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__env_position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->env_position = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__gravity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gravity = _arg;
    return *this;
  }
  Type & set__air_pressure(
    const double & _arg)
  {
    this->air_pressure = _arg;
    return *this;
  }
  Type & set__temperature(
    const double & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__air_density(
    const double & _arg)
  {
    this->air_density = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__angular_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_acceleration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::SimState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::SimState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::SimState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::SimState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__SimState
    std::shared_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__SimState
    std::shared_ptr<rcraicer_msgs::msg::SimState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SimState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->steering != other.steering) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->env_position != other.env_position) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->gravity != other.gravity) {
      return false;
    }
    if (this->air_pressure != other.air_pressure) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->air_density != other.air_density) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->angular_acceleration != other.angular_acceleration) {
      return false;
    }
    return true;
  }
  bool operator!=(const SimState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SimState_

// alias to use template instance with default allocator
using SimState =
  rcraicer_msgs::msg::SimState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_HPP_

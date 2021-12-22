// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'env_position'
// Member 'gravity'
// Member 'position'
// Member 'linear_velocity'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'angular_acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

// Struct defined in msg/SimState in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__SimState
{
  std_msgs__msg__Header header;
  double speed;
  double throttle;
  double steering;
  double brake;
  geometry_msgs__msg__Vector3 env_position;
  double latitude;
  double longitude;
  double altitude;
  geometry_msgs__msg__Vector3 gravity;
  double air_pressure;
  double temperature;
  double air_density;
  geometry_msgs__msg__Vector3 position;
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 linear_velocity;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 angular_acceleration;
} rcraicer_msgs__msg__SimState;

// Struct for a sequence of rcraicer_msgs__msg__SimState.
typedef struct rcraicer_msgs__msg__SimState__Sequence
{
  rcraicer_msgs__msg__SimState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__SimState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_

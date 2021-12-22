// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/ChassisState.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_H_

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

// Struct defined in msg/ChassisState in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__ChassisState
{
  std_msgs__msg__Header header;
  bool armed;
  double throttle;
  double steer;
  double steer_angle;
} rcraicer_msgs__msg__ChassisState;

// Struct for a sequence of rcraicer_msgs__msg__ChassisState.
typedef struct rcraicer_msgs__msg__ChassisState__Sequence
{
  rcraicer_msgs__msg__ChassisState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__ChassisState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__CHASSIS_STATE__STRUCT_H_

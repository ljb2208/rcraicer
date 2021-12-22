// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/WheelSpeed.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__WHEEL_SPEED__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__WHEEL_SPEED__STRUCT_H_

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

// Struct defined in msg/WheelSpeed in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__WheelSpeed
{
  std_msgs__msg__Header header;
  double left_rear;
  double left_front;
  double right_front;
  double right_rear;
} rcraicer_msgs__msg__WheelSpeed;

// Struct for a sequence of rcraicer_msgs__msg__WheelSpeed.
typedef struct rcraicer_msgs__msg__WheelSpeed__Sequence
{
  rcraicer_msgs__msg__WheelSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__WheelSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__WHEEL_SPEED__STRUCT_H_

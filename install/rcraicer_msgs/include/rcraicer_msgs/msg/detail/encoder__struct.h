// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/Encoder.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_H_

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

// Struct defined in msg/Encoder in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__Encoder
{
  std_msgs__msg__Header header;
  int32_t left_rear;
  int32_t left_front;
  int32_t right_front;
  int32_t right_rear;
} rcraicer_msgs__msg__Encoder;

// Struct for a sequence of rcraicer_msgs__msg__Encoder.
typedef struct rcraicer_msgs__msg__Encoder__Sequence
{
  rcraicer_msgs__msg__Encoder * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__Encoder__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__ENCODER__STRUCT_H_

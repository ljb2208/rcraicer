// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/GPSSurvey.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_H_

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

// Struct defined in msg/GPSSurvey in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__GPSSurvey
{
  std_msgs__msg__Header header;
  int32_t mean_x;
  int32_t mean_y;
  int32_t mean_z;
  int32_t observations;
  int32_t duration;
  float accuracy;
  int16_t valid;
  int16_t active;
} rcraicer_msgs__msg__GPSSurvey;

// Struct for a sequence of rcraicer_msgs__msg__GPSSurvey.
typedef struct rcraicer_msgs__msg__GPSSurvey__Sequence
{
  rcraicer_msgs__msg__GPSSurvey * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__GPSSurvey__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_SURVEY__STRUCT_H_

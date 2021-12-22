// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/LapStats.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/LapStats in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__LapStats
{
  int64_t lap_number;
  double lap_time;
  double max_speed;
  double max_slip;
} rcraicer_msgs__msg__LapStats;

// Struct for a sequence of rcraicer_msgs__msg__LapStats.
typedef struct rcraicer_msgs__msg__LapStats__Sequence
{
  rcraicer_msgs__msg__LapStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__LapStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__LAP_STATS__STRUCT_H_

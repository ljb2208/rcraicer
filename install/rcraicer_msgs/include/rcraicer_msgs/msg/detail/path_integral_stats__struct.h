// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/PathIntegralStats.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__STRUCT_H_

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
// Member 'tag'
#include "rosidl_runtime_c/string.h"
// Member 'params'
#include "rcraicer_msgs/msg/detail/path_integral_params__struct.h"
// Member 'stats'
#include "rcraicer_msgs/msg/detail/lap_stats__struct.h"

// Struct defined in msg/PathIntegralStats in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__PathIntegralStats
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String tag;
  rcraicer_msgs__msg__PathIntegralParams params;
  rcraicer_msgs__msg__LapStats stats;
} rcraicer_msgs__msg__PathIntegralStats;

// Struct for a sequence of rcraicer_msgs__msg__PathIntegralStats.
typedef struct rcraicer_msgs__msg__PathIntegralStats__Sequence
{
  rcraicer_msgs__msg__PathIntegralStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__PathIntegralStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATS__STRUCT_H_

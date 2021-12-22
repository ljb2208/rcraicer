// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/ComputeStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_H_

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
// Member 'host_name'
// Member 'mem_total'
// Member 'mem_free'
// Member 'mem_pct'
// Member 'stat_time'
// Member 'cpu1'
// Member 'cpu2'
// Member 'cpu3'
// Member 'ssid'
// Member 'bit_rate'
// Member 'link_quality'
// Member 'power_level'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/ComputeStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__ComputeStatus
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String host_name;
  rosidl_runtime_c__String mem_total;
  rosidl_runtime_c__String mem_free;
  rosidl_runtime_c__String mem_pct;
  rosidl_runtime_c__String stat_time;
  rosidl_runtime_c__String cpu1;
  rosidl_runtime_c__String cpu2;
  rosidl_runtime_c__String cpu3;
  rosidl_runtime_c__String ssid;
  rosidl_runtime_c__String bit_rate;
  rosidl_runtime_c__String link_quality;
  rosidl_runtime_c__String power_level;
} rcraicer_msgs__msg__ComputeStatus;

// Struct for a sequence of rcraicer_msgs__msg__ComputeStatus.
typedef struct rcraicer_msgs__msg__ComputeStatus__Sequence
{
  rcraicer_msgs__msg__ComputeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__ComputeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_H_

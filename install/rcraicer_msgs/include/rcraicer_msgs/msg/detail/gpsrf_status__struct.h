// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/GPSRFStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_H_

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

// Struct defined in msg/GPSRFStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__GPSRFStatus
{
  std_msgs__msg__Header header;
  int16_t block1_jamming;
  int16_t block1_antenna_status;
  int16_t block1_antenna_power;
  int32_t block1_noise;
  int16_t block1_jamming_ind;
  int16_t block2_jamming;
  int16_t block2_antenna_status;
  int16_t block2_antenna_power;
  int32_t block2_noise;
  int16_t block2_jamming_ind;
} rcraicer_msgs__msg__GPSRFStatus;

// Struct for a sequence of rcraicer_msgs__msg__GPSRFStatus.
typedef struct rcraicer_msgs__msg__GPSRFStatus__Sequence
{
  rcraicer_msgs__msg__GPSRFStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__GPSRFStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPSRF_STATUS__STRUCT_H_

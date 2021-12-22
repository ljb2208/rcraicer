// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/ImuFilterOutput.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_INVALID'.
enum
{
  rcraicer_msgs__msg__ImuFilterOutput__STATUS_INVALID = 0
};

/// Constant 'STATUS_VALID'.
enum
{
  rcraicer_msgs__msg__ImuFilterOutput__STATUS_VALID = 1
};

/// Constant 'STATUS_VALID_REFERENCED'.
enum
{
  rcraicer_msgs__msg__ImuFilterOutput__STATUS_VALID_REFERENCED = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'bias'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/ImuFilterOutput in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__ImuFilterOutput
{
  std_msgs__msg__Header header;
  uint16_t quat_status;
  uint16_t bias_status;
  geometry_msgs__msg__Quaternion orientation;
  double orientation_covariance[9];
  geometry_msgs__msg__Vector3 bias;
  double bias_covariance[9];
  uint16_t bias_covariance_status;
  uint16_t orientation_covariance_status;
} rcraicer_msgs__msg__ImuFilterOutput;

// Struct for a sequence of rcraicer_msgs__msg__ImuFilterOutput.
typedef struct rcraicer_msgs__msg__ImuFilterOutput__Sequence
{
  rcraicer_msgs__msg__ImuFilterOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__ImuFilterOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__IMU_FILTER_OUTPUT__STRUCT_H_

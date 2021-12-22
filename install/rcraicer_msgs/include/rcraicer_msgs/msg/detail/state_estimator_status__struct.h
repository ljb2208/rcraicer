// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/StateEstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OK'.
enum
{
  rcraicer_msgs__msg__StateEstimatorStatus__OK = 0
};

/// Constant 'WARN'.
enum
{
  rcraicer_msgs__msg__StateEstimatorStatus__WARN = 1
};

/// Constant 'ERROR'.
enum
{
  rcraicer_msgs__msg__StateEstimatorStatus__ERROR = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/StateEstimatorStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__StateEstimatorStatus
{
  std_msgs__msg__Header header;
  uint8_t status;
} rcraicer_msgs__msg__StateEstimatorStatus;

// Struct for a sequence of rcraicer_msgs__msg__StateEstimatorStatus.
typedef struct rcraicer_msgs__msg__StateEstimatorStatus__Sequence
{
  rcraicer_msgs__msg__StateEstimatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__StateEstimatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_H_

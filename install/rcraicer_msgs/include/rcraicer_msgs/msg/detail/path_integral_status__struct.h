// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/PathIntegralStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__STRUCT_H_

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
// Member 'info'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/PathIntegralStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__PathIntegralStatus
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String info;
  int32_t status;
} rcraicer_msgs__msg__PathIntegralStatus;

// Struct for a sequence of rcraicer_msgs__msg__PathIntegralStatus.
typedef struct rcraicer_msgs__msg__PathIntegralStatus__Sequence
{
  rcraicer_msgs__msg__PathIntegralStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__PathIntegralStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__PATH_INTEGRAL_STATUS__STRUCT_H_

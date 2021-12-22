// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/ChassisCommand.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__STRUCT_H_

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
// Member 'sender'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/ChassisCommand in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__ChassisCommand
{
  std_msgs__msg__Header header;
  double throttle;
  double steer;
  rosidl_runtime_c__String sender;
} rcraicer_msgs__msg__ChassisCommand;

// Struct for a sequence of rcraicer_msgs__msg__ChassisCommand.
typedef struct rcraicer_msgs__msg__ChassisCommand__Sequence
{
  rcraicer_msgs__msg__ChassisCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__ChassisCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__CHASSIS_COMMAND__STRUCT_H_

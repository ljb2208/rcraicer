// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/ArduinoStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_H_

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

// Struct defined in msg/ArduinoStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__ArduinoStatus
{
  std_msgs__msg__Header header;
  uint16_t servo_update_count;
  uint16_t encoder_msg_count;
  uint16_t main_loop_count;
  uint16_t main_loop_max;
  bool armed;
  uint8_t status;
  uint16_t invalid_crc_arduino;
  uint16_t unknown_msg_arduino;
  uint16_t invalid_crc;
  uint16_t unknown_msg;
} rcraicer_msgs__msg__ArduinoStatus;

// Struct for a sequence of rcraicer_msgs__msg__ArduinoStatus.
typedef struct rcraicer_msgs__msg__ArduinoStatus__Sequence
{
  rcraicer_msgs__msg__ArduinoStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__ArduinoStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__ARDUINO_STATUS__STRUCT_H_

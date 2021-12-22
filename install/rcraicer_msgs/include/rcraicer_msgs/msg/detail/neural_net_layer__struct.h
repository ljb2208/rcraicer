// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'weight'
// Member 'bias'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/NeuralNetLayer in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__NeuralNetLayer
{
  rosidl_runtime_c__String name;
  rosidl_runtime_c__float__Sequence weight;
  rosidl_runtime_c__float__Sequence bias;
} rcraicer_msgs__msg__NeuralNetLayer;

// Struct for a sequence of rcraicer_msgs__msg__NeuralNetLayer.
typedef struct rcraicer_msgs__msg__NeuralNetLayer__Sequence
{
  rcraicer_msgs__msg__NeuralNetLayer * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__NeuralNetLayer__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_LAYER__STRUCT_H_

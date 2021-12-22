// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_H_

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
// Member 'network'
#include "rcraicer_msgs/msg/detail/neural_net_layer__struct.h"
// Member 'structure'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/NeuralNetModel in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__NeuralNetModel
{
  std_msgs__msg__Header header;
  int32_t num_layers;
  rcraicer_msgs__msg__NeuralNetLayer__Sequence network;
  rosidl_runtime_c__int32__Sequence structure;
} rcraicer_msgs__msg__NeuralNetModel;

// Struct for a sequence of rcraicer_msgs__msg__NeuralNetModel.
typedef struct rcraicer_msgs__msg__NeuralNetModel__Sequence
{
  rcraicer_msgs__msg__NeuralNetModel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__NeuralNetModel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__NEURAL_NET_MODEL__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/neural_net_layer__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `weight`
// Member `bias`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rcraicer_msgs__msg__NeuralNetLayer__init(rcraicer_msgs__msg__NeuralNetLayer * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rcraicer_msgs__msg__NeuralNetLayer__fini(msg);
    return false;
  }
  // weight
  if (!rosidl_runtime_c__float__Sequence__init(&msg->weight, 0)) {
    rcraicer_msgs__msg__NeuralNetLayer__fini(msg);
    return false;
  }
  // bias
  if (!rosidl_runtime_c__float__Sequence__init(&msg->bias, 0)) {
    rcraicer_msgs__msg__NeuralNetLayer__fini(msg);
    return false;
  }
  return true;
}

void
rcraicer_msgs__msg__NeuralNetLayer__fini(rcraicer_msgs__msg__NeuralNetLayer * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // weight
  rosidl_runtime_c__float__Sequence__fini(&msg->weight);
  // bias
  rosidl_runtime_c__float__Sequence__fini(&msg->bias);
}

rcraicer_msgs__msg__NeuralNetLayer *
rcraicer_msgs__msg__NeuralNetLayer__create()
{
  rcraicer_msgs__msg__NeuralNetLayer * msg = (rcraicer_msgs__msg__NeuralNetLayer *)malloc(sizeof(rcraicer_msgs__msg__NeuralNetLayer));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__NeuralNetLayer));
  bool success = rcraicer_msgs__msg__NeuralNetLayer__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__NeuralNetLayer__destroy(rcraicer_msgs__msg__NeuralNetLayer * msg)
{
  if (msg) {
    rcraicer_msgs__msg__NeuralNetLayer__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__NeuralNetLayer__Sequence__init(rcraicer_msgs__msg__NeuralNetLayer__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__NeuralNetLayer * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__NeuralNetLayer *)calloc(size, sizeof(rcraicer_msgs__msg__NeuralNetLayer));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__NeuralNetLayer__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__NeuralNetLayer__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcraicer_msgs__msg__NeuralNetLayer__Sequence__fini(rcraicer_msgs__msg__NeuralNetLayer__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__NeuralNetLayer__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcraicer_msgs__msg__NeuralNetLayer__Sequence *
rcraicer_msgs__msg__NeuralNetLayer__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__NeuralNetLayer__Sequence * array = (rcraicer_msgs__msg__NeuralNetLayer__Sequence *)malloc(sizeof(rcraicer_msgs__msg__NeuralNetLayer__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__NeuralNetLayer__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__NeuralNetLayer__Sequence__destroy(rcraicer_msgs__msg__NeuralNetLayer__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__NeuralNetLayer__Sequence__fini(array);
  }
  free(array);
}

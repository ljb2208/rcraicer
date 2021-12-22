// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/PathIntegralParams.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/path_integral_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `map_path`
#include "rosidl_runtime_c/string_functions.h"

bool
rcraicer_msgs__msg__PathIntegralParams__init(rcraicer_msgs__msg__PathIntegralParams * msg)
{
  if (!msg) {
    return false;
  }
  // hz
  // num_timesteps
  // num_iters
  // gamma
  // init_steering
  // init_throttle
  // steering_var
  // throttle_var
  // max_throttle
  // speed_coefficient
  // track_coefficient
  // max_slip_angle
  // track_slop
  // crash_coeff
  // map_path
  if (!rosidl_runtime_c__String__init(&msg->map_path)) {
    rcraicer_msgs__msg__PathIntegralParams__fini(msg);
    return false;
  }
  // desired_speed
  return true;
}

void
rcraicer_msgs__msg__PathIntegralParams__fini(rcraicer_msgs__msg__PathIntegralParams * msg)
{
  if (!msg) {
    return;
  }
  // hz
  // num_timesteps
  // num_iters
  // gamma
  // init_steering
  // init_throttle
  // steering_var
  // throttle_var
  // max_throttle
  // speed_coefficient
  // track_coefficient
  // max_slip_angle
  // track_slop
  // crash_coeff
  // map_path
  rosidl_runtime_c__String__fini(&msg->map_path);
  // desired_speed
}

rcraicer_msgs__msg__PathIntegralParams *
rcraicer_msgs__msg__PathIntegralParams__create()
{
  rcraicer_msgs__msg__PathIntegralParams * msg = (rcraicer_msgs__msg__PathIntegralParams *)malloc(sizeof(rcraicer_msgs__msg__PathIntegralParams));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__PathIntegralParams));
  bool success = rcraicer_msgs__msg__PathIntegralParams__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__PathIntegralParams__destroy(rcraicer_msgs__msg__PathIntegralParams * msg)
{
  if (msg) {
    rcraicer_msgs__msg__PathIntegralParams__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__PathIntegralParams__Sequence__init(rcraicer_msgs__msg__PathIntegralParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__PathIntegralParams * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__PathIntegralParams *)calloc(size, sizeof(rcraicer_msgs__msg__PathIntegralParams));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__PathIntegralParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__PathIntegralParams__fini(&data[i - 1]);
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
rcraicer_msgs__msg__PathIntegralParams__Sequence__fini(rcraicer_msgs__msg__PathIntegralParams__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__PathIntegralParams__fini(&array->data[i]);
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

rcraicer_msgs__msg__PathIntegralParams__Sequence *
rcraicer_msgs__msg__PathIntegralParams__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__PathIntegralParams__Sequence * array = (rcraicer_msgs__msg__PathIntegralParams__Sequence *)malloc(sizeof(rcraicer_msgs__msg__PathIntegralParams__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__PathIntegralParams__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__PathIntegralParams__Sequence__destroy(rcraicer_msgs__msg__PathIntegralParams__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__PathIntegralParams__Sequence__fini(array);
  }
  free(array);
}

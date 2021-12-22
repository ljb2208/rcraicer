// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/PathIntegralStats.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/path_integral_stats__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `tag`
#include "rosidl_runtime_c/string_functions.h"
// Member `params`
#include "rcraicer_msgs/msg/detail/path_integral_params__functions.h"
// Member `stats`
#include "rcraicer_msgs/msg/detail/lap_stats__functions.h"

bool
rcraicer_msgs__msg__PathIntegralStats__init(rcraicer_msgs__msg__PathIntegralStats * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rcraicer_msgs__msg__PathIntegralStats__fini(msg);
    return false;
  }
  // tag
  if (!rosidl_runtime_c__String__init(&msg->tag)) {
    rcraicer_msgs__msg__PathIntegralStats__fini(msg);
    return false;
  }
  // params
  if (!rcraicer_msgs__msg__PathIntegralParams__init(&msg->params)) {
    rcraicer_msgs__msg__PathIntegralStats__fini(msg);
    return false;
  }
  // stats
  if (!rcraicer_msgs__msg__LapStats__init(&msg->stats)) {
    rcraicer_msgs__msg__PathIntegralStats__fini(msg);
    return false;
  }
  return true;
}

void
rcraicer_msgs__msg__PathIntegralStats__fini(rcraicer_msgs__msg__PathIntegralStats * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // tag
  rosidl_runtime_c__String__fini(&msg->tag);
  // params
  rcraicer_msgs__msg__PathIntegralParams__fini(&msg->params);
  // stats
  rcraicer_msgs__msg__LapStats__fini(&msg->stats);
}

rcraicer_msgs__msg__PathIntegralStats *
rcraicer_msgs__msg__PathIntegralStats__create()
{
  rcraicer_msgs__msg__PathIntegralStats * msg = (rcraicer_msgs__msg__PathIntegralStats *)malloc(sizeof(rcraicer_msgs__msg__PathIntegralStats));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__PathIntegralStats));
  bool success = rcraicer_msgs__msg__PathIntegralStats__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__PathIntegralStats__destroy(rcraicer_msgs__msg__PathIntegralStats * msg)
{
  if (msg) {
    rcraicer_msgs__msg__PathIntegralStats__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__PathIntegralStats__Sequence__init(rcraicer_msgs__msg__PathIntegralStats__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__PathIntegralStats * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__PathIntegralStats *)calloc(size, sizeof(rcraicer_msgs__msg__PathIntegralStats));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__PathIntegralStats__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__PathIntegralStats__fini(&data[i - 1]);
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
rcraicer_msgs__msg__PathIntegralStats__Sequence__fini(rcraicer_msgs__msg__PathIntegralStats__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__PathIntegralStats__fini(&array->data[i]);
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

rcraicer_msgs__msg__PathIntegralStats__Sequence *
rcraicer_msgs__msg__PathIntegralStats__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__PathIntegralStats__Sequence * array = (rcraicer_msgs__msg__PathIntegralStats__Sequence *)malloc(sizeof(rcraicer_msgs__msg__PathIntegralStats__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__PathIntegralStats__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__PathIntegralStats__Sequence__destroy(rcraicer_msgs__msg__PathIntegralStats__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__PathIntegralStats__Sequence__fini(array);
  }
  free(array);
}

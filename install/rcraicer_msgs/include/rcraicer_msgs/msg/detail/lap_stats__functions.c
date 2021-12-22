// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/LapStats.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/lap_stats__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
rcraicer_msgs__msg__LapStats__init(rcraicer_msgs__msg__LapStats * msg)
{
  if (!msg) {
    return false;
  }
  // lap_number
  // lap_time
  // max_speed
  // max_slip
  return true;
}

void
rcraicer_msgs__msg__LapStats__fini(rcraicer_msgs__msg__LapStats * msg)
{
  if (!msg) {
    return;
  }
  // lap_number
  // lap_time
  // max_speed
  // max_slip
}

rcraicer_msgs__msg__LapStats *
rcraicer_msgs__msg__LapStats__create()
{
  rcraicer_msgs__msg__LapStats * msg = (rcraicer_msgs__msg__LapStats *)malloc(sizeof(rcraicer_msgs__msg__LapStats));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__LapStats));
  bool success = rcraicer_msgs__msg__LapStats__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__LapStats__destroy(rcraicer_msgs__msg__LapStats * msg)
{
  if (msg) {
    rcraicer_msgs__msg__LapStats__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__LapStats__Sequence__init(rcraicer_msgs__msg__LapStats__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__LapStats * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__LapStats *)calloc(size, sizeof(rcraicer_msgs__msg__LapStats));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__LapStats__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__LapStats__fini(&data[i - 1]);
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
rcraicer_msgs__msg__LapStats__Sequence__fini(rcraicer_msgs__msg__LapStats__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__LapStats__fini(&array->data[i]);
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

rcraicer_msgs__msg__LapStats__Sequence *
rcraicer_msgs__msg__LapStats__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__LapStats__Sequence * array = (rcraicer_msgs__msg__LapStats__Sequence *)malloc(sizeof(rcraicer_msgs__msg__LapStats__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__LapStats__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__LapStats__Sequence__destroy(rcraicer_msgs__msg__LapStats__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__LapStats__Sequence__fini(array);
  }
  free(array);
}
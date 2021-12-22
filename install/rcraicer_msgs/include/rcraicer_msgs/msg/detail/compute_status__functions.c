// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/ComputeStatus.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/compute_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `host_name`
// Member `mem_total`
// Member `mem_free`
// Member `mem_pct`
// Member `stat_time`
// Member `cpu1`
// Member `cpu2`
// Member `cpu3`
// Member `ssid`
// Member `bit_rate`
// Member `link_quality`
// Member `power_level`
#include "rosidl_runtime_c/string_functions.h"

bool
rcraicer_msgs__msg__ComputeStatus__init(rcraicer_msgs__msg__ComputeStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // host_name
  if (!rosidl_runtime_c__String__init(&msg->host_name)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // mem_total
  if (!rosidl_runtime_c__String__init(&msg->mem_total)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // mem_free
  if (!rosidl_runtime_c__String__init(&msg->mem_free)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // mem_pct
  if (!rosidl_runtime_c__String__init(&msg->mem_pct)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // stat_time
  if (!rosidl_runtime_c__String__init(&msg->stat_time)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // cpu1
  if (!rosidl_runtime_c__String__init(&msg->cpu1)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // cpu2
  if (!rosidl_runtime_c__String__init(&msg->cpu2)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // cpu3
  if (!rosidl_runtime_c__String__init(&msg->cpu3)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // ssid
  if (!rosidl_runtime_c__String__init(&msg->ssid)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // bit_rate
  if (!rosidl_runtime_c__String__init(&msg->bit_rate)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // link_quality
  if (!rosidl_runtime_c__String__init(&msg->link_quality)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  // power_level
  if (!rosidl_runtime_c__String__init(&msg->power_level)) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
    return false;
  }
  return true;
}

void
rcraicer_msgs__msg__ComputeStatus__fini(rcraicer_msgs__msg__ComputeStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // host_name
  rosidl_runtime_c__String__fini(&msg->host_name);
  // mem_total
  rosidl_runtime_c__String__fini(&msg->mem_total);
  // mem_free
  rosidl_runtime_c__String__fini(&msg->mem_free);
  // mem_pct
  rosidl_runtime_c__String__fini(&msg->mem_pct);
  // stat_time
  rosidl_runtime_c__String__fini(&msg->stat_time);
  // cpu1
  rosidl_runtime_c__String__fini(&msg->cpu1);
  // cpu2
  rosidl_runtime_c__String__fini(&msg->cpu2);
  // cpu3
  rosidl_runtime_c__String__fini(&msg->cpu3);
  // ssid
  rosidl_runtime_c__String__fini(&msg->ssid);
  // bit_rate
  rosidl_runtime_c__String__fini(&msg->bit_rate);
  // link_quality
  rosidl_runtime_c__String__fini(&msg->link_quality);
  // power_level
  rosidl_runtime_c__String__fini(&msg->power_level);
}

rcraicer_msgs__msg__ComputeStatus *
rcraicer_msgs__msg__ComputeStatus__create()
{
  rcraicer_msgs__msg__ComputeStatus * msg = (rcraicer_msgs__msg__ComputeStatus *)malloc(sizeof(rcraicer_msgs__msg__ComputeStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__ComputeStatus));
  bool success = rcraicer_msgs__msg__ComputeStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__ComputeStatus__destroy(rcraicer_msgs__msg__ComputeStatus * msg)
{
  if (msg) {
    rcraicer_msgs__msg__ComputeStatus__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__ComputeStatus__Sequence__init(rcraicer_msgs__msg__ComputeStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__ComputeStatus * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__ComputeStatus *)calloc(size, sizeof(rcraicer_msgs__msg__ComputeStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__ComputeStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__ComputeStatus__fini(&data[i - 1]);
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
rcraicer_msgs__msg__ComputeStatus__Sequence__fini(rcraicer_msgs__msg__ComputeStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__ComputeStatus__fini(&array->data[i]);
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

rcraicer_msgs__msg__ComputeStatus__Sequence *
rcraicer_msgs__msg__ComputeStatus__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__ComputeStatus__Sequence * array = (rcraicer_msgs__msg__ComputeStatus__Sequence *)malloc(sizeof(rcraicer_msgs__msg__ComputeStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__ComputeStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__ComputeStatus__Sequence__destroy(rcraicer_msgs__msg__ComputeStatus__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__ComputeStatus__Sequence__fini(array);
  }
  free(array);
}

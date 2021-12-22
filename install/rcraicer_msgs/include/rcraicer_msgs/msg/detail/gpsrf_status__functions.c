// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/GPSRFStatus.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/gpsrf_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rcraicer_msgs__msg__GPSRFStatus__init(rcraicer_msgs__msg__GPSRFStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rcraicer_msgs__msg__GPSRFStatus__fini(msg);
    return false;
  }
  // block1_jamming
  // block1_antenna_status
  // block1_antenna_power
  // block1_noise
  // block1_jamming_ind
  // block2_jamming
  // block2_antenna_status
  // block2_antenna_power
  // block2_noise
  // block2_jamming_ind
  return true;
}

void
rcraicer_msgs__msg__GPSRFStatus__fini(rcraicer_msgs__msg__GPSRFStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // block1_jamming
  // block1_antenna_status
  // block1_antenna_power
  // block1_noise
  // block1_jamming_ind
  // block2_jamming
  // block2_antenna_status
  // block2_antenna_power
  // block2_noise
  // block2_jamming_ind
}

rcraicer_msgs__msg__GPSRFStatus *
rcraicer_msgs__msg__GPSRFStatus__create()
{
  rcraicer_msgs__msg__GPSRFStatus * msg = (rcraicer_msgs__msg__GPSRFStatus *)malloc(sizeof(rcraicer_msgs__msg__GPSRFStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__GPSRFStatus));
  bool success = rcraicer_msgs__msg__GPSRFStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__GPSRFStatus__destroy(rcraicer_msgs__msg__GPSRFStatus * msg)
{
  if (msg) {
    rcraicer_msgs__msg__GPSRFStatus__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__GPSRFStatus__Sequence__init(rcraicer_msgs__msg__GPSRFStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__GPSRFStatus * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__GPSRFStatus *)calloc(size, sizeof(rcraicer_msgs__msg__GPSRFStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__GPSRFStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__GPSRFStatus__fini(&data[i - 1]);
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
rcraicer_msgs__msg__GPSRFStatus__Sequence__fini(rcraicer_msgs__msg__GPSRFStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__GPSRFStatus__fini(&array->data[i]);
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

rcraicer_msgs__msg__GPSRFStatus__Sequence *
rcraicer_msgs__msg__GPSRFStatus__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__GPSRFStatus__Sequence * array = (rcraicer_msgs__msg__GPSRFStatus__Sequence *)malloc(sizeof(rcraicer_msgs__msg__GPSRFStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__GPSRFStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__GPSRFStatus__Sequence__destroy(rcraicer_msgs__msg__GPSRFStatus__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__GPSRFStatus__Sequence__fini(array);
  }
  free(array);
}

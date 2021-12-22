// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/GPSStatus.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/gps_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `satellite_used_prn`
// Member `satellite_visible_prn`
// Member `satellite_visible_z`
// Member `satellite_visible_azimuth`
// Member `satellite_visible_snr`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rcraicer_msgs__msg__GPSStatus__init(rcraicer_msgs__msg__GPSStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // satellites_used
  // satellite_used_prn
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->satellite_used_prn, 0)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // satellites_visible
  // satellite_visible_prn
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->satellite_visible_prn, 0)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // satellite_visible_z
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->satellite_visible_z, 0)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // satellite_visible_azimuth
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->satellite_visible_azimuth, 0)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // satellite_visible_snr
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->satellite_visible_snr, 0)) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
    return false;
  }
  // status
  // rtk_status
  // vdop
  // hdop
  // pdop
  // hacc
  // vacc
  // gspeed
  // sacc
  // headmot
  // headacc
  return true;
}

void
rcraicer_msgs__msg__GPSStatus__fini(rcraicer_msgs__msg__GPSStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // satellites_used
  // satellite_used_prn
  rosidl_runtime_c__int32__Sequence__fini(&msg->satellite_used_prn);
  // satellites_visible
  // satellite_visible_prn
  rosidl_runtime_c__int32__Sequence__fini(&msg->satellite_visible_prn);
  // satellite_visible_z
  rosidl_runtime_c__int32__Sequence__fini(&msg->satellite_visible_z);
  // satellite_visible_azimuth
  rosidl_runtime_c__int32__Sequence__fini(&msg->satellite_visible_azimuth);
  // satellite_visible_snr
  rosidl_runtime_c__int32__Sequence__fini(&msg->satellite_visible_snr);
  // status
  // rtk_status
  // vdop
  // hdop
  // pdop
  // hacc
  // vacc
  // gspeed
  // sacc
  // headmot
  // headacc
}

rcraicer_msgs__msg__GPSStatus *
rcraicer_msgs__msg__GPSStatus__create()
{
  rcraicer_msgs__msg__GPSStatus * msg = (rcraicer_msgs__msg__GPSStatus *)malloc(sizeof(rcraicer_msgs__msg__GPSStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__GPSStatus));
  bool success = rcraicer_msgs__msg__GPSStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__GPSStatus__destroy(rcraicer_msgs__msg__GPSStatus * msg)
{
  if (msg) {
    rcraicer_msgs__msg__GPSStatus__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__GPSStatus__Sequence__init(rcraicer_msgs__msg__GPSStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__GPSStatus * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__GPSStatus *)calloc(size, sizeof(rcraicer_msgs__msg__GPSStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__GPSStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__GPSStatus__fini(&data[i - 1]);
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
rcraicer_msgs__msg__GPSStatus__Sequence__fini(rcraicer_msgs__msg__GPSStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__GPSStatus__fini(&array->data[i]);
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

rcraicer_msgs__msg__GPSStatus__Sequence *
rcraicer_msgs__msg__GPSStatus__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__GPSStatus__Sequence * array = (rcraicer_msgs__msg__GPSStatus__Sequence *)malloc(sizeof(rcraicer_msgs__msg__GPSStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__GPSStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__GPSStatus__Sequence__destroy(rcraicer_msgs__msg__GPSStatus__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__GPSStatus__Sequence__fini(array);
  }
  free(array);
}

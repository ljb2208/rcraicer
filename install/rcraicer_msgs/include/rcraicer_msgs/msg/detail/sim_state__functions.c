// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcraicer_msgs:msg/SimState.idl
// generated code does not contain a copyright notice
#include "rcraicer_msgs/msg/detail/sim_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `env_position`
// Member `gravity`
// Member `position`
// Member `linear_velocity`
// Member `linear_acceleration`
// Member `angular_velocity`
// Member `angular_acceleration`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
rcraicer_msgs__msg__SimState__init(rcraicer_msgs__msg__SimState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // speed
  // throttle
  // steering
  // brake
  // env_position
  if (!geometry_msgs__msg__Vector3__init(&msg->env_position)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // latitude
  // longitude
  // altitude
  // gravity
  if (!geometry_msgs__msg__Vector3__init(&msg->gravity)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // air_pressure
  // temperature
  // air_density
  // position
  if (!geometry_msgs__msg__Vector3__init(&msg->position)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // linear_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_velocity)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  // angular_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_acceleration)) {
    rcraicer_msgs__msg__SimState__fini(msg);
    return false;
  }
  return true;
}

void
rcraicer_msgs__msg__SimState__fini(rcraicer_msgs__msg__SimState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // speed
  // throttle
  // steering
  // brake
  // env_position
  geometry_msgs__msg__Vector3__fini(&msg->env_position);
  // latitude
  // longitude
  // altitude
  // gravity
  geometry_msgs__msg__Vector3__fini(&msg->gravity);
  // air_pressure
  // temperature
  // air_density
  // position
  geometry_msgs__msg__Vector3__fini(&msg->position);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // linear_velocity
  geometry_msgs__msg__Vector3__fini(&msg->linear_velocity);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // angular_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->angular_acceleration);
}

rcraicer_msgs__msg__SimState *
rcraicer_msgs__msg__SimState__create()
{
  rcraicer_msgs__msg__SimState * msg = (rcraicer_msgs__msg__SimState *)malloc(sizeof(rcraicer_msgs__msg__SimState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcraicer_msgs__msg__SimState));
  bool success = rcraicer_msgs__msg__SimState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcraicer_msgs__msg__SimState__destroy(rcraicer_msgs__msg__SimState * msg)
{
  if (msg) {
    rcraicer_msgs__msg__SimState__fini(msg);
  }
  free(msg);
}


bool
rcraicer_msgs__msg__SimState__Sequence__init(rcraicer_msgs__msg__SimState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcraicer_msgs__msg__SimState * data = NULL;
  if (size) {
    data = (rcraicer_msgs__msg__SimState *)calloc(size, sizeof(rcraicer_msgs__msg__SimState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcraicer_msgs__msg__SimState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcraicer_msgs__msg__SimState__fini(&data[i - 1]);
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
rcraicer_msgs__msg__SimState__Sequence__fini(rcraicer_msgs__msg__SimState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcraicer_msgs__msg__SimState__fini(&array->data[i]);
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

rcraicer_msgs__msg__SimState__Sequence *
rcraicer_msgs__msg__SimState__Sequence__create(size_t size)
{
  rcraicer_msgs__msg__SimState__Sequence * array = (rcraicer_msgs__msg__SimState__Sequence *)malloc(sizeof(rcraicer_msgs__msg__SimState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcraicer_msgs__msg__SimState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcraicer_msgs__msg__SimState__Sequence__destroy(rcraicer_msgs__msg__SimState__Sequence * array)
{
  if (array) {
    rcraicer_msgs__msg__SimState__Sequence__fini(array);
  }
  free(array);
}

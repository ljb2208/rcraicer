// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcraicer_msgs:msg/GPSStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_H_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_NO_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_NO_FIX = -1
};

/// Constant 'STATUS_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_FIX = 0
};

/// Constant 'STATUS_SBAS_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_SBAS_FIX = 1
};

/// Constant 'STATUS_GBAS_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_GBAS_FIX = 2
};

/// Constant 'STATUS_DGPS_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_DGPS_FIX = 18
};

/// Constant 'STATUS_WAAS_FIX'.
enum
{
  rcraicer_msgs__msg__GPSStatus__STATUS_WAAS_FIX = 33
};

/// Constant 'RTK_STATUS_NONE'.
enum
{
  rcraicer_msgs__msg__GPSStatus__RTK_STATUS_NONE = 0
};

/// Constant 'RTK_STATUS_FLOAT'.
enum
{
  rcraicer_msgs__msg__GPSStatus__RTK_STATUS_FLOAT = 1
};

/// Constant 'RTK_STATUS_FIXED'.
enum
{
  rcraicer_msgs__msg__GPSStatus__RTK_STATUS_FIXED = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'satellite_used_prn'
// Member 'satellite_visible_prn'
// Member 'satellite_visible_z'
// Member 'satellite_visible_azimuth'
// Member 'satellite_visible_snr'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/GPSStatus in the package rcraicer_msgs.
typedef struct rcraicer_msgs__msg__GPSStatus
{
  std_msgs__msg__Header header;
  uint16_t satellites_used;
  rosidl_runtime_c__int32__Sequence satellite_used_prn;
  uint16_t satellites_visible;
  rosidl_runtime_c__int32__Sequence satellite_visible_prn;
  rosidl_runtime_c__int32__Sequence satellite_visible_z;
  rosidl_runtime_c__int32__Sequence satellite_visible_azimuth;
  rosidl_runtime_c__int32__Sequence satellite_visible_snr;
  int16_t status;
  int16_t rtk_status;
  float vdop;
  float hdop;
  float pdop;
  float hacc;
  float vacc;
  float gspeed;
  float sacc;
  float headmot;
  float headacc;
} rcraicer_msgs__msg__GPSStatus;

// Struct for a sequence of rcraicer_msgs__msg__GPSStatus.
typedef struct rcraicer_msgs__msg__GPSStatus__Sequence
{
  rcraicer_msgs__msg__GPSStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcraicer_msgs__msg__GPSStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_H_

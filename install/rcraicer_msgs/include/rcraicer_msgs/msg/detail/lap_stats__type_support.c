// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcraicer_msgs:msg/LapStats.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcraicer_msgs/msg/detail/lap_stats__rosidl_typesupport_introspection_c.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcraicer_msgs/msg/detail/lap_stats__functions.h"
#include "rcraicer_msgs/msg/detail/lap_stats__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void LapStats__rosidl_typesupport_introspection_c__LapStats_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcraicer_msgs__msg__LapStats__init(message_memory);
}

void LapStats__rosidl_typesupport_introspection_c__LapStats_fini_function(void * message_memory)
{
  rcraicer_msgs__msg__LapStats__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LapStats__rosidl_typesupport_introspection_c__LapStats_message_member_array[4] = {
  {
    "lap_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__LapStats, lap_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lap_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__LapStats, lap_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__LapStats, max_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_slip",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__LapStats, max_slip),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LapStats__rosidl_typesupport_introspection_c__LapStats_message_members = {
  "rcraicer_msgs__msg",  // message namespace
  "LapStats",  // message name
  4,  // number of fields
  sizeof(rcraicer_msgs__msg__LapStats),
  LapStats__rosidl_typesupport_introspection_c__LapStats_message_member_array,  // message members
  LapStats__rosidl_typesupport_introspection_c__LapStats_init_function,  // function to initialize message memory (memory has to be allocated)
  LapStats__rosidl_typesupport_introspection_c__LapStats_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LapStats__rosidl_typesupport_introspection_c__LapStats_message_type_support_handle = {
  0,
  &LapStats__rosidl_typesupport_introspection_c__LapStats_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcraicer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, LapStats)() {
  if (!LapStats__rosidl_typesupport_introspection_c__LapStats_message_type_support_handle.typesupport_identifier) {
    LapStats__rosidl_typesupport_introspection_c__LapStats_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LapStats__rosidl_typesupport_introspection_c__LapStats_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

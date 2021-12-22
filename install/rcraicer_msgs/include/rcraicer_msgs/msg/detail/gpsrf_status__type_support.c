// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcraicer_msgs:msg/GPSRFStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcraicer_msgs/msg/detail/gpsrf_status__rosidl_typesupport_introspection_c.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcraicer_msgs/msg/detail/gpsrf_status__functions.h"
#include "rcraicer_msgs/msg/detail/gpsrf_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcraicer_msgs__msg__GPSRFStatus__init(message_memory);
}

void GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_fini_function(void * message_memory)
{
  rcraicer_msgs__msg__GPSRFStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_member_array[11] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block1_jamming",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block1_jamming),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block1_antenna_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block1_antenna_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block1_antenna_power",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block1_antenna_power),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block1_noise",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block1_noise),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block1_jamming_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block1_jamming_ind),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block2_jamming",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block2_jamming),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block2_antenna_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block2_antenna_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block2_antenna_power",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block2_antenna_power),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block2_noise",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block2_noise),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "block2_jamming_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__GPSRFStatus, block2_jamming_ind),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_members = {
  "rcraicer_msgs__msg",  // message namespace
  "GPSRFStatus",  // message name
  11,  // number of fields
  sizeof(rcraicer_msgs__msg__GPSRFStatus),
  GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_member_array,  // message members
  GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_type_support_handle = {
  0,
  &GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcraicer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, GPSRFStatus)() {
  GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_type_support_handle.typesupport_identifier) {
    GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GPSRFStatus__rosidl_typesupport_introspection_c__GPSRFStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

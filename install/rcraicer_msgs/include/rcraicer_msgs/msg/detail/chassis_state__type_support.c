// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcraicer_msgs:msg/ChassisState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcraicer_msgs/msg/detail/chassis_state__rosidl_typesupport_introspection_c.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcraicer_msgs/msg/detail/chassis_state__functions.h"
#include "rcraicer_msgs/msg/detail/chassis_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ChassisState__rosidl_typesupport_introspection_c__ChassisState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcraicer_msgs__msg__ChassisState__init(message_memory);
}

void ChassisState__rosidl_typesupport_introspection_c__ChassisState_fini_function(void * message_memory)
{
  rcraicer_msgs__msg__ChassisState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__ChassisState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "armed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__ChassisState, armed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "throttle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__ChassisState, throttle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__ChassisState, steer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steer_angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__ChassisState, steer_angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_members = {
  "rcraicer_msgs__msg",  // message namespace
  "ChassisState",  // message name
  5,  // number of fields
  sizeof(rcraicer_msgs__msg__ChassisState),
  ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_member_array,  // message members
  ChassisState__rosidl_typesupport_introspection_c__ChassisState_init_function,  // function to initialize message memory (memory has to be allocated)
  ChassisState__rosidl_typesupport_introspection_c__ChassisState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_type_support_handle = {
  0,
  &ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcraicer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, ChassisState)() {
  ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_type_support_handle.typesupport_identifier) {
    ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ChassisState__rosidl_typesupport_introspection_c__ChassisState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

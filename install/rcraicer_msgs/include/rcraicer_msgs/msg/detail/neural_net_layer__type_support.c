// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcraicer_msgs:msg/NeuralNetLayer.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcraicer_msgs/msg/detail/neural_net_layer__rosidl_typesupport_introspection_c.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcraicer_msgs/msg/detail/neural_net_layer__functions.h"
#include "rcraicer_msgs/msg/detail/neural_net_layer__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `weight`
// Member `bias`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcraicer_msgs__msg__NeuralNetLayer__init(message_memory);
}

void NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_fini_function(void * message_memory)
{
  rcraicer_msgs__msg__NeuralNetLayer__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_member_array[3] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetLayer, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "weight",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetLayer, weight),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bias",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetLayer, bias),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_members = {
  "rcraicer_msgs__msg",  // message namespace
  "NeuralNetLayer",  // message name
  3,  // number of fields
  sizeof(rcraicer_msgs__msg__NeuralNetLayer),
  NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_member_array,  // message members
  NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_init_function,  // function to initialize message memory (memory has to be allocated)
  NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_type_support_handle = {
  0,
  &NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcraicer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, NeuralNetLayer)() {
  if (!NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_type_support_handle.typesupport_identifier) {
    NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &NeuralNetLayer__rosidl_typesupport_introspection_c__NeuralNetLayer_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcraicer_msgs/msg/detail/neural_net_model__rosidl_typesupport_introspection_c.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcraicer_msgs/msg/detail/neural_net_model__functions.h"
#include "rcraicer_msgs/msg/detail/neural_net_model__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `network`
#include "rcraicer_msgs/msg/neural_net_layer.h"
// Member `network`
#include "rcraicer_msgs/msg/detail/neural_net_layer__rosidl_typesupport_introspection_c.h"
// Member `structure`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcraicer_msgs__msg__NeuralNetModel__init(message_memory);
}

void NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_fini_function(void * message_memory)
{
  rcraicer_msgs__msg__NeuralNetModel__fini(message_memory);
}

size_t NeuralNetModel__rosidl_typesupport_introspection_c__size_function__NeuralNetLayer__network(
  const void * untyped_member)
{
  const rcraicer_msgs__msg__NeuralNetLayer__Sequence * member =
    (const rcraicer_msgs__msg__NeuralNetLayer__Sequence *)(untyped_member);
  return member->size;
}

const void * NeuralNetModel__rosidl_typesupport_introspection_c__get_const_function__NeuralNetLayer__network(
  const void * untyped_member, size_t index)
{
  const rcraicer_msgs__msg__NeuralNetLayer__Sequence * member =
    (const rcraicer_msgs__msg__NeuralNetLayer__Sequence *)(untyped_member);
  return &member->data[index];
}

void * NeuralNetModel__rosidl_typesupport_introspection_c__get_function__NeuralNetLayer__network(
  void * untyped_member, size_t index)
{
  rcraicer_msgs__msg__NeuralNetLayer__Sequence * member =
    (rcraicer_msgs__msg__NeuralNetLayer__Sequence *)(untyped_member);
  return &member->data[index];
}

bool NeuralNetModel__rosidl_typesupport_introspection_c__resize_function__NeuralNetLayer__network(
  void * untyped_member, size_t size)
{
  rcraicer_msgs__msg__NeuralNetLayer__Sequence * member =
    (rcraicer_msgs__msg__NeuralNetLayer__Sequence *)(untyped_member);
  rcraicer_msgs__msg__NeuralNetLayer__Sequence__fini(member);
  return rcraicer_msgs__msg__NeuralNetLayer__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetModel, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_layers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetModel, num_layers),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "network",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetModel, network),  // bytes offset in struct
    NULL,  // default value
    NeuralNetModel__rosidl_typesupport_introspection_c__size_function__NeuralNetLayer__network,  // size() function pointer
    NeuralNetModel__rosidl_typesupport_introspection_c__get_const_function__NeuralNetLayer__network,  // get_const(index) function pointer
    NeuralNetModel__rosidl_typesupport_introspection_c__get_function__NeuralNetLayer__network,  // get(index) function pointer
    NeuralNetModel__rosidl_typesupport_introspection_c__resize_function__NeuralNetLayer__network  // resize(index) function pointer
  },
  {
    "structure",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs__msg__NeuralNetModel, structure),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_members = {
  "rcraicer_msgs__msg",  // message namespace
  "NeuralNetModel",  // message name
  4,  // number of fields
  sizeof(rcraicer_msgs__msg__NeuralNetModel),
  NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_member_array,  // message members
  NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_init_function,  // function to initialize message memory (memory has to be allocated)
  NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_type_support_handle = {
  0,
  &NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcraicer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, NeuralNetModel)() {
  NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcraicer_msgs, msg, NeuralNetLayer)();
  if (!NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_type_support_handle.typesupport_identifier) {
    NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &NeuralNetModel__rosidl_typesupport_introspection_c__NeuralNetModel_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

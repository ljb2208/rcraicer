// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rcraicer_msgs:msg/ChassisCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rcraicer_msgs/msg/detail/chassis_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcraicer_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ChassisCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcraicer_msgs::msg::ChassisCommand(_init);
}

void ChassisCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcraicer_msgs::msg::ChassisCommand *>(message_memory);
  typed_message->~ChassisCommand();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ChassisCommand_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::ChassisCommand, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "throttle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::ChassisCommand, throttle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "steer",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::ChassisCommand, steer),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sender",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::ChassisCommand, sender),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ChassisCommand_message_members = {
  "rcraicer_msgs::msg",  // message namespace
  "ChassisCommand",  // message name
  4,  // number of fields
  sizeof(rcraicer_msgs::msg::ChassisCommand),
  ChassisCommand_message_member_array,  // message members
  ChassisCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  ChassisCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ChassisCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ChassisCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rcraicer_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcraicer_msgs::msg::ChassisCommand>()
{
  return &::rcraicer_msgs::msg::rosidl_typesupport_introspection_cpp::ChassisCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcraicer_msgs, msg, ChassisCommand)() {
  return &::rcraicer_msgs::msg::rosidl_typesupport_introspection_cpp::ChassisCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rcraicer_msgs:msg/NeuralNetModel.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rcraicer_msgs/msg/detail/neural_net_model__struct.hpp"
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

void NeuralNetModel_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcraicer_msgs::msg::NeuralNetModel(_init);
}

void NeuralNetModel_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcraicer_msgs::msg::NeuralNetModel *>(message_memory);
  typed_message->~NeuralNetModel();
}

size_t size_function__NeuralNetModel__network(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rcraicer_msgs::msg::NeuralNetLayer> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NeuralNetModel__network(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rcraicer_msgs::msg::NeuralNetLayer> *>(untyped_member);
  return &member[index];
}

void * get_function__NeuralNetModel__network(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rcraicer_msgs::msg::NeuralNetLayer> *>(untyped_member);
  return &member[index];
}

void resize_function__NeuralNetModel__network(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rcraicer_msgs::msg::NeuralNetLayer> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NeuralNetModel__structure(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NeuralNetModel__structure(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__NeuralNetModel__structure(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void resize_function__NeuralNetModel__structure(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NeuralNetModel_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::NeuralNetModel, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "num_layers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::NeuralNetModel, num_layers),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "network",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcraicer_msgs::msg::NeuralNetLayer>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::NeuralNetModel, network),  // bytes offset in struct
    nullptr,  // default value
    size_function__NeuralNetModel__network,  // size() function pointer
    get_const_function__NeuralNetModel__network,  // get_const(index) function pointer
    get_function__NeuralNetModel__network,  // get(index) function pointer
    resize_function__NeuralNetModel__network  // resize(index) function pointer
  },
  {
    "structure",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcraicer_msgs::msg::NeuralNetModel, structure),  // bytes offset in struct
    nullptr,  // default value
    size_function__NeuralNetModel__structure,  // size() function pointer
    get_const_function__NeuralNetModel__structure,  // get_const(index) function pointer
    get_function__NeuralNetModel__structure,  // get(index) function pointer
    resize_function__NeuralNetModel__structure  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NeuralNetModel_message_members = {
  "rcraicer_msgs::msg",  // message namespace
  "NeuralNetModel",  // message name
  4,  // number of fields
  sizeof(rcraicer_msgs::msg::NeuralNetModel),
  NeuralNetModel_message_member_array,  // message members
  NeuralNetModel_init_function,  // function to initialize message memory (memory has to be allocated)
  NeuralNetModel_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NeuralNetModel_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NeuralNetModel_message_members,
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
get_message_type_support_handle<rcraicer_msgs::msg::NeuralNetModel>()
{
  return &::rcraicer_msgs::msg::rosidl_typesupport_introspection_cpp::NeuralNetModel_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcraicer_msgs, msg, NeuralNetModel)() {
  return &::rcraicer_msgs::msg::rosidl_typesupport_introspection_cpp::NeuralNetModel_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from rcraicer_msgs:msg/StateEstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rcraicer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "rcraicer_msgs/msg/detail/state_estimator_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace rcraicer_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcraicer_msgs
cdr_serialize(
  const rcraicer_msgs::msg::StateEstimatorStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcraicer_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rcraicer_msgs::msg::StateEstimatorStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcraicer_msgs
get_serialized_size(
  const rcraicer_msgs::msg::StateEstimatorStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcraicer_msgs
max_serialized_size_StateEstimatorStatus(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rcraicer_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcraicer_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcraicer_msgs, msg, StateEstimatorStatus)();

#ifdef __cplusplus
}
#endif

#endif  // RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

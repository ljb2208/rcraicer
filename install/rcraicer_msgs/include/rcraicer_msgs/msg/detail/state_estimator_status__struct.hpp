// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/StateEstimatorStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rcraicer_msgs__msg__StateEstimatorStatus __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__StateEstimatorStatus __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateEstimatorStatus_
{
  using Type = StateEstimatorStatus_<ContainerAllocator>;

  explicit StateEstimatorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit StateEstimatorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    unsigned char;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const unsigned char & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr unsigned char OK =
    0;
  static constexpr unsigned char WARN =
    1;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr unsigned char ERROR =
    2;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__StateEstimatorStatus
    std::shared_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__StateEstimatorStatus
    std::shared_ptr<rcraicer_msgs::msg::StateEstimatorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateEstimatorStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateEstimatorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateEstimatorStatus_

// alias to use template instance with default allocator
using StateEstimatorStatus =
  rcraicer_msgs::msg::StateEstimatorStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr unsigned char StateEstimatorStatus_<ContainerAllocator>::OK;
template<typename ContainerAllocator>
constexpr unsigned char StateEstimatorStatus_<ContainerAllocator>::WARN;
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
template<typename ContainerAllocator>
constexpr unsigned char StateEstimatorStatus_<ContainerAllocator>::ERROR;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__STATE_ESTIMATOR_STATUS__STRUCT_HPP_

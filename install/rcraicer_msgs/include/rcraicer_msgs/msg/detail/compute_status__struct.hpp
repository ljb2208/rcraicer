// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/ComputeStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__ComputeStatus __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__ComputeStatus __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ComputeStatus_
{
  using Type = ComputeStatus_<ContainerAllocator>;

  explicit ComputeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->host_name = "";
      this->mem_total = "";
      this->mem_free = "";
      this->mem_pct = "";
      this->stat_time = "";
      this->cpu1 = "";
      this->cpu2 = "";
      this->cpu3 = "";
      this->ssid = "";
      this->bit_rate = "";
      this->link_quality = "";
      this->power_level = "";
    }
  }

  explicit ComputeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    host_name(_alloc),
    mem_total(_alloc),
    mem_free(_alloc),
    mem_pct(_alloc),
    stat_time(_alloc),
    cpu1(_alloc),
    cpu2(_alloc),
    cpu3(_alloc),
    ssid(_alloc),
    bit_rate(_alloc),
    link_quality(_alloc),
    power_level(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->host_name = "";
      this->mem_total = "";
      this->mem_free = "";
      this->mem_pct = "";
      this->stat_time = "";
      this->cpu1 = "";
      this->cpu2 = "";
      this->cpu3 = "";
      this->ssid = "";
      this->bit_rate = "";
      this->link_quality = "";
      this->power_level = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _host_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _host_name_type host_name;
  using _mem_total_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _mem_total_type mem_total;
  using _mem_free_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _mem_free_type mem_free;
  using _mem_pct_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _mem_pct_type mem_pct;
  using _stat_time_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _stat_time_type stat_time;
  using _cpu1_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _cpu1_type cpu1;
  using _cpu2_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _cpu2_type cpu2;
  using _cpu3_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _cpu3_type cpu3;
  using _ssid_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _ssid_type ssid;
  using _bit_rate_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _bit_rate_type bit_rate;
  using _link_quality_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _link_quality_type link_quality;
  using _power_level_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _power_level_type power_level;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__host_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->host_name = _arg;
    return *this;
  }
  Type & set__mem_total(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->mem_total = _arg;
    return *this;
  }
  Type & set__mem_free(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->mem_free = _arg;
    return *this;
  }
  Type & set__mem_pct(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->mem_pct = _arg;
    return *this;
  }
  Type & set__stat_time(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->stat_time = _arg;
    return *this;
  }
  Type & set__cpu1(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->cpu1 = _arg;
    return *this;
  }
  Type & set__cpu2(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->cpu2 = _arg;
    return *this;
  }
  Type & set__cpu3(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->cpu3 = _arg;
    return *this;
  }
  Type & set__ssid(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->ssid = _arg;
    return *this;
  }
  Type & set__bit_rate(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->bit_rate = _arg;
    return *this;
  }
  Type & set__link_quality(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->link_quality = _arg;
    return *this;
  }
  Type & set__power_level(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->power_level = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__ComputeStatus
    std::shared_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__ComputeStatus
    std::shared_ptr<rcraicer_msgs::msg::ComputeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->host_name != other.host_name) {
      return false;
    }
    if (this->mem_total != other.mem_total) {
      return false;
    }
    if (this->mem_free != other.mem_free) {
      return false;
    }
    if (this->mem_pct != other.mem_pct) {
      return false;
    }
    if (this->stat_time != other.stat_time) {
      return false;
    }
    if (this->cpu1 != other.cpu1) {
      return false;
    }
    if (this->cpu2 != other.cpu2) {
      return false;
    }
    if (this->cpu3 != other.cpu3) {
      return false;
    }
    if (this->ssid != other.ssid) {
      return false;
    }
    if (this->bit_rate != other.bit_rate) {
      return false;
    }
    if (this->link_quality != other.link_quality) {
      return false;
    }
    if (this->power_level != other.power_level) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeStatus_

// alias to use template instance with default allocator
using ComputeStatus =
  rcraicer_msgs::msg::ComputeStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__COMPUTE_STATUS__STRUCT_HPP_

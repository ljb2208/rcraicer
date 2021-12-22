// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rcraicer_msgs:msg/GPSStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_HPP_

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
# define DEPRECATED__rcraicer_msgs__msg__GPSStatus __attribute__((deprecated))
#else
# define DEPRECATED__rcraicer_msgs__msg__GPSStatus __declspec(deprecated)
#endif

namespace rcraicer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSStatus_
{
  using Type = GPSStatus_<ContainerAllocator>;

  explicit GPSStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->satellites_used = 0;
      this->satellites_visible = 0;
      this->status = 0;
      this->rtk_status = 0;
      this->vdop = 0.0f;
      this->hdop = 0.0f;
      this->pdop = 0.0f;
      this->hacc = 0.0f;
      this->vacc = 0.0f;
      this->gspeed = 0.0f;
      this->sacc = 0.0f;
      this->headmot = 0.0f;
      this->headacc = 0.0f;
    }
  }

  explicit GPSStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->satellites_used = 0;
      this->satellites_visible = 0;
      this->status = 0;
      this->rtk_status = 0;
      this->vdop = 0.0f;
      this->hdop = 0.0f;
      this->pdop = 0.0f;
      this->hacc = 0.0f;
      this->vacc = 0.0f;
      this->gspeed = 0.0f;
      this->sacc = 0.0f;
      this->headmot = 0.0f;
      this->headacc = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _satellites_used_type =
    uint16_t;
  _satellites_used_type satellites_used;
  using _satellite_used_prn_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _satellite_used_prn_type satellite_used_prn;
  using _satellites_visible_type =
    uint16_t;
  _satellites_visible_type satellites_visible;
  using _satellite_visible_prn_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _satellite_visible_prn_type satellite_visible_prn;
  using _satellite_visible_z_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _satellite_visible_z_type satellite_visible_z;
  using _satellite_visible_azimuth_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _satellite_visible_azimuth_type satellite_visible_azimuth;
  using _satellite_visible_snr_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _satellite_visible_snr_type satellite_visible_snr;
  using _status_type =
    int16_t;
  _status_type status;
  using _rtk_status_type =
    int16_t;
  _rtk_status_type rtk_status;
  using _vdop_type =
    float;
  _vdop_type vdop;
  using _hdop_type =
    float;
  _hdop_type hdop;
  using _pdop_type =
    float;
  _pdop_type pdop;
  using _hacc_type =
    float;
  _hacc_type hacc;
  using _vacc_type =
    float;
  _vacc_type vacc;
  using _gspeed_type =
    float;
  _gspeed_type gspeed;
  using _sacc_type =
    float;
  _sacc_type sacc;
  using _headmot_type =
    float;
  _headmot_type headmot;
  using _headacc_type =
    float;
  _headacc_type headacc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__satellites_used(
    const uint16_t & _arg)
  {
    this->satellites_used = _arg;
    return *this;
  }
  Type & set__satellite_used_prn(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->satellite_used_prn = _arg;
    return *this;
  }
  Type & set__satellites_visible(
    const uint16_t & _arg)
  {
    this->satellites_visible = _arg;
    return *this;
  }
  Type & set__satellite_visible_prn(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->satellite_visible_prn = _arg;
    return *this;
  }
  Type & set__satellite_visible_z(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->satellite_visible_z = _arg;
    return *this;
  }
  Type & set__satellite_visible_azimuth(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->satellite_visible_azimuth = _arg;
    return *this;
  }
  Type & set__satellite_visible_snr(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->satellite_visible_snr = _arg;
    return *this;
  }
  Type & set__status(
    const int16_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__rtk_status(
    const int16_t & _arg)
  {
    this->rtk_status = _arg;
    return *this;
  }
  Type & set__vdop(
    const float & _arg)
  {
    this->vdop = _arg;
    return *this;
  }
  Type & set__hdop(
    const float & _arg)
  {
    this->hdop = _arg;
    return *this;
  }
  Type & set__pdop(
    const float & _arg)
  {
    this->pdop = _arg;
    return *this;
  }
  Type & set__hacc(
    const float & _arg)
  {
    this->hacc = _arg;
    return *this;
  }
  Type & set__vacc(
    const float & _arg)
  {
    this->vacc = _arg;
    return *this;
  }
  Type & set__gspeed(
    const float & _arg)
  {
    this->gspeed = _arg;
    return *this;
  }
  Type & set__sacc(
    const float & _arg)
  {
    this->sacc = _arg;
    return *this;
  }
  Type & set__headmot(
    const float & _arg)
  {
    this->headmot = _arg;
    return *this;
  }
  Type & set__headacc(
    const float & _arg)
  {
    this->headacc = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int16_t STATUS_NO_FIX =
    -1;
  static constexpr int16_t STATUS_FIX =
    0;
  static constexpr int16_t STATUS_SBAS_FIX =
    1;
  static constexpr int16_t STATUS_GBAS_FIX =
    2;
  static constexpr int16_t STATUS_DGPS_FIX =
    18;
  static constexpr int16_t STATUS_WAAS_FIX =
    33;
  static constexpr int16_t RTK_STATUS_NONE =
    0;
  static constexpr int16_t RTK_STATUS_FLOAT =
    1;
  static constexpr int16_t RTK_STATUS_FIXED =
    2;

  // pointer types
  using RawPtr =
    rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rcraicer_msgs__msg__GPSStatus
    std::shared_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rcraicer_msgs__msg__GPSStatus
    std::shared_ptr<rcraicer_msgs::msg::GPSStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->satellites_used != other.satellites_used) {
      return false;
    }
    if (this->satellite_used_prn != other.satellite_used_prn) {
      return false;
    }
    if (this->satellites_visible != other.satellites_visible) {
      return false;
    }
    if (this->satellite_visible_prn != other.satellite_visible_prn) {
      return false;
    }
    if (this->satellite_visible_z != other.satellite_visible_z) {
      return false;
    }
    if (this->satellite_visible_azimuth != other.satellite_visible_azimuth) {
      return false;
    }
    if (this->satellite_visible_snr != other.satellite_visible_snr) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->rtk_status != other.rtk_status) {
      return false;
    }
    if (this->vdop != other.vdop) {
      return false;
    }
    if (this->hdop != other.hdop) {
      return false;
    }
    if (this->pdop != other.pdop) {
      return false;
    }
    if (this->hacc != other.hacc) {
      return false;
    }
    if (this->vacc != other.vacc) {
      return false;
    }
    if (this->gspeed != other.gspeed) {
      return false;
    }
    if (this->sacc != other.sacc) {
      return false;
    }
    if (this->headmot != other.headmot) {
      return false;
    }
    if (this->headacc != other.headacc) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSStatus_

// alias to use template instance with default allocator
using GPSStatus =
  rcraicer_msgs::msg::GPSStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_NO_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_SBAS_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_GBAS_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_DGPS_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::STATUS_WAAS_FIX;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::RTK_STATUS_NONE;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::RTK_STATUS_FLOAT;
template<typename ContainerAllocator>
constexpr int16_t GPSStatus_<ContainerAllocator>::RTK_STATUS_FIXED;

}  // namespace msg

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__STRUCT_HPP_

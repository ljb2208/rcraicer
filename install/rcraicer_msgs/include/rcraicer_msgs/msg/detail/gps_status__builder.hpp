// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcraicer_msgs:msg/GPSStatus.idl
// generated code does not contain a copyright notice

#ifndef RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__BUILDER_HPP_
#define RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__BUILDER_HPP_

#include "rcraicer_msgs/msg/detail/gps_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcraicer_msgs
{

namespace msg
{

namespace builder
{

class Init_GPSStatus_headacc
{
public:
  explicit Init_GPSStatus_headacc(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  ::rcraicer_msgs::msg::GPSStatus headacc(::rcraicer_msgs::msg::GPSStatus::_headacc_type arg)
  {
    msg_.headacc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_headmot
{
public:
  explicit Init_GPSStatus_headmot(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_headacc headmot(::rcraicer_msgs::msg::GPSStatus::_headmot_type arg)
  {
    msg_.headmot = std::move(arg);
    return Init_GPSStatus_headacc(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_sacc
{
public:
  explicit Init_GPSStatus_sacc(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_headmot sacc(::rcraicer_msgs::msg::GPSStatus::_sacc_type arg)
  {
    msg_.sacc = std::move(arg);
    return Init_GPSStatus_headmot(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_gspeed
{
public:
  explicit Init_GPSStatus_gspeed(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_sacc gspeed(::rcraicer_msgs::msg::GPSStatus::_gspeed_type arg)
  {
    msg_.gspeed = std::move(arg);
    return Init_GPSStatus_sacc(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_vacc
{
public:
  explicit Init_GPSStatus_vacc(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_gspeed vacc(::rcraicer_msgs::msg::GPSStatus::_vacc_type arg)
  {
    msg_.vacc = std::move(arg);
    return Init_GPSStatus_gspeed(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_hacc
{
public:
  explicit Init_GPSStatus_hacc(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_vacc hacc(::rcraicer_msgs::msg::GPSStatus::_hacc_type arg)
  {
    msg_.hacc = std::move(arg);
    return Init_GPSStatus_vacc(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_pdop
{
public:
  explicit Init_GPSStatus_pdop(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_hacc pdop(::rcraicer_msgs::msg::GPSStatus::_pdop_type arg)
  {
    msg_.pdop = std::move(arg);
    return Init_GPSStatus_hacc(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_hdop
{
public:
  explicit Init_GPSStatus_hdop(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_pdop hdop(::rcraicer_msgs::msg::GPSStatus::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_GPSStatus_pdop(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_vdop
{
public:
  explicit Init_GPSStatus_vdop(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_hdop vdop(::rcraicer_msgs::msg::GPSStatus::_vdop_type arg)
  {
    msg_.vdop = std::move(arg);
    return Init_GPSStatus_hdop(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_rtk_status
{
public:
  explicit Init_GPSStatus_rtk_status(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_vdop rtk_status(::rcraicer_msgs::msg::GPSStatus::_rtk_status_type arg)
  {
    msg_.rtk_status = std::move(arg);
    return Init_GPSStatus_vdop(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_status
{
public:
  explicit Init_GPSStatus_status(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_rtk_status status(::rcraicer_msgs::msg::GPSStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GPSStatus_rtk_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellite_visible_snr
{
public:
  explicit Init_GPSStatus_satellite_visible_snr(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_status satellite_visible_snr(::rcraicer_msgs::msg::GPSStatus::_satellite_visible_snr_type arg)
  {
    msg_.satellite_visible_snr = std::move(arg);
    return Init_GPSStatus_status(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellite_visible_azimuth
{
public:
  explicit Init_GPSStatus_satellite_visible_azimuth(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellite_visible_snr satellite_visible_azimuth(::rcraicer_msgs::msg::GPSStatus::_satellite_visible_azimuth_type arg)
  {
    msg_.satellite_visible_azimuth = std::move(arg);
    return Init_GPSStatus_satellite_visible_snr(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellite_visible_z
{
public:
  explicit Init_GPSStatus_satellite_visible_z(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellite_visible_azimuth satellite_visible_z(::rcraicer_msgs::msg::GPSStatus::_satellite_visible_z_type arg)
  {
    msg_.satellite_visible_z = std::move(arg);
    return Init_GPSStatus_satellite_visible_azimuth(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellite_visible_prn
{
public:
  explicit Init_GPSStatus_satellite_visible_prn(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellite_visible_z satellite_visible_prn(::rcraicer_msgs::msg::GPSStatus::_satellite_visible_prn_type arg)
  {
    msg_.satellite_visible_prn = std::move(arg);
    return Init_GPSStatus_satellite_visible_z(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellites_visible
{
public:
  explicit Init_GPSStatus_satellites_visible(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellite_visible_prn satellites_visible(::rcraicer_msgs::msg::GPSStatus::_satellites_visible_type arg)
  {
    msg_.satellites_visible = std::move(arg);
    return Init_GPSStatus_satellite_visible_prn(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellite_used_prn
{
public:
  explicit Init_GPSStatus_satellite_used_prn(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellites_visible satellite_used_prn(::rcraicer_msgs::msg::GPSStatus::_satellite_used_prn_type arg)
  {
    msg_.satellite_used_prn = std::move(arg);
    return Init_GPSStatus_satellites_visible(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_satellites_used
{
public:
  explicit Init_GPSStatus_satellites_used(::rcraicer_msgs::msg::GPSStatus & msg)
  : msg_(msg)
  {}
  Init_GPSStatus_satellite_used_prn satellites_used(::rcraicer_msgs::msg::GPSStatus::_satellites_used_type arg)
  {
    msg_.satellites_used = std::move(arg);
    return Init_GPSStatus_satellite_used_prn(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

class Init_GPSStatus_header
{
public:
  Init_GPSStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSStatus_satellites_used header(::rcraicer_msgs::msg::GPSStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSStatus_satellites_used(msg_);
  }

private:
  ::rcraicer_msgs::msg::GPSStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcraicer_msgs::msg::GPSStatus>()
{
  return rcraicer_msgs::msg::builder::Init_GPSStatus_header();
}

}  // namespace rcraicer_msgs

#endif  // RCRAICER_MSGS__MSG__DETAIL__GPS_STATUS__BUILDER_HPP_

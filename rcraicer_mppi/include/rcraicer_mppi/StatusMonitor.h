#ifndef MPPI_STATUS_MONITOR_
#define MPPI_STATUS_MONITOR_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcraicer_msgs/msg/mppi_status.hpp"

class StatusMonitor : public rclcpp::Node
{

public: 

	const double TIMEOUT = 0.5;

	StatusMonitor(ros::NodeHandle nh);

	void statusCallback(rcraicer_msgs::msg::MPPIStatus msg); 

	void diagnosticStatus(const ros::TimerEvent& time);

private:

	rclcpp::Time last_status_;
	std::string info_;
	int status_;
	ros::Subscriber status_sub_;

};

#endif /*MPPI_STATUS_MONITOR_*/
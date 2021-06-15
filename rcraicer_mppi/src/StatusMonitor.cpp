
#include "../include/rcraicer_mppi/StatusMonitor.h"


StatusMonitor::StatusMonitor Node("mppi_status_monitor")
{
	last_status_ = rclcpp::get_clock();
	status_sub_ = nh.subscribe("/mppi_controller/mppiStatus", 1, &StatusMonitor::statusCallback, this);
	std::string info = "MPPI Controller";
	std::string hardwareID = "none";
	std::string portPath = "";
	Diagnostics::init(info, hardwareID, portPath);
}

void StatusMonitor::statusCallback(rcraicer_msgs::msg::MPPIStatus msg)
{
	info_ = msg.info;
	status_ = msg.status;
	last_status_ = ros::Time::now();
}

void StatusMonitor::diagnosticStatus(const ros::TimerEvent& time)
{
	if ((double)ros::Time::now().toSec() - (double)last_status_.toSec() > TIMEOUT){
		diag_error("CONTROLLER TIMEOUT");
	}
	else if (status_ == 0){
		diag_ok(info_);
	}
	else if (status_ == 1){
		diag_warn(info_);
	}
	else if (status_ == 2){
		diag_error(info_);
	}
}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusMonitor>());
    rclcpp::shutdown();
    return 0;
}
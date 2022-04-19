#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <chrono>

#include "rcraicer_msgs/msg/run_stop.hpp"

#include "serial_port.h"


class RunStop : public rclcpp::Node 
{
    public:
        RunStop();
        ~RunStop();

    private:                
        rclcpp::Publisher<rcraicer_msgs::msg::RunStop>::SharedPtr rsPublisher;        
        
        rclcpp::Parameter port_param;
        rclcpp::Parameter rate_param;        
        rclcpp::Parameter baud_rate_param;        

        SerialPort* serialPort;

        std::string state_; ///< Current run stop button state received from Arduino
        rclcpp::Time lastMessageTime_; ///< Time of most recent message from Arduino

        bool processData();
        void doWorkTimerCallback();

        rclcpp::TimerBase::SharedPtr rsControlTimer;

};
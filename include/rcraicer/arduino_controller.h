#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>
#include "serial_port.h"


class ArduinoController : public rclcpp::Node 
{
    public: 
        ArduinoController();        

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;
        rclcpp::AsyncParametersClient::SharedPtr parameterClient;

        SerialPort serialPort;

        rclcpp::Parameter portPath;
        rclcpp::Parameter throttleAxis;
        rclcpp::Parameter steeringAxis;
  
};
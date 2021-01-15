#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"


class ArduinoController : public rclcpp::Node 
{
    public: 
        ArduinoController();        

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const;   
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

};
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>

#include "serial_port.h"
#include "serial_data_msg.h"


class ArduinoController : public rclcpp::Node 
{
    public: 
        ArduinoController();        
        ~ArduinoController();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);
        void serial_data_callback();
        
        void writeData(data_msg dmsg);        
        void processMessage(unsigned char* data, int length);

        void updateInternalParams();

        int32_t getSteeringPWM(float value);
        int32_t getThrottlePWM(float value);
        int32_t getPWM(float value, int64_t min, int64_t mid, int64_t max);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;
        rclcpp::AsyncParametersClient::SharedPtr parameterClient;

        SerialPort* serialPort;

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;
        rclcpp::Parameter throttleAxis;
        rclcpp::Parameter steeringAxis;
        rclcpp::Parameter steeringServoPoints;
        rclcpp::Parameter throttleServoPoints;

        int64_t steeringServoMin;
        int64_t steeringServoMax;
        int64_t steeringServoMid;

        int64_t throttleServoMin;
        int64_t throttleServoMax;
        int64_t throttleServoMid;

        int64_t steeringAxisID;
        int64_t throttleAxisID;

        bool published;
  
};
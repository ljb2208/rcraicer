#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>

#include "serial_port.h"
#include "serial_data_msg.h"

#include "rcraicer_msgs/msg/encoder.hpp"


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
        int32_t getPWM(float value, float inputFactor, int64_t min, int64_t mid, int64_t max);

        rclcpp::Publisher<rcraicer_msgs::msg::Encoder>::SharedPtr encPublisher;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;
        rclcpp::AsyncParametersClient::SharedPtr parameterClient;

        SerialPort* serialPort;

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;
        rclcpp::Parameter throttleAxis;
        rclcpp::Parameter steeringAxis;
        rclcpp::Parameter reverseThrottleInput;
        rclcpp::Parameter reverseSteeringInput;
        rclcpp::Parameter steeringServoPoints;
        rclcpp::Parameter throttleServoPoints;

        float steeringInputFactor;
        float throttleInputFactor;

        int64_t steeringServoMin;
        int64_t steeringServoMax;
        int64_t steeringServoMid;

        int64_t throttleServoMin;
        int64_t throttleServoMax;
        int64_t throttleServoMid;

        int64_t steeringAxisID;
        int64_t throttleAxisID;
  
};
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>

#include "serial_port.h"
#include "serial_data_msg.h"

#include "rcraicer_msgs/msg/encoder.hpp"
#include "rcraicer_msgs/msg/arduino_status.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/wheel_speed.hpp"


class ArduinoController : public rclcpp::Node 
{
    public: 
        ArduinoController();        
        ~ArduinoController();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg);        
        void serial_data_callback();
        void sendActuatorData(float throttle, float steer);
        void publishChassisState(float throttle, float steer, float steerAngle);
        
        void writeData(data_msg dmsg);        
        void processMessage(unsigned char* data, int length);

        void updateInternalParams();

        float getSteeringAngle(int steerPWM);
        int32_t getSteeringPWM(float value);
        int32_t getThrottlePWM(float value);
        int32_t getPWM(float value, float inputFactor, int64_t min, int64_t mid, int64_t max);

        rclcpp::Publisher<rcraicer_msgs::msg::Encoder>::SharedPtr encPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::WheelSpeed>::SharedPtr wsPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ArduinoStatus>::SharedPtr statusPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisState>::SharedPtr statePublisher;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisCommand>::SharedPtr commandSubscription;        

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;

        SerialPort* serialPort;

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;
        rclcpp::Parameter throttleAxis;
        rclcpp::Parameter steeringAxis;
        rclcpp::Parameter armButton;
        rclcpp::Parameter throttleInputFactorParam;
        rclcpp::Parameter steeringInputFactorParam;
        rclcpp::Parameter steeringServoPoints;
        rclcpp::Parameter throttleServoPoints;
        rclcpp::Parameter steeringDegreesPerTick;
        rclcpp::Parameter wheelDiameterParam;

        double steeringAngleCoefficient;

        float steeringInputFactor;
        float throttleInputFactor;

        double wheelDiameter;

        int64_t steeringServoMin;
        int64_t steeringServoMax;
        int64_t steeringServoMid;

        int64_t throttleServoMin;
        int64_t throttleServoMax;
        int64_t throttleServoMid;

        int64_t steeringAxisID;
        int64_t throttleAxisID;

        int64_t armButtonID;        

        bool cmdReceived;
        float lastCommandTime;

        bool isArmed;
        int64_t armButtonValue;
        uint16_t invalidCRC;
        uint16_t unknownMsg;

        const double PI = 3.14159265; ///< Value for pi

  
};
#include "../include/rcraicer/arduino_controller.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

ArduinoController::ArduinoController() : Node("arduino_controller")
{
    // init parameters    
    rclcpp::Parameter defPortPath;
    this->get_parameter_or("serial_port", portPath, rclcpp::Parameter("serial_port", "/dev/ttyUSB0"));    
    this->get_parameter_or("steering_axis", steeringAxis, rclcpp::Parameter("steering_axis", 0));
    this->get_parameter_or("throttle_axis", throttleAxis, rclcpp::Parameter("throttleAxis", 1));
    
    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", std::bind(&ArduinoController::joy_callback, this, _1));   // add queue size in later versions of ros2       

    parameterClient = std::make_shared<rclcpp::AsyncParametersClient>(this);
    paramSubscription = parameterClient->on_parameter_event(std::bind(&ArduinoController::param_callback, this, _1));

    while (!parameterClient->wait_for_service(10s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service.");
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Parameter service available.");
    RCLCPP_INFO(this->get_logger(), "Node started. Connected on %s. Steering Axis: %i. Throttle Axis: %i", portPath.as_string().c_str(), 
                                    steeringAxis.as_int(), throttleAxis.as_int());    
}
 
void ArduinoController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{    
    if (msg->axes[0] == 0.0)
    {

    }
    // RCLCPP_INFO(this->get_logger(), "Received joy msg: %d %d", msg->axes[0], msg->axes[1]);
}

void ArduinoController::param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent)
{
    for (auto & param : paramEvent->changed_parameters)
    {
        if (param.name == "serial_port")
        {
            portPath.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Serial Port Param changed: %s", portPath.as_string().c_str());    
        }
        else if (param.name == "steering_axis")
        {
            steeringAxis.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Steering Axis Param changed: %i", steeringAxis.as_int());    
        }
        else if (param.name == "throttle_axis")
        {
            throttleAxis.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Throttle Axis Param changed: %i", throttleAxis.as_int());    
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoController>());
    rclcpp::shutdown();
    return 0;
}
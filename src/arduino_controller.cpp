#include "../include/rcraicer/arduino_controller.h"

using std::placeholders::_1;

ArduinoController::ArduinoController() : Node("ArduinoController")
{
    // joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
    //   "joy", 10, std::bind(&ArduinoController::joy_callback, this, _1));     

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ArduinoController::joy_callback, this, _1));    

    
}
 
void ArduinoController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received joy msg: %i", msg->buttons[0]);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoController>());
    rclcpp::shutdown();
    return 0;
}
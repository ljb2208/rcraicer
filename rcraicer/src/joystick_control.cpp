#include "../include/rcraicer/joystick_control.h"

using namespace std::chrono_literals;

JoystickControl::JoystickControl() : Node("joystick_control") , throttleEnabled_(true), steeringEnabled_(true), throttleDamping_(0.0),
    steeringDamping_(0.0)
{                        
    this->declare_parameter("throttle_damping", 0.0);
    this->declare_parameter("steering_damping", 0.0);

    this->declare_parameter("throttle_axis", 1);
    this->declare_parameter("steering_axis", 3);
    this->declare_parameter("brake_axis", 5);

    this->declare_parameter("throttle_enable_button");
    this->declare_parameter("steering_enable_button");    

    this->declare_parameter("runstop_toggle_buttons");    

    
    throttleAxis_ = this->get_parameter("throttle_axis").as_int();    
    steeringAxis_ = this->get_parameter("steering_axis").as_int();    
    brakeAxis_ = this->get_parameter("brake_axis").as_int();    

    throttleDamping_ = this->get_parameter("throttle_damping").as_double();
    steeringDamping_ = this->get_parameter("steering_damping").as_double();

    throttleEnableButton_ = this->get_parameter("throttle_enable_button").as_int();
    steeringEnableButton_ = this->get_parameter("steering_enable_button").as_int();

    runstopToggleButtons_ = this->get_parameter("runstop_toggle_buttons").as_integer_array();  

    runstop_.sender = "joystick";
    runstop_.motion_enabled = false;

    commandPublisher = this->create_publisher<rcraicer_msgs::msg::ChassisCommand>("joystick/chassisCommand", 1);      
    rsPublisher = this->create_publisher<rcraicer_msgs::msg::RunStop>("runstop", 1);      

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickControl::joy_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       
       

    // create runstop timer
    rsControlTimer = this->create_wall_timer(std::chrono::milliseconds(200) , 
            std::bind(&JoystickControl::doWorkTimerCallback, this));
}

JoystickControl::~JoystickControl()
{
    
}

void JoystickControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    //toggle runstop if a runstop toggle button changed from 0 to 1
    for(auto vecIt : runstopToggleButtons_)
    {
        if(joy->buttons[vecIt] == 1 && prevJoy_.buttons[vecIt] == 0)
        {
            runstop_.motion_enabled = !runstop_.motion_enabled;
        }
    }

    //can enable/disable throttle control with L2 on game pad, only toggle if button changed from 0 to 1
    if(joy->buttons[throttleEnableButton_] == 1 && prevJoy_.buttons[throttleEnableButton_] == 0)
    {
        throttleEnabled_ = !throttleEnabled_;
    }

    //can enable/disable steering control with R2 on game pad, only toggle if button changed from 0 to 1
    if(joy->buttons[steeringEnableButton_] == 1 && prevJoy_.buttons[steeringEnableButton_] == 0)
    {
        steeringEnabled_ = !steeringEnabled_;
    }

    if(steeringEnabled_)
    {
        chassis_command_.steer = -steeringDamping_*joy->axes[steeringAxis_];
    } else
    {
        chassis_command_.steer = -10.0;
    }

    if(throttleEnabled_)
    {

        chassis_command_.throttle = throttleDamping_*joy->axes[throttleAxis_];

        if(chassis_command_.throttle < 0.0)
        {
            chassis_command_.front_brake = fabs(chassis_command_.throttle);
        } else
        {
            chassis_command_.front_brake = 0.0;
        }
    } else
    {
        chassis_command_.throttle = -10.0;
        chassis_command_.front_brake = -10.0;
    }

    prevJoy_ = *joy;
    chassis_command_.header.frame_id = "joystick";
    chassis_command_.sender = "joystick";
    chassis_command_.header.stamp = this->get_clock()->now();
    commandPublisher->publish(chassis_command_);
}

void JoystickControl::doWorkTimerCallback()
{
    runstop_.header.stamp = this->get_clock()->now();
    rsPublisher->publish(runstop_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickControl>());
    rclcpp::shutdown();
    return 0;
}
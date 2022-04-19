#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <chrono>

#include "sensor_msgs/msg/joy.hpp"

#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/run_stop.hpp"


class JoystickControl : public rclcpp::Node 
{
    public:
        JoystickControl();
        ~JoystickControl();

    private:                
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisCommand>::SharedPtr commandPublisher;        
        rclcpp::Publisher<rcraicer_msgs::msg::RunStop>::SharedPtr rsPublisher;        

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);


        sensor_msgs::msg::Joy prevJoy_;
        rcraicer_msgs::msg::RunStop runstop_;
        rcraicer_msgs::msg::ChassisCommand chassis_command_;
        
        double throttleDamping_;
        double steeringDamping_;
        
        bool throttleEnabled_;
        bool steeringEnabled_;
        bool throttleBrakePaired_;
        bool frontBrake_;

        int throttleAxis_;
        int steeringAxis_;
        int brakeAxis_;
        int throttleEnableButton_;
        int steeringEnableButton_;
        std::vector<int64_t> runstopToggleButtons_;

        void doWorkTimerCallback();

        rclcpp::TimerBase::SharedPtr rsControlTimer;

};
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include <tf2_ros/transform_broadcaster.h>


class JointStates : public rclcpp::Node 
{
    public:
        JointStates();
        ~JointStates();

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jsPublisher;
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisState>::SharedPtr csSubscriber;    

        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;        

        void cs_callback(const rcraicer_msgs::msg::ChassisState::SharedPtr msg);
        void publishJointStates(float steer);
};

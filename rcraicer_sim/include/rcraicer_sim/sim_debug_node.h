#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <vector>

class SimDebugNode : public rclcpp::Node
{
    public:
        SimDebugNode();
        ~SimDebugNode();

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::SimState>::SharedPtr simSubscription;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odomMarkerPublisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gtMarkerPublisher;

        void onOdomMessage(const nav_msgs::msg::Odometry::SharedPtr msg);
        void onSimMessage(const rcraicer_msgs::msg::SimState::SharedPtr msg);

        int odomMarkerId {0};
        int gtMarkerId {0};

};
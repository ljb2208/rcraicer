#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>


#include "tcp_server.h"

class SimNode : public rclcpp::Node
{
    public:
        SimNode();
        ~SimNode();
        
        void publishTelemetryMessages(rcraicer_msgs::msg::WheelSpeed wsMsg, rcraicer_msgs::msg::ChassisState csMsg, sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::NavSatFix fixMsg);

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::WheelSpeed>::SharedPtr wsPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisState>::SharedPtr csPublisher;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fixPublisher;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisCommand>::SharedPtr cmdSubscription;

        rclcpp::TimerBase::SharedPtr connectTimer;
        void connectTimerCallback();  
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg);

        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);        
        void updateInternalParams();

        void sendControls(float throttle, float steering, float brake);

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;

        void setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev);

        rclcpp::Parameter ipAddress;
        rclcpp::Parameter port;

        rclcpp::Parameter linear_stdev;
        rclcpp::Parameter angular_stdev;
        rclcpp::Parameter orientation_stdev;
        rclcpp::Parameter mag_stdev;
        rclcpp::Parameter frame_id_param;
        rclcpp::Parameter pub_freq;
        rclcpp::Parameter steering_axis;
        rclcpp::Parameter throttle_axis;
        rclcpp::Parameter auto_button;
        rclcpp::Parameter reverse_steering;
        rclcpp::Parameter reverse_throttle;

        sensor_msgs::msg::Imu::_angular_velocity_covariance_type linear_acceleration_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type angular_velocity_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type unk_orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type magnetic_cov;

        std::string frame_id;              

        TcpServer* server;

        int steeringAxisID;
        int throttleAxisID;
        int autoButtonID;

        int reverseSteering;
        int reverseThrottle;

        bool autoEnabled;


};
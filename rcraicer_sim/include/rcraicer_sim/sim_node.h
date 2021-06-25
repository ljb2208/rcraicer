#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>


#include "tcp_server.h"

class SimNode : public rclcpp::Node
{
    public:
        SimNode();
        ~SimNode();

        void publishSensorMessages(sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::MagneticField magMsg, sensor_msgs::msg::Temperature tempMsg);
        
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisherRaw;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher;  

        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);        
        void updateInternalParams();

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

        sensor_msgs::msg::Imu::_angular_velocity_covariance_type linear_acceleration_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type angular_velocity_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type unk_orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type magnetic_cov;

        std::string frame_id;              

        TcpServer* server;


};
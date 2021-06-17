#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "y3_protocol.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>


class Y3Imu : public rclcpp::Node
{
    public:
        Y3Imu();
        ~Y3Imu();

        void publishIMUMessage(sensor_msgs::msg::Imu msg);
        void publishRawIMUMessage(sensor_msgs::msg::Imu msg);
        void publishMagMessage(sensor_msgs::msg::MagneticField msg);
        void publishTempMessage(sensor_msgs::msg::Temperature msg);

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

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;

        rclcpp::Parameter linear_stdev;
        rclcpp::Parameter angular_stdev;
        rclcpp::Parameter orientation_stdev;
        rclcpp::Parameter mag_stdev;
        rclcpp::Parameter frame_id_param;

        sensor_msgs::msg::Imu::_angular_velocity_covariance_type linear_acceleration_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type angular_velocity_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type unk_orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type magnetic_cov;

        std::string frame_id;      

        Y3Protocol* proto;


};
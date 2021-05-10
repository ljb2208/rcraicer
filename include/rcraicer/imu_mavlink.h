#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include <tf2_eigen/tf2_eigen.h>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "mavlink_serial_port.h"
#include "serial_data_msg.h"
#include "../mavlink/common/mavlink.h"

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
 {
         // YPR - ZYX
         return Eigen::Quaterniond(
                         Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                         );
 }

static constexpr double GAUSS_TO_TESLA = 1.0e-4;

static const auto AIRCRAFT_BASELINK_Q = quaternion_from_rpy(Eigen::Vector3d(M_PI, 0.0, 0.0));
static const auto NED_ENU_Q = quaternion_from_rpy(Eigen::Vector3d(M_PI, 0.0, M_PI_2));
static const Eigen::Affine3d AIRCRAFT_BASELINK_AFFINE(AIRCRAFT_BASELINK_Q);
static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1,0,2));
static const Eigen::DiagonalMatrix<double,3> NED_ENU_REFLECTION_Z(1,1,-1);

class IMUMavlink : public rclcpp::Node 
{
    public: 
        IMUMavlink();        
        ~IMUMavlink();

        enum class StaticTF {
            NED_TO_ENU,		//!< will change orientation from being expressed WRT NED frame to WRT ENU frame
            ENU_TO_NED,		//!< change from expressed WRT ENU frame to WRT NED frame
            AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
            BASELINK_TO_AIRCRAFT,	//!< change from expressed WRT baselnk to WRT aircraft
            ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK,//!< change orientation from being expressed in aircraft frame to baselink frame in an absolute frame of reference.
            ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT,//!< change orientation from being expressed in baselink frame to aircraft frame in an absolute frame of reference
        };

    private:        
        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);
        void serial_data_callback();
        void updateInternalParams();
        void publishImuData(uint32_t time_boot_ms, Eigen::Quaterniond &orientation_enu, Eigen::Vector3d &gyro_flu);
        void publishImuDataRaw(uint32_t time_usec, Eigen::Vector3d &gyro_flu, Eigen::Vector3d &accel_flu, Eigen::Vector3d &accel_frd);
        void publishMagData(uint32_t time_boot_ms, Eigen::Vector3d &mag_field);

        void handleAttitudeQ(mavlink_attitude_quaternion_t attitude_q);
        void handleImu(mavlink_highres_imu_t imu_hr);

        void setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev);

        // Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);
        Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);                
        Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform);

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisherRaw;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher;        

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;

        MavlinkSerialPort* serialPort;

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

        int32_t rxDropCount;        
        Eigen::Vector3d linear_accel_vec_flu;     
        Eigen::Vector3d linear_accel_vec_frd;     

        std::string frame_id;      

        bool received_linear_accel = false;
};
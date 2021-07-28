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
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"
#include "rcraicer_msgs/msg/imu_filter_output.hpp"

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <vector>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */

class AirSimNode : public rclcpp::Node
{
    public:
        AirSimNode();
        ~AirSimNode();        

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::WheelSpeed>::SharedPtr wsPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisState>::SharedPtr csPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::SimState>::SharedPtr ssPublisher;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fixPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;       
        rclcpp::Publisher<rcraicer_msgs::msg::ImuFilterOutput>::SharedPtr imuFilterPublisher; 

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisCommand>::SharedPtr cmdSubscription;

        tf2_ros::StaticTransformBroadcaster staticBroadcaster;
        tf2_ros::TransformBroadcaster baseToOdomBroadcaster;

        rclcpp::TimerBase::SharedPtr connectTimer;
        rclcpp::TimerBase::SharedPtr sensorTimer;
        rclcpp::TimerBase::SharedPtr stateTimer;
        rclcpp::TimerBase::SharedPtr gpsTimer;

        void connectTimerCallback();                  
        void disconnected();

        void publishSensorData();        
        void publishImuData();
        void publishGpsData();
        void publishMagData();
        void publishStateData();

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg);
        
        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);        
        void updateInternalParams();

        void setActive(bool active);

        void sendControls(float throttle, float steering, float brake);

        std::string getScene();

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;

        void setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev);

        geometry_msgs::msg::Quaternion atorQuat(const msr::airlib::Quaternionr& airlib_quat);
        geometry_msgs::msg::Vector3 atorVec(const msr::airlib::Vector3r& airlib_vec);
        geometry_msgs::msg::Point atorPoint(const msr::airlib::Vector3r& airlib_vec);
        geometry_msgs::msg::Quaternion atorQuatLocal(const msr::airlib::Quaternionr& airlib_quat);
        geometry_msgs::msg::Vector3 atorVecLocal(const msr::airlib::Vector3r& airlib_vec);
        geometry_msgs::msg::Point atorPointLocal(const msr::airlib::Vector3r& airlib_vec);
        rclcpp::Time atorTime(const uint64_t& timestamp);

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
        rclcpp::Parameter brake_axis;        
        rclcpp::Parameter auto_button;
        rclcpp::Parameter reverse_steering;
        rclcpp::Parameter reverse_throttle;
        rclcpp::Parameter reverse_brake;
        rclcpp::Parameter publish_image;
        rclcpp::Parameter scene_name;

        sensor_msgs::msg::Imu::_angular_velocity_covariance_type linear_acceleration_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type angular_velocity_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type unk_orientation_cov;
        sensor_msgs::msg::Imu::_angular_velocity_covariance_type magnetic_cov;

        std::string frame_id;                      

        std::string vehicle_name;

        int steeringAxisID;
        int throttleAxisID;
        int brakeAxisID;
        int autoButtonID;

        int reverseSteering;
        int reverseThrottle;
        int reverseBrake;

        bool connected {false};

        bool active {false};
        bool autoEnabled;
        bool publishImage;

        double lastFixPub {0.0};
        double gps_accuracy {0.02};

        // int gps_update {100};
        // int sensor_update {20};
        // int state_update {50};


        int gps_update {200};
        int sensor_update {50};
        int state_update {50};


        double lastImuStamp {0.0};
        double initPosX {0.0};
        double initPosY {0.0};
        bool initPos {false};

        double priorX {0.0};
        double priorYaw {0.0};
        double priorOdomStamp {0.0};

        msr::airlib::CarRpcLibClient* client;
        msr::airlib::CarApiBase::CarControls controls;

        tf2::Quaternion ENU_NED_Q;
        tf2::Quaternion LOCAL_AIRSIM_ROS_Q;



};
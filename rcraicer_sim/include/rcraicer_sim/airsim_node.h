#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <vector>

class AirSimNode : public rclcpp::Node
{
    public:
        AirSimNode();
        ~AirSimNode();
        
        void publishTelemetryMessages(rcraicer_msgs::msg::WheelSpeed wsMsg, rcraicer_msgs::msg::ChassisState csMsg, sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::NavSatFix fixMsg);

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::WheelSpeed>::SharedPtr wsPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisState>::SharedPtr csPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::SimState>::SharedPtr ssPublisher;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fixPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
       

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisCommand>::SharedPtr cmdSubscription;

        rclcpp::TimerBase::SharedPtr connectTimer;
        rclcpp::TimerBase::SharedPtr sensorTimer;
        rclcpp::TimerBase::SharedPtr stateTimer;
        rclcpp::TimerBase::SharedPtr gpsTimer;

        void connectTimerCallback();                  

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
        rclcpp::Parameter auto_button;
        rclcpp::Parameter reverse_steering;
        rclcpp::Parameter reverse_throttle;
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
        int autoButtonID;

        int reverseSteering;
        int reverseThrottle;

        bool connected {false};

        bool active {false};
        bool autoEnabled;
        bool publishImage;

        double lastFixPub {0.0};
        double gps_accuracy {0.02};

        int gps_update {100};
        int sensor_update {10};
        int state_update {50};

        msr::airlib::CarRpcLibClient* client;
        msr::airlib::CarApiBase::CarControls controls;

};
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <vector>
#include <math.h>

#include "rcraicer_msgs/msg/encoder.hpp"
#include "rcraicer_msgs/msg/arduino_status.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"


class WheelOdometry : public rclcpp::Node 
{
    public:
        WheelOdometry();
        ~WheelOdometry();

    private:
        void state_callback(const rcraicer_msgs::msg::ChassisState::SharedPtr msg);
        void encoder_callback(const rcraicer_msgs::msg::Encoder::SharedPtr msg);
        void param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent);

        void update_internal_params();


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;                
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;        
        rclcpp::Subscription<rcraicer_msgs::msg::ChassisState>::SharedPtr stateSubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::Encoder>::SharedPtr encSubscription;
        rclcpp::AsyncParametersClient::SharedPtr parameterClient;        

        rclcpp::Parameter vehicle_wheelbase_param;
        rclcpp::Parameter vehicle_width_param;
        rclcpp::Parameter time_delay_param;
        rclcpp::Parameter wheel_radius_param;

        double length;
        double width;
        double time_delay;
        double wheel_radius;
        double mpt; // metres per tick 

        int32_t prev_enc_fl;
        int32_t prev_enc_fr;
        int32_t prev_enc_lr;
        int32_t prev_enc_rr;

        double speed_FL_;   ///< Speed of the front left wheel in m/s
        double speed_FR_;   ///< Speed of the front right wheel in m/s
        double speed_BL_;   ///< Speed of the back left wheel in m/s
        double speed_BR_;   ///< Speed of the back right wheel in m/s
        double avg_speed_;  ///< Average speed of the front two wheels in m/s
        
        double servo_val_;

        double prev_;  ///< Timestamp of previous message

        double x_;      ///< X position of vehicle relative to initial pose
        double y_;      ///< Y position of vehicle relative to initial pose
        double theta_;  ///< Heading of vehicle relative to initial pose

        double delta_x_;      ///< X velocity in local frame in m/s
        double delta_y_;      ///< Y velocity in local frame in m/s
        double delta_theta_;  ///< Yaw rate in local frame in rad/s
        double yaw_angle_;    ///< Heading in global frame from state estimator

        double delta_t_;          ///< Time difference between the two most recent wheelSpeeds messages

        double steering_angle_;  ///< Steering angle of vehicle
        double turn_radius_;     ///< Turn radius of the vehicle's immediate path
  
        double error_velocity_x_;  ///< Error value proportional to difference between estimated x velocity and true x
        /// velocity
        double error_velocity_theta_;  ///< Error value proportional to difference between estimated yaw rate and true yaw
        /// rate
        double velocity_x_var_;      ///< Variance of the difference between estimated and true x velocities
        double velocity_theta_var_;  ///< Variance of the difference between estimated and true yaw velocities

        const double PI = 3.14159265; ///< Value for pi
        const double MAX_SERVO_VAL = 0.65; ///< Maximum servo value vehicle will steer
        const double STEERING_ALPHA = -21.0832; ///< Coefficient for calculating steering angle
        const double STEERING_BETA = -0.1235; ///< Coefficient for calculating steering angle
        const double VELOCITY_X_ALPHA = 0; ///< Coefficient for calculating variance in x velocity
        const double VELOCITY_X_BETA = 0.569; ///< Coefficient for calculating variance in x velocity
        const double VELOCITY_THETA_ALPHA = -3.199; ///< Coefficient for calculating variance in yaw rate
        const double VELOCITY_THETA_BETA = -5.1233; ///< Coefficient for calculating variance in yaw rate
        const double VELOCITY_THETA_GAMMA = 3.7705; ///< Coefficient for calculating variance in yaw rate

};
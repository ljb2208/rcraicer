#include "../include/rcraicer/wheel_odometry.h"

using namespace std::chrono_literals;

WheelOdometry::WheelOdometry() : Node("wheel_odometry")
{
    prev_enc_fl = 0;
    prev_enc_fr = 0;
    prev_enc_lr = 0;
    prev_enc_rr = 0;

    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);        

    // set defaults
    length = 0.29;    
    width = 0.23;
    wheel_radius = 0.053;
    time_delay = 0.1;
    mpt = 2.0 * wheel_radius * PI / 4.0;

    this->declare_parameter("vehicle_wheelbase", length);
    this->declare_parameter("vehicle_width", width);
    this->declare_parameter("wheel_radius", wheel_radius);
    this->declare_parameter("time_delay", time_delay);

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&WheelOdometry::paramSetCallback, this, std::placeholders::_1));

    this->get_parameter_or("vehicle_wheelbase", vehicle_wheelbase_param, rclcpp::Parameter("vehicle_wheelbase", length));    
    this->get_parameter_or("vehicle_width", vehicle_width_param, rclcpp::Parameter("vehicle_width", width));    
    this->get_parameter_or("wheel_radius", wheel_radius_param, rclcpp::Parameter("wheel_radius", wheel_radius));
    this->get_parameter_or("time_delay", time_delay_param, rclcpp::Parameter("time_delay", time_delay));    
    
    update_internal_params();

    stateSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisState>(
          "chassis_state", 10, std::bind(&WheelOdometry::state_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

    encSubscription = this->create_subscription<rcraicer_msgs::msg::Encoder>(
          "encoder", 10, std::bind(&WheelOdometry::encoder_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

}

WheelOdometry::~WheelOdometry()
{
    
}

rcl_interfaces::msg::SetParametersResult WheelOdometry::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
        if (param.get_name() == "vehicle_wheelbase")
        {
            vehicle_wheelbase_param = param;
        }
        else if (param.get_name() == "vehicle_width")
        {
            vehicle_width_param = param;
        }
        else if (param.get_name() == "wheel_radius")
        {
            wheel_radius_param = param;
        }
        else if (param.get_name() == "time_delay")
        {
            time_delay_param = param;
        }
    }

    update_internal_params();

    return result;
}

void WheelOdometry::update_internal_params()
{
    if (length != vehicle_wheelbase_param.as_double())
    {
        length = vehicle_wheelbase_param.as_double();    
        RCLCPP_INFO(this->get_logger(), "Wheelbase Param changed: %d", vehicle_wheelbase_param.as_double());    
    }

    if (width != vehicle_width_param.as_double())
    {
        width = vehicle_width_param.as_double();
        RCLCPP_INFO(this->get_logger(), "Vehicle width Param changed: %d", vehicle_width_param.as_double());  
    }

    if (time_delay != time_delay_param.as_double())
    {
        time_delay = time_delay_param.as_double();
        RCLCPP_INFO(this->get_logger(), "Time delay Param changed: %d", time_delay_param.as_double());    
    }

    if (wheel_radius != wheel_radius_param.as_double())
    {
        wheel_radius = wheel_radius_param.as_double();
        RCLCPP_INFO(this->get_logger(), "Wheel radius Param changed: %d", wheel_radius_param.as_double());
        mpt = 2.0 * wheel_radius * PI / 4.0;
    }
}

void WheelOdometry::state_callback(const rcraicer_msgs::msg::ChassisState::SharedPtr msg)
{
    steering_angle_ = -msg->steer_angle; // neeed to check sign
}

void WheelOdometry::encoder_callback(const rcraicer_msgs::msg::Encoder::SharedPtr msg)
{
    int32_t fl = msg->left_front - prev_enc_fl;
    int32_t fr = msg->right_front - prev_enc_fr;
    int32_t lr = msg->left_rear - prev_enc_lr;
    int32_t rr = msg->right_rear - prev_enc_rr;
    
    speed_FL_ = fl * mpt;
    speed_FR_ = fr * mpt;
    speed_BL_ = lr * mpt;
    speed_BR_ = rr * mpt;
    avg_speed_ = (speed_FL_ + speed_FR_) / 2;

    rclcpp::Time ts = msg->header.stamp;

    if (prev_ == 0)
        delta_t_ = 0.02;
    else
        delta_t_ = ts.seconds() - prev_;
    
    prev_ = ts;

    // RCLCPP_INFO(this->get_logger(), "FL: %i FR: %i, DFL: %i DFR: %i Angle: %f delta_t: %f" , msg->left_front, msg->right_front, fl, fr, steering_angle_, delta_t_);
    

    if (std::abs(steering_angle_) < 1e-6)
    {
        delta_x_ = avg_speed_ * delta_t_;
        delta_y_ = 0;
        delta_theta_ = 0;
    }
    else
    {
        turn_radius_ = length / sin(std::abs(steering_angle_) * PI / 180.0);
        double turning_phi = avg_speed_ * delta_t_ / turn_radius_;
        delta_x_ = turn_radius_ * sin(turning_phi);
        if (steering_angle_ > 0)
        {
            delta_y_ = turn_radius_ - turn_radius_ * cos(turning_phi);
        }
        else if (steering_angle_ < 0)
        {
            delta_y_ = -(turn_radius_ - turn_radius_ * cos(turning_phi));
        }
        delta_theta_ = avg_speed_ / length * sin(steering_angle_ * PI / 180.0) * 180 / PI * delta_t_;        
    }
    
    // update x and y positions in meters
    x_ += (delta_x_ * cos(theta_ * PI / 180.0) - delta_y_ * sin(theta_ * PI / 180.0));
    y_ += (delta_x_ * sin(theta_ * PI / 180.0) + delta_y_ * cos(theta_ * PI / 180.0));

    // RCLCPP_INFO(this->get_logger(), "X: %f Y: %f, Delta_X: %f Delta_Y: %f", x_, y_, delta_x_, delta_y_);

    theta_ = fmod((theta_ + delta_theta_), 360.0);

    // Two estimations, based on left and right front wheels
    double phi_1;
    double phi_2;
    double velocity_estimate_1;
    double velocity_estimate_2;
    if (steering_angle_ != 0)
    {
        if (steering_angle_ > 0)
        {
            phi_1 = speed_FL_ / (turn_radius_ - width / 2);
            phi_2 = speed_FR_ / (turn_radius_ + width / 2);
        }
        else
        {
            phi_1 = speed_FR_ / (turn_radius_ - width / 2);
            phi_2 = speed_FL_ / (turn_radius_ + width / 2);
        }
        velocity_estimate_1 = turn_radius_ * phi_1;
        velocity_estimate_2 = turn_radius_ * phi_2;
    }
    else
    {
        velocity_estimate_1 = speed_FL_;
        velocity_estimate_2 = speed_FR_;
    }

    error_velocity_x_ = .5 * std::abs(speed_FL_ - speed_BL_) + .5 * std::abs(speed_FR_ - speed_BR_);
    error_velocity_theta_ = std::abs(velocity_estimate_1 - velocity_estimate_2);

    // these are the two error metrics published
    // velocity_x_var currently is a constant 0.0569
    velocity_x_var_ = VELOCITY_X_ALPHA * error_velocity_x_ + VELOCITY_X_BETA;
    // function for error_velocity_theta_: -0.6398 * exp(-5.1233 * error_velocity_theta_) + 0.7541
    velocity_theta_var_ = VELOCITY_THETA_ALPHA * exp(VELOCITY_THETA_BETA * error_velocity_theta_) + VELOCITY_THETA_GAMMA;
    
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->get_clock()->now();
    // the pose is relative to the header.frame_id reference published
    odom_msg.header.frame_id = "wheel_odom";
    // the twist is relative to the child_fram_id
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0;
    double yaw = theta_ * PI / 180.0;

    tf2::Quaternion q_orientation = tf2::Quaternion();
    q_orientation.setRPY(0, 0, yaw);
    odom_msg.pose.pose.orientation.x = q_orientation.x();
    odom_msg.pose.pose.orientation.y = q_orientation.y();
    odom_msg.pose.pose.orientation.z = q_orientation.z();
    odom_msg.pose.pose.orientation.w = q_orientation.w();

    // covariance matrix takes same form as above
    odom_msg.pose.covariance = {
            10000,  1e-9,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9, 10000,  1e-9,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9, 10000,  1e-9,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9, 10000,  1e-9,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9, 10000,  1e-9,
             1e-9,  1e-9,  1e-9,  1e-9,  1e-9, 10000
    };

    odom_msg.twist.twist.linear.x = delta_x_ / delta_t_;
    odom_msg.twist.twist.linear.y = 0; // assume instantaneous y velocity is 0
    odom_msg.twist.twist.linear.z = 0;

    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = delta_theta_ * PI / 180.0 / delta_t_;

    // covariance matrix takes same form as above
    odom_msg.twist.covariance = {
            velocity_x_var_,              1e-9,                1e-9,                  1e-9,                  1e-9,                1e-9,
                       1e-9, velocity_x_var_*2,                1e-9,                  1e-9,                  1e-9,                1e-9,
                       1e-9,              1e-9,   velocity_x_var_*2,                  1e-9,                  1e-9,                1e-9,
                       1e-9,              1e-9,                1e-9, velocity_theta_var_*2,                  1e-9,                1e-9,
                       1e-9,              1e-9,                1e-9,                  1e-9, velocity_theta_var_*2,                1e-9,
                       1e-9,              1e-9,                1e-9,                  1e-9,                  1e-9, velocity_theta_var_
    };

    odomPublisher->publish(odom_msg);

    prev_enc_fl = msg->left_front;
    prev_enc_fr = msg->right_front;
    prev_enc_lr = msg->left_rear;
    prev_enc_rr = msg->right_rear;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}
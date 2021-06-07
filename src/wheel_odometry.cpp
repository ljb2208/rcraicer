#include "../include/rcraicer/wheel_odometry.h"

using namespace std::chrono_literals;

WheelOdometry::WheelOdometry() : Node("wheel_odometry")
{
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);        

    // set defaults
    length = 0.29;    
    width = 0.23;
    time_delay = 0.1;    
    debug = true;

    this->declare_parameter<double>("vehicle_wheelbase", length);
    this->declare_parameter<double>("vehicle_width", width);    
    this->declare_parameter<double>("time_delay", time_delay);
    this->declare_parameter<bool>("debug", debug);

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&WheelOdometry::paramSetCallback, this, std::placeholders::_1));

    vehicle_wheelbase_param = this->get_parameter("vehicle_wheelbase");    
    vehicle_width_param = this->get_parameter("vehicle_width");        
    time_delay_param = this->get_parameter("time_delay");    
    debug_param = this->get_parameter("debug");    
    
    update_internal_params();

    stateSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisState>(
          "chassis_state", 10, std::bind(&WheelOdometry::state_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

    wsSubscription = this->create_subscription<rcraicer_msgs::msg::WheelSpeed>(
          "wheel_speeds", 10, std::bind(&WheelOdometry::wheelspeed_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

    if (debug)
    {
        markerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("odom_markers", 10);
    }    

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
        else if (param.get_name() == "time_delay")
        {
            time_delay_param = param;
        }
        else if (param.get_name() == "debug")
        {
            debug_param = param;
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

    if (debug != debug_param.as_bool())
    {
        debug = debug_param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Debug Param changed: %i", debug);    
    }    
}

void WheelOdometry::state_callback(const rcraicer_msgs::msg::ChassisState::SharedPtr msg)
{
    steering_angle_ = -msg->steer_angle; // neeed to check sign
}

void WheelOdometry::wheelspeed_callback(const rcraicer_msgs::msg::WheelSpeed::SharedPtr msg)
{    
    speed_FL_ = msg->left_front;
    speed_FR_ = msg->right_front;
    speed_BL_ = msg->left_rear;
    speed_BR_ = msg->right_rear;
    avg_speed_ = (speed_FL_ + speed_FR_) / 2;

    rclcpp::Time ts = msg->header.stamp;    

    if (prev_ == 0)
        delta_t_ = 0.02;
    else
        delta_t_ = ts.seconds() - prev_;
    
    prev_ = ts.seconds();

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

    // RCLCPP_INFO(this->get_logger(), "X: %f Y: %f Steer: %f Theta: %f Delta Theta: %f turn radius: %f Delta_X: %f Delta_Y: %f Delta_t: %f lf: %i rf: %i", x_, y_, steering_angle_, theta_, delta_theta_, turn_radius_, delta_x_, delta_y_, delta_t_, msg->left_front, msg->right_front);

    theta_ = fmod((theta_ + delta_theta_), 360.0);
    // theta_ = fmod((theta_ + delta_theta_), PI);

    // RCLCPP_INFO(this->get_logger(), "X: %f Y: %f Steer: %f Theta: %f Delta Theta: %f turn radius: %f Delta_X: %f Delta_Y: %f", x_, y_, steering_angle_, theta_, delta_theta_, turn_radius_, delta_x_, delta_y_);

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

    if (debug)
    {
        visualization_msgs::msg::Marker dMsg = visualization_msgs::msg::Marker();

        dMsg.header.frame_id = "map";
        dMsg.header.stamp = this->get_clock()->now();
        dMsg.id = markerId++;
        dMsg.type = visualization_msgs::msg::Marker::SPHERE;
        dMsg.action = visualization_msgs::msg::Marker::ADD;
        dMsg.pose = odom_msg.pose.pose;
        dMsg.scale.x = 0.1;
        dMsg.scale.y = 0.1;
        dMsg.scale.z = 0.1;
        dMsg.color.r = 1.0;
        dMsg.color.g = 0.0;
        dMsg.color.b = 0.0;
        dMsg.color.a = 1.0;

        dMsg.frame_locked = false;        

        markerPublisher->publish(dMsg);
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}
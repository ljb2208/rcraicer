#include "../include/rcraicer_sim/airsim_node.h"
#include "rpc/rpc_error.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


AirSimNode::AirSimNode() : Node("airsim_node"), autoEnabled(false), vehicle_name("PhysXCar"), client(NULL), staticBroadcaster(this), baseToOdomBroadcaster(this)
{

    ENU_NED_Q.setRPY(M_PI, 0, M_PI_2);
    LOCAL_AIRSIM_ROS_Q.setRPY(M_PI, 0, 0);

    // init parameters    
    this->declare_parameter("ip_address", "127.0.0.1");
    this->declare_parameter("port", "9091");
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("linear_acceleration_stdev", 0.0003);
    this->declare_parameter("angular_velocity_stdev",  0.02 * (M_PI / 180.0));
    this->declare_parameter("orientation_stdev", 1.0);
    this->declare_parameter("magnetic_stdev", 0.0);
    this->declare_parameter("pub_freq", 400); // hz
    this->declare_parameter("throttle_axis", 1);
    this->declare_parameter("steering_axis", 3);
    this->declare_parameter("brake_axis", 5);
    this->declare_parameter("auto_button", 0);
    this->declare_parameter("reverse_steering", true);
    this->declare_parameter("reverse_throttle", false);
    this->declare_parameter("reverse_brake", true);
    this->declare_parameter("publish_image", true);
    this->declare_parameter("scene_name", "generated_track");
        
    ipAddress = this->get_parameter("ip_address");
    port = this->get_parameter("port");
    frame_id_param = this->get_parameter("frame_id");
    linear_stdev = this->get_parameter("linear_acceleration_stdev");
    angular_stdev = this->get_parameter("angular_velocity_stdev");
    orientation_stdev = this->get_parameter("orientation_stdev");
    mag_stdev = this->get_parameter("magnetic_stdev");
    pub_freq = this->get_parameter("pub_freq");
    auto_button = this->get_parameter("auto_button");
    steering_axis = this->get_parameter("steering_axis");
    throttle_axis = this->get_parameter("throttle_axis");
    brake_axis = this->get_parameter("brake_axis");
    reverse_steering = this->get_parameter("reverse_steering");
    reverse_throttle = this->get_parameter("reverse_throttle");
    reverse_brake = this->get_parameter("reverse_brake");
    scene_name = this->get_parameter("scene_name");

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&AirSimNode::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();

    if (publishImage)
        imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);    
    magPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_mag", 10);
    wsPublisher = this->create_publisher<rcraicer_msgs::msg::WheelSpeed>("wheel_speeds", 10);
    csPublisher = this->create_publisher<rcraicer_msgs::msg::ChassisState>("chassis_state", 10);
    ssPublisher = this->create_publisher<rcraicer_msgs::msg::SimState>("sim_state", 10);
    fixPublisher = this->create_publisher<sensor_msgs::msg::NavSatFix>("rover_navsat_fix", 10);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imuFilterPublisher = this->create_publisher<rcraicer_msgs::msg::ImuFilterOutput>("imu_filter", 10);

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&AirSimNode::joy_callback, this, std::placeholders::_1));   

    cmdSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisCommand>(
      "chassis_cmds", 10, std::bind(&AirSimNode::command_callback, this, std::placeholders::_1));       
    
    connectTimerCallback();

    if (!connected)
        connectTimer = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&AirSimNode::connectTimerCallback, this));        

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

AirSimNode::~AirSimNode()
{
    if (client != NULL)
        delete client;
}

void AirSimNode::connectTimerCallback()
{    
    if (client == NULL)
    {
        try
        {
            client = new msr::airlib::CarRpcLibClient();    
        }
        catch(const std::exception& e)
        {                        
            return;
        }        
    }
    if (client != NULL && !connected)
    {                
        msr::airlib::RpcLibClientBase::ConnectionState state = msr::airlib::RpcLibClientBase::ConnectionState::Unknown;        
        
        try
        {
            state = client->getConnectionState();                                    
        }
        catch(const rpc::system_error& e)
        {                     
        }
        
        if (state == msr::airlib::RpcLibClientBase::ConnectionState::Connected)
        {
            connected = true;
            try
            {
                client->reset();
                client->enableApiControl(true);            
            }
            catch(const rpc::system_error& e)
            {                     
                return;
            }
            

            sensorTimer = this->create_wall_timer(std::chrono::milliseconds(sensor_update), std::bind(&AirSimNode::publishSensorData, this));        
            stateTimer = this->create_wall_timer(std::chrono::milliseconds(state_update), std::bind(&AirSimNode::publishStateData, this));        
            gpsTimer = this->create_wall_timer(std::chrono::milliseconds(gps_update), std::bind(&AirSimNode::publishGpsData, this));        
            RCLCPP_INFO(this->get_logger(), "Connected to sim");

            geometry_msgs::msg::TransformStamped imuTF;
            imuTF.header.frame_id = "base_link";
            imuTF.header.stamp = this->get_clock()->now();

            imuTF.child_frame_id = "imu_link";
            imuTF.transform.translation.x = 0;
            imuTF.transform.translation.y = 0;
            imuTF.transform.translation.z = 0.1;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            q.normalize();

            imuTF.transform.rotation.x = q.x();
            imuTF.transform.rotation.y = q.y();
            imuTF.transform.rotation.z = q.z();
            imuTF.transform.rotation.w = q.w();

            staticBroadcaster.sendTransform(imuTF);
        }
        else
        {            
            delete client;
            client = new msr::airlib::CarRpcLibClient();                
        }
    }    
}

void AirSimNode::publishSensorData()
{    
    publishImuData();
    publishMagData();    
}

void AirSimNode::disconnected()
{
    connected = false;
    
    if (client!= NULL)
    {
        delete client;
        client = NULL;
    }

    RCLCPP_WARN(this->get_logger(), "Disconnected from Sim");
}

void AirSimNode::publishStateData()
{
    if (!connected)
        return;
    
    rcraicer_msgs::msg::SimState state_msg;        
    msr::airlib::Environment::State env_data = client->simGetGroundTruthEnvironment();
    msr::airlib::Kinematics::State kin_data = client->simGetGroundTruthKinematics();
    msr::airlib::CarApiBase::CarState state_data = client->getCarState();
    msr::airlib::CarApiBase::CarControls control_data = client->getCarControls();    

    try
    {
        env_data = client->simGetGroundTruthEnvironment();
        kin_data = client->simGetGroundTruthKinematics();
        state_data = client->getCarState();
        control_data = client->getCarControls();    
    }
    catch(const rpc::timeout& e)
    {
        disconnected();
        return;
    }

    state_msg.header.frame_id = "base_link";
    state_msg.header.stamp = atorTime(state_data.timestamp);

    state_msg.speed = state_data.speed;
    state_msg.throttle = control_data.throttle;
    state_msg.steering = control_data.steering;
    state_msg.brake = control_data.brake;

    state_msg.env_position = atorVec(env_data.position);
    state_msg.latitude = env_data.geo_point.latitude;
    state_msg.longitude = env_data.geo_point.longitude;
    state_msg.altitude = env_data.geo_point.altitude;

    state_msg.gravity = atorVec(env_data.gravity);
    state_msg.air_pressure = env_data.air_pressure;
    state_msg.temperature = env_data.temperature;
    state_msg.air_density = env_data.air_density;

    state_msg.position = atorVec(kin_data.pose.position);
    state_msg.orientation = atorQuat(kin_data.pose.orientation);
    state_msg.linear_velocity = atorVec(kin_data.twist.linear);
    state_msg.linear_acceleration = atorVec(kin_data.accelerations.linear);
    state_msg.angular_velocity = atorVecLocal(kin_data.twist.angular);
    state_msg.angular_acceleration = atorVecLocal(kin_data.accelerations.angular);        

    ssPublisher->publish(state_msg);

    if (!initPos)
    {
        initPos = true;
        initPosX = state_msg.position.x;
        initPosY = state_msg.position.y;

        return;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "wheel_odom";
    odom_msg.header.stamp = state_msg.header.stamp;
    odom_msg.child_frame_id = "base_link";

    // odom_msg.pose.pose.position = atorPoint(state_data.kinematics_estimated.pose.position);
    // odom_msg.pose.pose.orientation = atorQuat(state_data.kinematics_estimated.pose.orientation);
    // odom_msg.twist.twist.linear = atorVec(state_data.kinematics_estimated.twist.linear);
    // odom_msg.twist.twist.angular = atorVec(state_data.kinematics_estimated.twist.angular);

    // odomPublisher->publish(odom_msg);

    odom_msg.pose.pose.position.x = state_msg.position.x - initPosX;
    odom_msg.pose.pose.position.y = state_msg.position.y - initPosY;
    // odom_msg.pose.pose.position.x = 0;
    // odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;

    double roll, pitch, yaw;

    tf2::Quaternion qq(state_msg.orientation.x, state_msg.orientation.y, state_msg.orientation.z, state_msg.orientation.w);    
    tf2::Matrix3x3 m(qq);

    m.getRPY(roll, pitch, yaw);            
    // RCLCPP_INFO(this->get_logger(), "Roll: %f Pitch: %f Yaw: %f", roll, pitch, yaw);    
    

    tf2::Quaternion q_orientation = tf2::Quaternion();
    q_orientation.setRPY(0, 0, yaw);
    // q_orientation.setRPY(0, 0, 0);
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

    double delta_x = odom_msg.pose.pose.position.x - priorX;
    double delta_yaw = yaw - priorYaw;

    rclcpp::Time t = odom_msg.header.stamp;
    double delta_t = t.seconds() - priorOdomStamp;

    // odom_msg.twist.twist.linear.x = delta_x / delta_t;
    // odom_msg.twist.twist.linear.x = state_data.speed;
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0; // assume instantaneous y velocity is 0
    odom_msg.twist.twist.linear.z = 0;

    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    // odom_msg.twist.twist.angular.z = delta_yaw / delta_t;
    odom_msg.twist.twist.angular.z = 0;

    double velocity_x_var_ = 0.6;
    double velocity_theta_var_ = 0.57;

    // covariance matrix takes same form as above
    odom_msg.twist.covariance = {
            velocity_x_var_,              1e-9,                1e-9,                  1e-9,                  1e-9,                1e-9,
                       1e-9, velocity_x_var_*2,                1e-9,                  1e-9,                  1e-9,                1e-9,
                       1e-9,              1e-9,   velocity_x_var_*2,                  1e-9,                  1e-9,                1e-9,
                       1e-9,              1e-9,                1e-9, velocity_theta_var_*2,                  1e-9,                1e-9,
                       1e-9,              1e-9,                1e-9,                  1e-9, velocity_theta_var_*2,                1e-9,
                       1e-9,              1e-9,                1e-9,                  1e-9,                  1e-9, velocity_theta_var_
    };

    priorX = odom_msg.pose.pose.position.x;
    priorYaw = yaw;
    priorOdomStamp = t.seconds();

    odomPublisher->publish(odom_msg);

    geometry_msgs::msg::TransformStamped odomTF;
    odomTF.header.frame_id = "odom";
    odomTF.header.stamp = this->get_clock()->now();
    odomTF.child_frame_id = "base_link";

    odomTF.transform.translation.x = state_msg.position.x;
    odomTF.transform.translation.y = state_msg.position.y;
    odomTF.transform.translation.z = state_msg.position.z;    

    odomTF.transform.rotation = state_msg.orientation;    

    baseToOdomBroadcaster.sendTransform(odomTF);

}


void AirSimNode::publishImuData()
{
    if (!connected)
        return;

    msr::airlib::ImuBase::Output imu_data;

    try
    {
        imu_data = client->getImuData();
    }
    catch(const rpc::timeout& e)
    {
        disconnected();
        return;
    }

    if (imu_data.time_stamp <= lastImuStamp)
        return;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = atorTime(imu_data.time_stamp);
    imu_msg.orientation = atorQuat(imu_data.orientation);
    imu_msg.angular_velocity = atorVecLocal(imu_data.angular_velocity);
    imu_msg.linear_acceleration = atorVecLocal(imu_data.linear_acceleration);

    imu_msg.orientation_covariance = orientation_cov;
    imu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_msg.linear_acceleration_covariance = linear_acceleration_cov;

    lastImuStamp = imu_data.time_stamp;
    imuPublisher->publish(imu_msg);

    rcraicer_msgs::msg::ImuFilterOutput filter_msg;
    filter_msg.header = imu_msg.header;
    filter_msg.orientation = imu_msg.orientation;
    filter_msg.bias.x = 0;
    filter_msg.bias.y = 0;
    filter_msg.bias.z = 0;

    imuFilterPublisher->publish(filter_msg);    
}

void AirSimNode::publishGpsData()
{
    if (!connected)
        return;

    msr::airlib::GpsBase::Output gps_data;
    try
    {
        gps_data = client->getGpsData();
    }
    catch(const rpc::timeout& e)
    {
        disconnected();
        return;
    }


    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.frame_id = "gps";
    gps_msg.header.stamp = atorTime(gps_data.time_stamp);

    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude;
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;    

    switch (gps_data.gnss.fix_type)
    {
        case msr::airlib::GpsBase::GnssFixType::GNSS_FIX_2D_FIX:
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            break;
        case msr::airlib::GpsBase::GnssFixType::GNSS_FIX_3D_FIX:
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
        default:
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            break;

    }
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS + sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS + sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    double cov = gps_data.gnss.epv * gps_accuracy;
    double covh =  gps_data.gnss.eph  * gps_accuracy;

    // gps_msg.position_covariance = ?
    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    gps_msg.position_covariance[0] = cov * cov;
    gps_msg.position_covariance[4] = cov * cov;
    gps_msg.position_covariance[8] = covh * covh;

    fixPublisher->publish(gps_msg);

}

void AirSimNode::publishMagData()
{
    if (!connected)
        return;

    msr::airlib::MagnetometerBase::Output mag_data;
        
    try
    {
        mag_data = client->getMagnetometerData();
    }
    catch(const rpc::timeout& e)
    {
        disconnected();
        return;
    }

    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.frame_id = "imu";
    mag_msg.header.stamp = atorTime(mag_data.time_stamp);

    // need to check the conversion?
    mag_msg.magnetic_field = atorVec(mag_data.magnetic_field_body);
        
    std::copy(std::begin(mag_data.magnetic_field_covariance),
              std::end(mag_data.magnetic_field_covariance),
              std::begin(mag_msg.magnetic_field_covariance));    

    magPublisher->publish(mag_msg);


}

/*
    Convert Global frames
    ROS x - East, y - North, z - Up
    AirSim x - North, y - East, Z - Down
*/

geometry_msgs::msg::Quaternion AirSimNode::atorQuat(const msr::airlib::Quaternionr& airlib_quat)
{
    double roll, pitch, yaw;
    // std::cout.precision(17);

    tf2::Quaternion newq(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());

    tf2::Matrix3x3 morig(newq);
    morig.getRPY(roll, pitch, yaw);

    // std::cout << "Global orig" << std::fixed << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << "\r\n";

    // tf2::Quaternion qrot = ENU_NED_Q * newq;
    // qrot.normalize();
    tf2::Quaternion qrot;
    qrot.setRPY(roll, -pitch, -yaw + M_PI_2);
    // tf2::Quaternion qrot(airlib_quat.y(), airlib_quat.x(), -airlib_quat.z(), airlib_quat.w());

    geometry_msgs::msg::Quaternion q;
    q.x = qrot.x();
    q.y = qrot.y();
    q.z = qrot.z();
    q.w = qrot.w();

    // geometry_msgs::msg::Quaternion q;
    // q.x = airlib_quat.w();
    // q.y = -airlib_quat.x();
    // q.z = -airlib_quat.y();
    // q.w = airlib_quat.z();

    // tf2::Quaternion qrot(q.x, q.y, q.x, q.w);

    // tf2::Matrix3x3 m(qrot);
    // m.getRPY(roll, pitch, yaw);

    // std::cout << "Global " << std::fixed << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << "\r\n";

    return q;
}

geometry_msgs::msg::Vector3 AirSimNode::atorVec(const msr::airlib::Vector3r& airlib_vec)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = airlib_vec.y();
    vec.y = airlib_vec.x();
    vec.z = -airlib_vec.z();

    return vec;
}

geometry_msgs::msg::Point AirSimNode::atorPoint(const msr::airlib::Vector3r& airlib_vec)
{
    geometry_msgs::msg::Point pt;
    pt.x = airlib_vec.y();
    pt.y = airlib_vec.x();
    pt.z = -airlib_vec.z();

    return pt;

}

/*
    Convert Local frames
    ROS x - Forward, y - Left, z - Up
    AirSim x - Forward, y - Right, Z - Down
*/

geometry_msgs::msg::Quaternion AirSimNode::atorQuatLocal(const msr::airlib::Quaternionr& airlib_quat)
{
    double roll, pitch, yaw;
    std::cout.precision(17);

    tf2::Quaternion newq(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());

    tf2::Matrix3x3 morig(newq);
    morig.getRPY(roll, pitch, yaw);

    // std::cout << "Local orig" << std::fixed << " roll: " << roll << " pitch: " <<  pitch << " yaw: " << yaw << "\r\n";


    

    tf2::Quaternion qrot = LOCAL_AIRSIM_ROS_Q * newq;
    qrot.normalize();

    geometry_msgs::msg::Quaternion q;
    q.x = qrot.x();
    q.y = qrot.y();
    q.z = qrot.z();
    q.w = qrot.w();
    

    tf2::Matrix3x3 m(qrot);
    m.getRPY(roll, pitch, yaw);

    // std::cout << "Local " << std::fixed << " roll: " << roll << " pitch: " <<  pitch << " yaw: " << yaw << "\r\n";

    return q;

    // geometry_msgs::msg::Quaternion newq;
    // newq.x = airlib_quat.x();
    // newq.y = -airlib_quat.y();
    // newq.z = -airlib_quat.z();
    // newq.w = airlib_quat.w();

    // return newq;
}

geometry_msgs::msg::Vector3 AirSimNode::atorVecLocal(const msr::airlib::Vector3r& airlib_vec)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = airlib_vec.x();
    vec.y = -airlib_vec.y();
    vec.z = -airlib_vec.z();

    return vec;
}

geometry_msgs::msg::Point AirSimNode::atorPointLocal(const msr::airlib::Vector3r& airlib_vec)
{
    geometry_msgs::msg::Point pt;
    pt.x = airlib_vec.x();
    pt.y = -airlib_vec.y();
    pt.z = -airlib_vec.z();

    return pt;

}

rclcpp::Time AirSimNode::atorTime(const uint64_t& timestamp)
{
    rclcpp::Time tm(timestamp);    
    return tm;
}

rcl_interfaces::msg::SetParametersResult AirSimNode::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
        if (param.get_name() == "ip_address")
        {
            ipAddress = param;
        }
        else if (param.get_name() == "port")
        {
            port = param;
        }
        else if (param.get_name() == "frame_id")
        {
            frame_id_param = param;
        }
        else if (param.get_name() == "linear_acceleration_stdev")
        {
            linear_stdev = param;
        }
        else if (param.get_name() == "angular_velocity_stdev")
        {
            angular_stdev = param;
        }        
        else if (param.get_name() == "orientation_stdev")
        {
            orientation_stdev = param;
        }
        else if (param.get_name() == "magnetic_stdev")
        {
            mag_stdev = param;
        }
        else if (param.get_name() == "pub_freq")
        {
            pub_freq = param;
        }
        else if (param.get_name() == "scene_name")
        {
            scene_name = param;
        }
    }

    updateInternalParams();

    return result;
}


void AirSimNode::updateInternalParams()
{
    throttleAxisID = throttle_axis.as_int();
    steeringAxisID = steering_axis.as_int();
    brakeAxisID = brake_axis.as_int();
    autoButtonID = auto_button.as_int();

    if (reverse_steering.as_bool())
        reverseSteering = -1;
    else
        reverseSteering = 1;

    if (reverse_throttle.as_bool())
        reverseThrottle = -1;
    else
        reverseThrottle = 1;

    if (reverse_brake.as_bool())
        reverseBrake = -1;
    else
        reverseBrake = 1;

    frame_id = frame_id_param.as_string();  
    setup_covariance(linear_acceleration_cov, linear_stdev.as_double());
    setup_covariance(angular_velocity_cov, angular_stdev.as_double());
    setup_covariance(orientation_cov, orientation_stdev.as_double());
    setup_covariance(magnetic_cov, mag_stdev.as_double());
    setup_covariance(unk_orientation_cov, 0.0);
}

 
void AirSimNode::setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev)
{
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > c(cov.data());

    c.setZero();
    if (stdev) {
        double sr = stdev * stdev;
        c.diagonal() << sr, sr, sr;
    }
    else {
        c(0,0) = -1.0;
    }
}

void AirSimNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{    
    rclcpp::Time ts = msg->header.stamp;

    if (msg->buttons[autoButtonID] == 1 && (ts.seconds() - lastAutoUpdate) >= 0.5)
    {
        autoEnabled = !autoEnabled;
        lastAutoUpdate = ts.seconds();

        if (autoEnabled)
            RCLCPP_INFO(this->get_logger(), "Auto mode enabled");
        else
            RCLCPP_INFO(this->get_logger(), "Auto mode disabled");
    }

    if (autoEnabled)
        return;

    float steer = msg->axes[steeringAxisID]  * reverseSteering;
    float throttle = msg->axes[throttleAxisID]  * reverseThrottle;
    float brake = msg->axes[brakeAxisID];

    sendControls(throttle, steer, brake);
}

void AirSimNode::command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg)
{
    if (autoEnabled)
    {
        sendControls(msg->throttle, msg->steer, 0.0);
    }
}

void AirSimNode::setActive(bool active)
{
    this->active = active;

    if (active == false)
    {
        sendControls(0.0, 0.0, 0.0);
    }
}

void AirSimNode::sendControls(float throttle, float steering, float brake)
{
    controls.throttle = throttle;
    controls.steering = steering;
    controls.handbrake = false;

    if (autoEnabled)
        controls.brake = brake;
    else
        controls.brake = (brake * reverseBrake + 1.0) / 2.0;

    if (throttle >= 0)
    {
        controls.set_throttle(throttle, true);
        controls.is_manual_gear = false;
    }
    else
    {
        controls.set_throttle(throttle, false);
        controls.is_manual_gear = true;
        controls.manual_gear = -1;
    }

    if (connected)
    {
        try
        {
            client->setCarControls(controls);    
        }
        catch(const rpc::timeout& e)
        {
            disconnected();
            return;
        }        
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirSimNode>());
    rclcpp::shutdown();
    return 0;
}
#include "../include/rcraicer_sim/airsim_node.h"
#include "rpc/rpc_error.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;


AirSimNode::AirSimNode() : Node("airsim_node"), autoEnabled(false), vehicle_name("PhysXCar"), client(NULL)
{
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
    this->declare_parameter("auto_button", 0);
    this->declare_parameter("reverse_steering", true);
    this->declare_parameter("reverse_throttle", false);
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
    reverse_steering = this->get_parameter("reverse_steering");
    reverse_throttle = this->get_parameter("reverse_throttle");
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

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&AirSimNode::joy_callback, this, std::placeholders::_1));   

    cmdSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisCommand>(
      "cmds", 10, std::bind(&AirSimNode::command_callback, this, std::placeholders::_1));       

    client = new msr::airlib::CarRpcLibClient();

    connectTimerCallback();

    if (!connected)
        connectTimer = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&AirSimNode::connectTimerCallback, this));        

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

AirSimNode::~AirSimNode()
{
    if (client != NULL)
        delete client;
}

void AirSimNode::connectTimerCallback()
{
    if (client != NULL && !connected)
    {
        if (client->getConnectionState() == msr::airlib::RpcLibClientBase::ConnectionState::Connected)
        {
            connected = true;
            client->enableApiControl(true);            

            sensorTimer = this->create_wall_timer(std::chrono::milliseconds(sensor_update), std::bind(&AirSimNode::publishSensorData, this));        
            stateTimer = this->create_wall_timer(std::chrono::milliseconds(state_update), std::bind(&AirSimNode::publishStateData, this));        
            gpsTimer = this->create_wall_timer(std::chrono::milliseconds(gps_update), std::bind(&AirSimNode::publishGpsData, this));        
            RCLCPP_INFO(this->get_logger(), "Connected to sim");
        }
    }    
}

void AirSimNode::publishSensorData()
{    
    publishImuData();
    publishMagData();    
}

void AirSimNode::publishStateData()
{
    rcraicer_msgs::msg::SimState state_msg;    
    

    msr::airlib::Environment::State env_data = client->simGetGroundTruthEnvironment();
    msr::airlib::Kinematics::State kin_data = client->simGetGroundTruthKinematics();
    msr::airlib::CarApiBase::CarState state_data = client->getCarState();
    msr::airlib::CarApiBase::CarControls control_data = client->getCarControls();

    state_msg.header.frame_id = "base_link";
    state_msg.header.stamp = atorTime(state_data.timestamp);

    state_msg.speed = state_data.speed;
    state_msg.throttle = control_data.throttle;
    state_msg.steering = control_data.steering;

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
    state_msg.angular_velocity = atorVec(kin_data.twist.angular);
    state_msg.angular_acceleration = atorVec(kin_data.accelerations.angular);

    ssPublisher->publish(state_msg);
}


void AirSimNode::publishImuData()
{
    msr::airlib::ImuBase::Output imu_data = client->getImuData();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = atorTime(imu_data.time_stamp);
    imu_msg.orientation = atorQuat(imu_data.orientation);
    imu_msg.angular_velocity = atorVec(imu_data.angular_velocity);
    imu_msg.linear_acceleration = atorVec(imu_data.linear_acceleration);

    imu_msg.orientation_covariance = orientation_cov;
    imu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_msg.linear_acceleration_covariance = linear_acceleration_cov;

    imuPublisher->publish(imu_msg);
}

void AirSimNode::publishGpsData()
{
    msr::airlib::GpsBase::Output gps_data = client->getGpsData();
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
    msr::airlib::MagnetometerBase::Output mag_data = client->getMagnetometerData();

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

geometry_msgs::msg::Quaternion AirSimNode::atorQuat(const msr::airlib::Quaternionr& airlib_quat)
{
    // tf2::Quaternion q(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
    // tf2::Matrix3x3 m(q);

    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // std::cout <<"q: " << airlib_quat.x() << ":" << airlib_quat.y() << ":" << airlib_quat.z() << ":" << airlib_quat.w() << "\r\n";
    // std::cout <<"roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << "\r\n";

    // tf2::Quaternion q2;
    // q2.setRPY(roll, -pitch, -yaw);

    // std::cout <<"q2: " << q2.x() << ":" << q2.y() << ":" << q2.z() << ":" << q2.w() << "\r\n";    


    geometry_msgs::msg::Quaternion newq;
    newq.x = airlib_quat.x();
    newq.y = -airlib_quat.y();
    newq.z = -airlib_quat.z();
    newq.w = airlib_quat.w();

    return newq;
}

geometry_msgs::msg::Vector3 AirSimNode::atorVec(const msr::airlib::Vector3r& airlib_vec)
{
    geometry_msgs::msg::Vector3 vec;
    vec.x = airlib_vec.x();
    vec.y = -airlib_vec.y();
    vec.z = -airlib_vec.z();

    return vec;
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
    autoButtonID = auto_button.as_int();

    if (reverse_steering.as_bool())
        reverseSteering = -1;
    else
        reverseSteering = 1;

    if (reverse_throttle.as_bool())
        reverseThrottle = -1;
    else
        reverseThrottle = 1;

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

void AirSimNode::publishTelemetryMessages(rcraicer_msgs::msg::WheelSpeed wsMsg, rcraicer_msgs::msg::ChassisState csMsg, sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::NavSatFix fixMsg)
{    

    wsMsg.header.stamp = this->get_clock()->now();;
    wsMsg.header.frame_id = "base_link";

    csMsg.header.stamp = this->get_clock()->now();;
    csMsg.header.frame_id = "base_link";

    imuMsg.header.stamp = this->get_clock()->now();;
    imuMsg.header.frame_id = "imu_link";
    imuMsg.orientation_covariance = orientation_cov;
    imuMsg.angular_velocity_covariance = angular_velocity_cov;
    imuMsg.linear_acceleration_covariance = linear_acceleration_cov;

    rclcpp::Time tm = this->get_clock()->now();

    if ((tm.seconds() - lastFixPub) >= 0.1)
    {
        fixMsg.header.stamp = tm;
        fixMsg.header.frame_id = "gps_link";
        fixPublisher->publish(fixMsg);
        lastFixPub = tm.seconds();
    }

    wsPublisher->publish(wsMsg);
    csPublisher->publish(csMsg);
    imuPublisher->publish(imuMsg);
    
}

void AirSimNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{    

    if (msg->buttons[autoButtonID] == 1)
    {
        autoEnabled = !autoEnabled;
    }

    if (autoEnabled)
        return;

    float steer = msg->axes[steeringAxisID];
    float throttle = msg->axes[throttleAxisID];
    float brake = 0.0;

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
    controls.throttle = throttle * reverseThrottle;
    controls.steering = steering * reverseSteering;
    controls.handbrake = false;
    controls.brake = brake;

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

    client->setCarControls(controls);    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AirSimNode>());
    rclcpp::shutdown();
    return 0;
}
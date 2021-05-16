#include "../include/rcraicer/imu_mavlink.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

IMUMavlink::IMUMavlink() : Node("imu_mavlink"), serialPort(NULL), rxDropCount(0), linear_accel_vec_flu(Eigen::Vector3d::Zero())
{    
    // init parameters    
    this->declare_parameter("serial_port", "/dev/rcIMU");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("frame_id", 115200);
    this->declare_parameter("linear_acceleration_stdev", 115200);
    this->declare_parameter("angular_velocity_stdev", 115200);
    this->declare_parameter("orientation_stdev", 115200);
    this->declare_parameter("magnetic_stdev", 115200);

    
    updateInternalParams();

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&IMUMavlink::paramSetCallback, this, std::placeholders::_1));
        

    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    imuPublisherRaw = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
    magPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_mag", 10);
    tempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("imu_temp", 10);

    serialPort = new MavlinkSerialPort(portPath.as_string(), baudRate.as_int());
    serialPort->registerDataCallback(std::bind(&IMUMavlink::serial_data_callback, this));

    if (serialPort->isConnected())
    {
        RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", portPath.as_string().c_str(), 
                                    baudRate.as_int());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error connecting on %s @ %i. Error: %s", portPath.as_string().c_str(), 
                                    baudRate.as_int(), serialPort->getErrorString().c_str());        
    }    

    RCLCPP_INFO(this->get_logger(), "Node started.");    
    
}

IMUMavlink::~IMUMavlink()
{
    if (serialPort)
        delete serialPort;
}

rcl_interfaces::msg::SetParametersResult IMUMavlink::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    this->declare_parameter("serial_port", "/dev/rcIMU");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("frame_id", 115200);
    this->declare_parameter("linear_acceleration_stdev", 115200);
    this->declare_parameter("angular_velocity_stdev", 115200);
    this->declare_parameter("orientation_stdev", 115200);
    this->declare_parameter("magnetic_stdev", 115200);


    for (auto param : parameters)
    {        
        if (param.get_name() == "serial_port")
        {
            portPath = param;
        }
        else if (param.get_name() == "baud_rate")
        {
            baudRate = param;
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
    }

    updateInternalParams();

    return result;
}

 
void IMUMavlink::setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev)
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


void IMUMavlink::publishImuData(uint32_t time_boot_ms, Eigen::Quaterniond &orientation_enu, Eigen::Vector3d &gyro_flu)
{    
    sensor_msgs::msg::Imu imu_enu_msg = sensor_msgs::msg::Imu();

    // Fill message header
    imu_enu_msg.header.frame_id = frame_id;
    imu_enu_msg.header.stamp = this->get_clock()->now();    

    imu_enu_msg.orientation.x = orientation_enu.x();
    imu_enu_msg.orientation.y = orientation_enu.y();
    imu_enu_msg.orientation.z = orientation_enu.z();
    imu_enu_msg.orientation.w = orientation_enu.w();
        
    imu_enu_msg.angular_velocity.x = gyro_flu.x();
    imu_enu_msg.angular_velocity.y = gyro_flu.y();
    imu_enu_msg.angular_velocity.z = gyro_flu.z();
        
    imu_enu_msg.linear_acceleration.x = linear_accel_vec_flu.x();
    imu_enu_msg.linear_acceleration.y = linear_accel_vec_flu.y();
    imu_enu_msg.linear_acceleration.z = linear_accel_vec_flu.z();

    // // Pass ENU msg covariances
    imu_enu_msg.orientation_covariance = orientation_cov;
    imu_enu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_enu_msg.linear_acceleration_covariance = linear_acceleration_cov;
    

    if (!received_linear_accel) {
        // Set element 0 of covariance matrix to -1 if no data received as per sensor_msgs/Imu defintion
        imu_enu_msg.linear_acceleration_covariance[0] = -1;        
    }
            
    imuPublisher->publish(imu_enu_msg);		
}

void IMUMavlink::publishMagData(uint32_t time_boot_ms, Eigen::Vector3d &mag_field)
{
      
    sensor_msgs::msg::MagneticField mag_msg = sensor_msgs::msg::MagneticField();

    // Fill message header
    mag_msg.header.frame_id = frame_id;
    mag_msg.header.stamp = this->get_clock()->now();  

    mag_msg.magnetic_field.x = mag_field.x();
    mag_msg.magnetic_field.y = mag_field.y();
    mag_msg.magnetic_field.z = mag_field.z();		
	
    mag_msg.magnetic_field_covariance = magnetic_cov;

    magPublisher->publish(mag_msg);
}

void IMUMavlink::publishImuDataRaw(uint32_t time_usec, Eigen::Vector3d &gyro_flu, 
				Eigen::Vector3d &accel_flu, Eigen::Vector3d &accel_frd)
{
    sensor_msgs::msg::Imu imu_enu_msg = sensor_msgs::msg::Imu();

    // Fill message header
    imu_enu_msg.header.frame_id = frame_id;
    imu_enu_msg.header.stamp = this->get_clock()->now();
        
    imu_enu_msg.angular_velocity.x = gyro_flu.x();
    imu_enu_msg.angular_velocity.y = gyro_flu.y();
    imu_enu_msg.angular_velocity.z = gyro_flu.z();
        
    imu_enu_msg.linear_acceleration.x = accel_flu.x();
    imu_enu_msg.linear_acceleration.y = accel_flu.y();
    imu_enu_msg.linear_acceleration.z = accel_flu.z();

    // Save readings
    linear_accel_vec_flu = accel_flu;
    linear_accel_vec_frd = accel_frd;
    received_linear_accel = true;

    // // Pass ENU msg covariances
    imu_enu_msg.orientation_covariance = unk_orientation_cov;
    imu_enu_msg.angular_velocity_covariance = angular_velocity_cov;
    imu_enu_msg.linear_acceleration_covariance = linear_acceleration_cov;
            
    imuPublisherRaw->publish(imu_enu_msg);		
}

void IMUMavlink::handleAttitudeQ(mavlink_attitude_quaternion_t attitude_q)
{
    auto ned_aircraft_orientation = Eigen::Quaterniond(attitude_q.q1, attitude_q.q2, attitude_q.q3, attitude_q.q4);
    auto gyro_frd = Eigen::Vector3d(attitude_q.rollspeed, attitude_q.pitchspeed, attitude_q.yawspeed);

    auto enu_baselink_orientation = transform_orientation(
        transform_orientation(ned_aircraft_orientation, StaticTF::NED_TO_ENU), StaticTF::AIRCRAFT_TO_BASELINK);

    auto gyro_flu = transform_static_frame(gyro_frd, StaticTF::AIRCRAFT_TO_BASELINK);

    publishImuData(attitude_q.time_boot_ms, enu_baselink_orientation, gyro_flu);
}

void IMUMavlink::handleImu(mavlink_highres_imu_t imu_hr)
{    
  	// [accel_available]
    if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
        auto gyro_flu = transform_static_frame(Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro), StaticTF::AIRCRAFT_TO_BASELINK);			
        auto accel_frd = Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc);
        auto accel_flu = transform_static_frame(accel_frd, StaticTF::AIRCRAFT_TO_BASELINK);

        publishImuDataRaw(imu_hr.time_usec, gyro_flu, accel_flu, accel_frd);
    }
	// [accel_available]

    // [mag_available]
    if (imu_hr.fields_updated & (7 << 6)) {
        auto mag_field = transform_static_frame(Eigen::Vector3d(imu_hr.xmag, imu_hr.ymag, imu_hr.zmag) * GAUSS_TO_TESLA, StaticTF::AIRCRAFT_TO_BASELINK);

        publishMagData(imu_hr.time_usec, mag_field);
    }
    // [mag_available]
		
    // [temperature_available]
    if (imu_hr.fields_updated & (1 << 12)) {
        sensor_msgs::msg::Temperature temp_msg = sensor_msgs::msg::Temperature();        

        temp_msg.header.frame_id = frame_id;
        temp_msg.header.stamp = this->get_clock()->now();
        temp_msg.temperature = imu_hr.temperature;

        tempPublisher->publish(temp_msg);
    }
    // [temperature_available]
}

Eigen::Quaterniond IMUMavlink::transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform)
{
	switch (transform) {
        case StaticTF::NED_TO_ENU:
        {
            return NED_ENU_Q * q;            
        }
        case StaticTF::ENU_TO_NED:
        {
            return NED_ENU_Q * q;
        }
        case StaticTF::AIRCRAFT_TO_BASELINK:
        {
            return q * AIRCRAFT_BASELINK_Q;
        }
        case StaticTF::BASELINK_TO_AIRCRAFT:
        {
            return q * AIRCRAFT_BASELINK_Q;
        }
        case StaticTF::ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK:
        {
            return AIRCRAFT_BASELINK_Q * q;	
        }
        case StaticTF::ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT:
        {
            return AIRCRAFT_BASELINK_Q * q;	
        }
	}

    return q;
}

Eigen::Vector3d IMUMavlink::transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform)
{
	switch (transform) {        
        case StaticTF::NED_TO_ENU:
        {
            return NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * vec);
        }
        case StaticTF::ENU_TO_NED:
        {
            return NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * vec);
        }
        case StaticTF::AIRCRAFT_TO_BASELINK:
        {
            return AIRCRAFT_BASELINK_AFFINE * vec;
        }
        case StaticTF::BASELINK_TO_AIRCRAFT:
        {
            return AIRCRAFT_BASELINK_AFFINE * vec;
        }
        case StaticTF::ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK:
        {
            return AIRCRAFT_BASELINK_AFFINE * vec;
        }
        case StaticTF::ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT:
        {
            return AIRCRAFT_BASELINK_AFFINE * vec;
        }
	}

    return vec;
}

void IMUMavlink::serial_data_callback()
{
    mavlink_message_t* msg = serialPort->getMessage();
    mavlink_status_t* status = serialPort->getStatus();


    switch(msg->msgid)
    {
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        {
            mavlink_attitude_quaternion_t attitude_q;
            mavlink_msg_attitude_quaternion_decode(msg, &attitude_q);

            handleAttitudeQ(attitude_q);
            break;

        }        
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            mavlink_highres_imu_t imu;
            mavlink_msg_highres_imu_decode(msg, &imu);

            handleImu(imu);                        
            break;
        }

    }

    if (status->packet_rx_drop_count > rxDropCount)
    {
        std::cout << "Packet dropped\n";
        rxDropCount = status->packet_rx_drop_count;
    }    
}


void IMUMavlink::updateInternalParams()
{
    frame_id = frame_id_param.as_string();  
    setup_covariance(linear_acceleration_cov, linear_stdev.as_double());
    setup_covariance(angular_velocity_cov, angular_stdev.as_double());
    setup_covariance(orientation_cov, orientation_stdev.as_double());
    setup_covariance(magnetic_cov, mag_stdev.as_double());
    setup_covariance(unk_orientation_cov, 0.0);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUMavlink>());
    rclcpp::shutdown();
    return 0;
}
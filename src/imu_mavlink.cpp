#include "../include/rcraicer/imu_mavlink.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

IMUMavlink::IMUMavlink() : Node("imu_mavlink"), serialPort(NULL), rxDropCount(0), linear_accel_vec_flu(Eigen::Vector3d::Zero())
{    
    // init parameters    
    this->get_parameter_or("serial_port", portPath, rclcpp::Parameter("serial_port", "/dev/ttyACM0"));        
    this->get_parameter_or("baud_rate", baudRate, rclcpp::Parameter("baud_rate", 115200));
    this->get_parameter_or("frame_id", frame_id_param, rclcpp::Parameter("frame_id", "base_link"));

    this->get_parameter_or("linear_acceleration_stdev", linear_stdev, rclcpp::Parameter("linear_acceleration_stdev", 0.0003));
    this->get_parameter_or("angular_velocity_stdev", angular_stdev, rclcpp::Parameter("angular_velocity_stdev",  0.02 * (M_PI / 180.0)));
    this->get_parameter_or("orientation_stdev", orientation_stdev, rclcpp::Parameter("orientation_stdev",  1.0));
    this->get_parameter_or("magnetic_stdev", mag_stdev, rclcpp::Parameter("magnetic_stdev",  0.0));

    updateInternalParams();        
    
    parameterClient = std::make_shared<rclcpp::AsyncParametersClient>(this);
    paramSubscription = parameterClient->on_parameter_event(std::bind(&IMUMavlink::param_callback, this, std::placeholders::_1));

    while (!parameterClient->wait_for_service(10s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service.");
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Parameter service available.");

    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu");
    imuPublisherRaw = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw");
    magPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_mag");
    tempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("imu_temp");

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
 

void IMUMavlink::param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent)
{
    for (auto & param : paramEvent->changed_parameters)
    {
        if (param.name == "frame_id")
        {
            frame_id_param.from_parameter_msg(param);            
        }
        if (param.name == "serial_port")
        {
            portPath.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Serial Port Param changed: %s", portPath.as_string().c_str());    
        }        

        if (param.name == "linear_acceleration_stdev")
        {
            linear_stdev.from_parameter_msg(param);            
        }

        if (param.name == "angular_velocity_stdev")
        {
            angular_stdev.from_parameter_msg(param);            
        }

        if (param.name == "orientation_stdev")
        {
            orientation_stdev.from_parameter_msg(param);            
        }

        if (param.name == "magnetic_stdev")
        {
            mag_stdev.from_parameter_msg(param);            
        }
    }

    updateInternalParams();
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
    std::shared_ptr<sensor_msgs::msg::Imu> imu_enu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    // Fill message header
    imu_enu_msg->header.frame_id = frame_id;
    imu_enu_msg->header.stamp = rclcpp::Time(time_boot_ms);    

    imu_enu_msg->orientation.x = orientation_enu.x();
    imu_enu_msg->orientation.y = orientation_enu.y();
    imu_enu_msg->orientation.z = orientation_enu.z();
    imu_enu_msg->orientation.w = orientation_enu.w();
        
    imu_enu_msg->angular_velocity.x = gyro_flu.x();
    imu_enu_msg->angular_velocity.y = gyro_flu.y();
    imu_enu_msg->angular_velocity.z = gyro_flu.z();
        
    imu_enu_msg->linear_acceleration.x = linear_accel_vec_flu.x();
    imu_enu_msg->linear_acceleration.y = linear_accel_vec_flu.y();
    imu_enu_msg->linear_acceleration.z = linear_accel_vec_flu.z();

    // // Pass ENU msg covariances
    imu_enu_msg->orientation_covariance = orientation_cov;
    imu_enu_msg->angular_velocity_covariance = angular_velocity_cov;
    imu_enu_msg->linear_acceleration_covariance = linear_acceleration_cov;
    

    if (!received_linear_accel) {
        // Set element 0 of covariance matrix to -1 if no data received as per sensor_msgs/Imu defintion
        imu_enu_msg->linear_acceleration_covariance[0] = -1;        
    }
            
    imuPublisher->publish(imu_enu_msg);		
}

void IMUMavlink::publishMagData(uint32_t time_boot_ms, Eigen::Vector3d &mag_field)
{
      
    std::shared_ptr<sensor_msgs::msg::MagneticField> mag_msg = std::make_shared<sensor_msgs::msg::MagneticField>();

    // Fill message header
    mag_msg->header.frame_id = frame_id;
    mag_msg->header.stamp = rclcpp::Time(time_boot_ms);  

    mag_msg->magnetic_field.x = mag_field.x();
    mag_msg->magnetic_field.y = mag_field.y();
    mag_msg->magnetic_field.z = mag_field.z();		
	
    mag_msg->magnetic_field_covariance = magnetic_cov;

    magPublisher->publish(mag_msg);
}

void IMUMavlink::publishImuDataRaw(uint32_t time_usec, Eigen::Vector3d &gyro_flu, 
				Eigen::Vector3d &accel_flu, Eigen::Vector3d &accel_frd)
{
    std::shared_ptr<sensor_msgs::msg::Imu> imu_enu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    // Fill message header
    imu_enu_msg->header.frame_id = frame_id;
    imu_enu_msg->header.stamp = rclcpp::Time(time_usec);    
        
    imu_enu_msg->angular_velocity.x = gyro_flu.x();
    imu_enu_msg->angular_velocity.y = gyro_flu.y();
    imu_enu_msg->angular_velocity.z = gyro_flu.z();
        
    imu_enu_msg->linear_acceleration.x = accel_flu.x();
    imu_enu_msg->linear_acceleration.y = accel_flu.y();
    imu_enu_msg->linear_acceleration.z = accel_flu.z();

    // Save readings
    linear_accel_vec_flu = accel_flu;
    linear_accel_vec_frd = accel_frd;
    received_linear_accel = true;

    // // Pass ENU msg covariances
    imu_enu_msg->orientation_covariance = unk_orientation_cov;
    imu_enu_msg->angular_velocity_covariance = angular_velocity_cov;
    imu_enu_msg->linear_acceleration_covariance = linear_acceleration_cov;
            
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

		// /** Check if static pressure sensor data is available:
		//  *  @snippet src/plugins/imu.cpp static_pressure_available
		//  */
		// // [static_pressure_available]
		// if (imu_hr.fields_updated & (1 << 9)) {
		// 	auto static_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

		// 	static_pressure_msg->header = header;
		// 	static_pressure_msg->fluid_pressure = imu_hr.abs_pressure;

		// 	static_press_pub.publish(static_pressure_msg);
		// }
		// // [static_pressure_available]

		// /** Check if differential pressure sensor data is available:
		//  *  @snippet src/plugins/imu.cpp differential_pressure_available
		//  */
		// // [differential_pressure_available]
		// if (imu_hr.fields_updated & (1 << 10)) {
		// 	auto differential_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

		// 	differential_pressure_msg->header = header;
		// 	differential_pressure_msg->fluid_pressure = imu_hr.diff_pressure;

		// 	diff_press_pub.publish(differential_pressure_msg);
		// }
		// // [differential_pressure_available]

		// /** Check if temperature data is available:
		//  *  @snippet src/plugins/imu.cpp temperature_available
		//  */
    // [temperature_available]
    if (imu_hr.fields_updated & (1 << 12)) {
        std::shared_ptr<sensor_msgs::msg::Temperature> temp_msg = std::make_shared<sensor_msgs::msg::Temperature>();        

        temp_msg->header.frame_id = frame_id;
        temp_msg->header.stamp = rclcpp::Time(imu_hr.time_usec);    
        temp_msg->temperature = imu_hr.temperature;

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
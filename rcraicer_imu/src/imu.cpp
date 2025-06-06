#include "../include/rcraicer_imu/imu.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


Y3Imu::Y3Imu() : Node("imu")
{
    // init parameters    
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("linear_acceleration_stdev", 0.0003);
    this->declare_parameter("angular_velocity_stdev",  0.02 * (M_PI / 180.0));
    this->declare_parameter("orientation_stdev", 1.0);
    this->declare_parameter("magnetic_stdev", 0.0);
    this->declare_parameter("pub_freq", 400); // hz
        
    portPath = this->get_parameter("serial_port");
    baudRate = this->get_parameter("baud_rate");
    frame_id_param = this->get_parameter("frame_id");
    linear_stdev = this->get_parameter("linear_acceleration_stdev");
    angular_stdev = this->get_parameter("angular_velocity_stdev");
    orientation_stdev = this->get_parameter("orientation_stdev");
    mag_stdev = this->get_parameter("magnetic_stdev");
    pub_freq = this->get_parameter("pub_freq");

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Y3Imu::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();

    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);    
    magPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_mag", 10);
    tempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("imu_temp", 10);

    proto = new Y3Protocol(pub_freq.as_int());
    proto->registerSensorMessagesCallback(std::bind(&Y3Imu::publishSensorMessages, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    if (proto->openPort(portPath.as_string(), baudRate.as_int()))
    {
        RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", portPath.as_string().c_str(), 
                                    baudRate.as_int());
    }

    proto->configure();
    

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

Y3Imu::~Y3Imu()
{
    if (proto != NULL)
        delete proto;
}

rcl_interfaces::msg::SetParametersResult Y3Imu::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

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
        else if (param.get_name() == "pub_freq")
        {
            pub_freq = param;
        }
    }

    updateInternalParams();

    return result;
}


void Y3Imu::updateInternalParams()
{
    frame_id = frame_id_param.as_string();  
    setup_covariance(linear_acceleration_cov, linear_stdev.as_double());
    setup_covariance(angular_velocity_cov, angular_stdev.as_double());
    setup_covariance(orientation_cov, orientation_stdev.as_double());
    setup_covariance(magnetic_cov, mag_stdev.as_double());
    setup_covariance(unk_orientation_cov, 0.0);
}

 
void Y3Imu::setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev)
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

void Y3Imu::publishSensorMessages(sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::MagneticField magMsg, sensor_msgs::msg::Temperature tempMsg)
{
    rclcpp::Time tm = this->get_clock()->now();    
    imuMsg.header.stamp = tm;
    imuMsg.header.frame_id = frame_id;

    magMsg.header.stamp = tm;
    magMsg.header.frame_id = frame_id;

    tempMsg.header.stamp = tm;
    tempMsg.header.frame_id = frame_id;

    imuPublisher->publish(imuMsg);    
    magPublisher->publish(magMsg);
    tempPublisher->publish(tempMsg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Y3Imu>());
    rclcpp::shutdown();
    return 0;
}
#include "../include/rcraicer_sim/sim_node.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


SimNode::SimNode() : Node("sim_node"), server(NULL)
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
        
    ipAddress = this->get_parameter("ip_address");
    port = this->get_parameter("port");
    frame_id_param = this->get_parameter("frame_id");
    linear_stdev = this->get_parameter("linear_acceleration_stdev");
    angular_stdev = this->get_parameter("angular_velocity_stdev");
    orientation_stdev = this->get_parameter("orientation_stdev");
    mag_stdev = this->get_parameter("magnetic_stdev");
    pub_freq = this->get_parameter("pub_freq");

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&SimNode::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();

    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);    
    magPublisher = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_mag", 10);
    wsPublisher = this->create_publisher<rcraicer_msgs::msg::WheelSpeed>("wheel_speeds", 10);
    csPublisher = this->create_publisher<rcraicer_msgs::msg::ChassisState>("chassis_state", 10);
    fixPublisher = this->create_publisher<sensor_msgs::msg::NavSatFix>("rover_gps_fix", 10);
    
    // server->registerSensorMessagesCallback(std::bind(&SimNode::publishSensorMessages, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // if (server->openPort(portPath.as_string(), baudRate.as_int()))
    // {
    //     RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", portPath.as_string().c_str(), 
    //                                 baudRate.as_int());
    // }    

    server = new TcpServer(ipAddress.as_string(), port.as_string());

    server->registerTelemetryCallback(std::bind(&SimNode::publishTelemetryMessages, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    if (server->connectToSocket())
    {
        RCLCPP_INFO(this->get_logger(), "Connected to %s:%s.", ipAddress.as_string().c_str(), port.as_string().c_str());    
    }
    else
    {
        connectTimer = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&SimNode::connectTimerCallback, this));
    }
    

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

SimNode::~SimNode()
{
    if (server != NULL)
        delete server;
}

void SimNode::connectTimerCallback()
{
    if (!server->isConnected())
    {
        if (server->connectToSocket())
        {
            RCLCPP_INFO(this->get_logger(), "Connected to %s:%s.", ipAddress.as_string().c_str(), port.as_string().c_str());    
        }
    }
}

rcl_interfaces::msg::SetParametersResult SimNode::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
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
    }

    updateInternalParams();

    return result;
}


void SimNode::updateInternalParams()
{
    frame_id = frame_id_param.as_string();  
    setup_covariance(linear_acceleration_cov, linear_stdev.as_double());
    setup_covariance(angular_velocity_cov, angular_stdev.as_double());
    setup_covariance(orientation_cov, orientation_stdev.as_double());
    setup_covariance(magnetic_cov, mag_stdev.as_double());
    setup_covariance(unk_orientation_cov, 0.0);
}

 
void SimNode::setup_covariance(sensor_msgs::msg::Imu::_angular_velocity_covariance_type &cov, double stdev)
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

void SimNode::publishTelemetryMessages(rcraicer_msgs::msg::WheelSpeed wsMsg, rcraicer_msgs::msg::ChassisState csMsg, sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::NavSatFix fixMsg)
{
    rclcpp::Time tm = this->get_clock()->now();

    wsMsg.header.stamp = tm;
    wsMsg.header.frame_id = "base_link";

    csMsg.header.stamp = tm;
    csMsg.header.frame_id = "base_link";

    imuMsg.header.stamp = tm;
    imuMsg.header.frame_id = "imu_link";

    fixMsg.header.stamp = tm;
    fixMsg.header.frame_id = "gps_link";

    wsPublisher->publish(wsMsg);
    csPublisher->publish(csMsg);
    imuPublisher->publish(imuMsg);
    fixPublisher->publish(fixMsg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimNode>());
    rclcpp::shutdown();
    return 0;
}
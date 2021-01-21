#include "../include/rcraicer/arduino_controller.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

ArduinoController::ArduinoController() : Node("arduino_controller"), serialPort(NULL), published(false)
{
    // init parameters    
    this->get_parameter_or("serial_port", portPath, rclcpp::Parameter("serial_port", "/dev/ttyUSB0"));    
    this->get_parameter_or("steering_axis", steeringAxis, rclcpp::Parameter("steering_axis", 0));
    this->get_parameter_or("throttle_axis", throttleAxis, rclcpp::Parameter("throttleAxis", 1));
    this->get_parameter_or("baud_rate", baudRate, rclcpp::Parameter("baud_rate", 115200));

    std::vector<int64_t> defaultServoPoints = {1000, 1500, 2000};
    this->get_parameter_or("steering_servo_points", steeringServoPoints, rclcpp::Parameter("steering_servo_points", defaultServoPoints));
    this->get_parameter_or("throttle_servo_points", throttleServoPoints, rclcpp::Parameter("throttle_servo_points", defaultServoPoints));

    updateInternalParams();
    
    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", std::bind(&ArduinoController::joy_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

    parameterClient = std::make_shared<rclcpp::AsyncParametersClient>(this);
    paramSubscription = parameterClient->on_parameter_event(std::bind(&ArduinoController::param_callback, this, std::placeholders::_1));

    while (!parameterClient->wait_for_service(10s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service.");
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Parameter service available.");

    serialPort = new SerialPort(portPath.as_string(), baudRate.as_int(), MESSAGE_DELIM);
    serialPort->registerDataCallback(std::bind(&ArduinoController::serial_data_callback, this));

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
    

    RCLCPP_INFO(this->get_logger(), "Node started. Steering Axis: %i. Throttle Axis: %i",
                                    steeringAxis.as_int(), throttleAxis.as_int());    
}

ArduinoController::~ArduinoController()
{
    if (serialPort)
        delete serialPort;
}
 
void ArduinoController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{
    data_msg dmsg;     
    servo_msg smsg;
    smsg.steer = getSteeringPWM(msg->axes[steeringAxisID]);
    smsg.throttle = getThrottlePWM(msg->axes[throttleAxisID]);    
    
    packServoMessage(smsg, dmsg);

    if (!published)
    {
        writeData(dmsg);
        published = true;
    }    
}

void ArduinoController::param_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr paramEvent)
{
    for (auto & param : paramEvent->changed_parameters)
    {
        if (param.name == "serial_port")
        {
            portPath.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Serial Port Param changed: %s", portPath.as_string().c_str());    
        }
        else if (param.name == "steering_axis")
        {
            steeringAxis.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Steering Axis Param changed: %i", steeringAxis.as_int());    
        }
        else if (param.name == "throttle_axis")
        {
            throttleAxis.from_parameter_msg(param);
            RCLCPP_INFO(this->get_logger(), "Throttle Axis Param changed: %i", throttleAxis.as_int());    
        }
        else if (param.name == "steering_servo_points")
        {
            if (param.value.integer_array_value.size() != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot set steering servo points param. Wrong number of items specified");
            }
            else
            {
                steeringServoPoints.from_parameter_msg(param);
            }            
        }
        else if (param.name == "throttle_servo_points")
        {
            if (param.value.integer_array_value.size() != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot set throttle servo points param. Wrong number of items specified");
            }
            else
            {
                throttleServoPoints.from_parameter_msg(param);
            }            
        }
    }

    updateInternalParams();
}

void ArduinoController::serial_data_callback()
{
    unsigned char data[MSG_SIZE];
    int length = 0;

    while (serialPort->getNextMessage(data, length))
    {
        // process message
        processMessage(data, length);        
        length = 0;
    }
}

void ArduinoController::processMessage(unsigned char* data, int length)
{
    if (length != sizeof(data_msg))
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid serial message length: %i", length);
        return;
    }

    data_msg msg;
    memcpy(&msg, data, length);

    // check crc
    if (!validateCRC(msg))
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid CRC. Ignoring message");
        return;
    }

    switch(msg.msg_type)
    {
        case ENCODER_MSG:
            RCLCPP_INFO(this->get_logger(), "received encoder message");
            encoder_msg enc_msg;
            unpackEncoderMessage(msg, enc_msg);
            RCLCPP_INFO(this->get_logger(), "Values: %i %i %i %i", enc_msg.left_rear, enc_msg.left_front, enc_msg.right_front, enc_msg.right_rear);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "received unknown serial message");
            break;
    }
}

void ArduinoController::updateInternalParams()
{
    steeringServoMin = steeringServoPoints.as_integer_array()[0];
    steeringServoMid = steeringServoPoints.as_integer_array()[0];
    steeringServoMax = steeringServoPoints.as_integer_array()[0];

    throttleServoMin = throttleServoPoints.as_integer_array()[0];
    throttleServoMid = throttleServoPoints.as_integer_array()[0];
    throttleServoMax = throttleServoPoints.as_integer_array()[0];

    steeringAxisID = steeringAxis.as_int();
    throttleAxisID = throttleAxis.as_int();
}

int32_t ArduinoController::getSteeringPWM(float value)
{
    return getPWM(value, steeringServoMin, steeringServoMid, steeringServoMax);    
}

int32_t ArduinoController::getThrottlePWM(float value)
{
    return getPWM(value, throttleServoMin, throttleServoMid, throttleServoMax);
}

int32_t ArduinoController::getPWM(float value, int64_t min, int64_t mid, int64_t max)
{
    int32_t pwm = 1500;
    int64_t range = (max - min)/2;
    pwm = value * range + mid;

    return pwm;
}

void ArduinoController::writeData(data_msg dmsg)
{
    if (!serialPort->isConnected())
        return;
    
    unsigned char data[MSG_SIZE + 2];
    memcpy(data, &dmsg, MSG_SIZE);
    data[MSG_SIZE] = MESSAGE_DELIM;
    data[MSG_SIZE+1] = MESSAGE_DELIM;

    volatile int writeCount = serialPort->writePort((const unsigned char*) &data, MSG_SIZE+2);

    if (writeCount != (int)(MSG_SIZE+2))
    {
        RCLCPP_ERROR(this->get_logger(), "Error occurred writing data to serial port. %i", writeCount);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoController>());
    rclcpp::shutdown();
    return 0;
}
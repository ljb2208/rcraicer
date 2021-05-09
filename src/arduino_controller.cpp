#include "../include/rcraicer/arduino_controller.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

ArduinoController::ArduinoController() : Node("arduino_controller"), serialPort(NULL), isArmed(false), armButtonValue(0), invalidCRC(0), unknownMsg(0)
{   
    // declare parameters 
    this->declare_parameter<std::string>("serial_port", "/dev/rcArduino");
    this->declare_parameter<int>("baud_rate", 230400);
    this->declare_parameter<int>("steering_axis", 3);
    this->declare_parameter<int>("throttle_axis", 1);
    this->declare_parameter<int>("arm_button", 0);

    this->declare_parameter("steering_input_factor", -1.0);
    this->declare_parameter("throttle_input_factor", 1.0);    

    std::vector<int64_t> defaultServoPoints = {1200, 1500, 1800};
    this->declare_parameter<std::vector<int64_t>>("steering_servo_points", defaultServoPoints);
    this->declare_parameter<std::vector<int64_t>>("throttle_servo_points", defaultServoPoints);
    this->declare_parameter<double>("steering_degrees_per_tick", 0.10834);

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&ArduinoController::paramSetCallback, this, std::placeholders::_1));


    // init parameters    
    portPath = this->get_parameter("serial_port"); 
    baudRate = this->get_parameter("baud_rate");

    steeringAxis = this->get_parameter("steering_axis");
    throttleAxis = this->get_parameter("throttle_axis");
    armButton = this->get_parameter("arm_button");

    steeringInputFactorParam = this->get_parameter("steering_input_factor");
    throttleInputFactorParam = this->get_parameter("throttle_input_factor");

    steeringServoPoints = this->get_parameter("steering_servo_points");
    throttleServoPoints = this->get_parameter("throttle_servo_points");

    steeringDegreesPerTick = this->get_parameter("steering_degrees_per_tick");
    
    updateInternalParams();

    encPublisher = this->create_publisher<rcraicer_msgs::msg::Encoder>("encoder", 10);
    statusPublisher = this->create_publisher<rcraicer_msgs::msg::ArduinoStatus>("arduino_status", 10);
    statePublisher = this->create_publisher<rcraicer_msgs::msg::ChassisState>("chassis_state", 10);
    
    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ArduinoController::joy_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       

    commandSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisCommand>(
      "cmds", 10, std::bind(&ArduinoController::command_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       
    
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

    RCLCPP_INFO(this->get_logger(), "Node started.");                                        
    
}

ArduinoController::~ArduinoController()
{
    if (serialPort)
        delete serialPort;
}

void ArduinoController::command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg)
{
    sendActuatorData(msg->throttle, msg->steer);    
}
 
void ArduinoController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{    
    float steer = msg->axes[steeringAxisID];
    float throttle = msg->axes[throttleAxisID];

    sendActuatorData(throttle, steer);    

    if (msg->buttons[armButtonID] != armButtonValue)
    {
        if (msg->buttons[armButtonID] == 1)
        {
            isArmed = !isArmed;
            data_msg dmsg;
            command_msg cmsg;
            cmsg.armed = isArmed;

            if (cmsg.armed)
            {
                RCLCPP_INFO(this->get_logger(), "Arming.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Disarming.");
            }
            
            packCommandMessage(cmsg, dmsg);
            writeData(dmsg);
        }

        armButtonValue = msg->buttons[armButtonID];
    }    
}

void ArduinoController::publishChassisState(float throttle, float steer, float steerAngle)
{
    rcraicer_msgs::msg::ChassisState state_msg = rcraicer_msgs::msg::ChassisState();
    state_msg.header.stamp = rclcpp::Node::now();
    state_msg.armed = isArmed;
    state_msg.throttle = throttle;
    state_msg.steer = steer;
    state_msg.steer_angle = steerAngle;

    statePublisher->publish(state_msg);
}

void ArduinoController::sendActuatorData(float throttle, float steer)
{         
    data_msg dmsg;
    servo_msg smsg;
    int steeringPWM = getSteeringPWM(steer);
    smsg.steer = steeringPWM;
    smsg.throttle = getThrottlePWM(throttle);        
    
    packServoMessage(smsg, dmsg);            
    writeData(dmsg);            

    float steerAngle = getSteeringAngle(steeringPWM);
    publishChassisState(throttle, steer, steerAngle);
}

rcl_interfaces::msg::SetParametersResult ArduinoController::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
        if (param.get_name() == "arm_button")
        {
            armButton = param;
        }
        else if (param.get_name() == "serial_port")
        {
            portPath = param;
        }
        else if (param.get_name() == "baud_rate")
        {
            baudRate = param;
        }
        else if (param.get_name() == "steering_axis")
        {
            steeringAxis = param;
        }
        else if (param.get_name() == "throttle_axis")
        {
            throttleAxis = param;
        }        
        else if (param.get_name() == "steering_servo_points")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY || param.as_integer_array().size() != 3)
            {
                result.successful = false;
                result.reason = "Wrong number of items specified, must be 3 (min, mid, max)";                
            }
            else
            {
                steeringServoPoints = param;
            }
            
        }
        else if (param.get_name() == "throttle_servo_points")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY || param.as_integer_array().size() != 3)
            {
                result.successful = false;
                result.reason = "Wrong number of items specified, must be 3 (min, mid, max)";
            }
            else
            {
                throttleServoPoints = param;
            }            
        }        
        else if (param.get_name() == "steering_degress_per_tick")
        {            
            steeringDegreesPerTick = param;                  
        }
        else if (param.get_name() == "steering_input_factor")
        {
            steeringInputFactorParam = param;
        }
        else if (param.get_name() == "throttle_input_factor")
        {
            throttleInputFactorParam = param;
        }
    }

    updateInternalParams();
    return result;
}

void ArduinoController::serial_data_callback()
{
    unsigned char data[MSG_SIZE];
    int length = 0;

    while (serialPort->getNextMessage(data, MSG_SIZE, length))
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
        if (invalidCRC >= 65535)
        {
            invalidCRC = 0;
        }

        invalidCRC++;

        RCLCPP_ERROR(this->get_logger(), "Invalid CRC. Ignoring message of type %i", msg.msg_type);
        return;
    }

    switch(msg.msg_type)
    {
        case ENCODER_MSG:
        {
            // RCLCPP_INFO(this->get_logger(), "received encoder message");
            encoder_msg enc_msg;
            unpackEncoderMessage(msg, enc_msg);

            rcraicer_msgs::msg::Encoder enc;
            enc.header.stamp = rclcpp::Node::now();
            enc.left_rear = enc_msg.left_rear;
            enc.left_front = enc_msg.left_front;
            enc.right_front = enc_msg.right_front;
            enc.right_rear = enc_msg.right_rear;

            encPublisher->publish(enc);

            // RCLCPP_INFO(this->get_logger(), "Values: %i %i %i %i", enc_msg.left_rear, enc_msg.left_front, enc_msg.right_front, enc_msg.right_rear);
            break;
        }
        case ARDUINO_STATUS_MSG:
        {
            arduino_status_msg smsg;
            unpackArduinoStatusMessage(msg, smsg);
            rcraicer_msgs::msg::ArduinoStatus status;
            status.header.stamp = rclcpp::Node::now();
            status.servo_update_count = smsg.servo_update_count;
            status.encoder_msg_count = smsg.encoder_msg_count;
            status.main_loop_count = smsg.main_loop_count;
            status.main_loop_max = smsg.main_loop_max;            
            status.armed = smsg.armed;
            status.status = smsg.status;
            status.invalid_crc_arduino = smsg.crc_error;
            status.unknown_msg_arduino = smsg.unknown_msg;
            status.invalid_crc = invalidCRC;
            status.unknown_msg = unknownMsg;

            statusPublisher->publish(status);
            break;

        }
        default:
        {
            if (unknownMsg >= 65535)
            {
                unknownMsg = 0;
            }

            unknownMsg++;
            RCLCPP_ERROR(this->get_logger(), "received unknown serial message");
            break;
        }
    }
}

void ArduinoController::updateInternalParams()
{
    if (steeringAngleCoefficient != steeringDegreesPerTick.as_double())
    {
        steeringAngleCoefficient = steeringDegreesPerTick.as_double();
        RCLCPP_INFO(this->get_logger(), "Steering degrees per tick: %f", steeringAngleCoefficient);
    }

    if (steeringServoMin != steeringServoPoints.as_integer_array()[0] || steeringServoMid != steeringServoPoints.as_integer_array()[1] || steeringServoMax != steeringServoPoints.as_integer_array()[2])    
    {
        steeringServoMin = steeringServoPoints.as_integer_array()[0];
        steeringServoMid = steeringServoPoints.as_integer_array()[1];
        steeringServoMax = steeringServoPoints.as_integer_array()[2];
        RCLCPP_INFO(this->get_logger(), "Steering servo points: %i/%i/%i", steeringServoMin, steeringServoMid, steeringServoMax);
    }

    if (throttleServoMin != throttleServoPoints.as_integer_array()[0] || throttleServoMid != throttleServoPoints.as_integer_array()[1] || throttleServoMax != throttleServoPoints.as_integer_array()[2])    
    {
        throttleServoMin = throttleServoPoints.as_integer_array()[0];
        throttleServoMid = throttleServoPoints.as_integer_array()[1];
        throttleServoMax = throttleServoPoints.as_integer_array()[2];
        RCLCPP_INFO(this->get_logger(), "Throttle servo points: %i/%i/%i", steeringServoMin, steeringServoMid, steeringServoMax);
    }       

    if (steeringInputFactor != steeringInputFactorParam.as_double())
    {
        steeringInputFactor = steeringInputFactorParam.as_double();
        RCLCPP_INFO(this->get_logger(), "Steering input factor: %f", steeringInputFactor);
    }

    if (throttleInputFactor != throttleInputFactorParam.as_double())
    {
        throttleInputFactor = throttleInputFactorParam.as_double();
        RCLCPP_INFO(this->get_logger(), "Throttle input factor: %f", throttleInputFactor);
    }    

    if (steeringAxisID != steeringAxis.as_int())
    {
        steeringAxisID = steeringAxis.as_int();
        RCLCPP_INFO(this->get_logger(), "Steering Axis: %i", steeringAxisID);
    }

    if (throttleAxisID != throttleAxis.as_int())
    {
        throttleAxisID = throttleAxis.as_int();    
        RCLCPP_INFO(this->get_logger(), "Throttle Axis: %i", throttleAxisID);
    }

    if (armButtonID != armButton.as_int())
    {
        armButtonID = armButton.as_int();
        RCLCPP_INFO(this->get_logger(), "Arm Button: %i", armButtonID);
    }    
}

float ArduinoController::getSteeringAngle(int steerPWM)
{
    int pwmValue = steerPWM - steeringServoMid;    
    return pwmValue * steeringAngleCoefficient;
}

int32_t ArduinoController::getSteeringPWM(float value)
{
    return getPWM(value, steeringInputFactor, steeringServoMin, steeringServoMid, steeringServoMax);    
}

int32_t ArduinoController::getThrottlePWM(float value)
{    
    return getPWM(value, throttleInputFactor, throttleServoMin, throttleServoMid, throttleServoMax);
}

int32_t ArduinoController::getPWM(float value, float inputFactor, int64_t min, int64_t mid, int64_t max)
{
    int32_t pwm = 1500;
    int64_t range = (max - min)/2;
    pwm = (value * inputFactor * range) + mid;

    return pwm;
}

void ArduinoController::writeData(data_msg dmsg)
{
    if (!serialPort->isConnected())
        return;
    
    unsigned char data[MSG_SIZE_WITH_DELIM];
    memcpy(data, &dmsg, MSG_SIZE);
    data[MSG_SIZE] = MESSAGE_DELIM;
    data[MSG_SIZE+1] = MESSAGE_DELIM;    

    if (serialPort->writePort((const unsigned char*) &data, MSG_SIZE_WITH_DELIM) != (int)(MSG_SIZE_WITH_DELIM))
    {
        RCLCPP_ERROR(this->get_logger(), "Error occurred writing data to serial port");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoController>());
    rclcpp::shutdown();
    return 0;
}
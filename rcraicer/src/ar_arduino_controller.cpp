#include "../include/rcraicer/ar_arduino_controller.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

ARArduinoController::ARArduinoController() : Node("arduino_controller"), serialPort(NULL), isArmed(false), armButtonValue(0), invalidCRC(0), unknownMsg(0), cmdReceived(false)
{   
    escDataFailCounter_ = 0;

    // declare parameters 
    this->declare_parameter<std::string>("serial_port", "/dev/arChassis");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("wheel_diameter", 0.190);

    this->declare_parameter("steering_reverse", false);
    this->declare_parameter("throttle_reverse", false);    
    this->declare_parameter("brake_reverse", false);    

    this->declare_parameter("command_rate", 100);    
    this->declare_parameter("chassis_command_max_age", 0.2);    
    this->declare_parameter("runstop_command_max_age", 2.0);    

    this->declare_parameter<std::string>("chassis_command_priorities", "mppi_controller;constantSpeedController;waypointFollower;joystick;OCS;RC");

    std::vector<int64_t> defaultServoPoints = {1200, 1500, 1800};
    this->declare_parameter<std::vector<int64_t>>("steering_servo_points", defaultServoPoints);
    this->declare_parameter<std::vector<int64_t>>("throttle_servo_points", defaultServoPoints);
    this->declare_parameter<std::vector<int64_t>>("brake_servo_points", defaultServoPoints);
    this->declare_parameter<double>("steering_degrees_per_tick", 0.10834);

    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&ARArduinoController::paramSetCallback, this, std::placeholders::_1));

    actuatorConfig_["throttle"] = ActuatorConfig();
    actuatorConfig_["steering"] = ActuatorConfig();
    actuatorConfig_["frontBrake"] = ActuatorConfig();

    // init parameters    
    portPath = this->get_parameter("serial_port"); 
    baudRate = this->get_parameter("baud_rate");

    steeringReverseParam = this->get_parameter("steering_reverse");
    throttleReverseParam = this->get_parameter("throttle_reverse");
    brakeReverseParam = this->get_parameter("brake_reverse");

    steeringServoPoints = this->get_parameter("steering_servo_points");
    throttleServoPoints = this->get_parameter("throttle_servo_points");
    brakeServoPoints = this->get_parameter("brake_servo_points");

    steeringDegreesPerTick = this->get_parameter("steering_degrees_per_tick");
    wheelDiameterParam = this->get_parameter("wheel_diameter");

    commandRateParam = this->get_parameter("command_rate");
    chassisCommandMaxAgeParam = this->get_parameter("chassis_command_max_age");
    runstopMaxAgeParam = this->get_parameter("runstop_command_max_age");

    chassisCommandPrioritiesPraram = this->get_parameter("chassis_command_priorities");
    
    updateInternalParams();

    wsPublisher = this->create_publisher<rcraicer_msgs::msg::WheelSpeed>("wheel_speeds", 10);        
    statePublisher = this->create_publisher<rcraicer_msgs::msg::ChassisState>("chassis_state", 10);
    cmdPublisher = this->create_publisher<rcraicer_msgs::msg::ChassisCommand>("chassis_cmd", 10);
    
    for (auto& mapIt : chassisCommands_)
    {
        std::string topic = mapIt.first + "/chassisCommand";
        
        RCLCPP_INFO(this->get_logger(), "Topic: %s", topic.c_str());

        rclcpp::Subscription<rcraicer_msgs::msg::ChassisCommand>::SharedPtr sub;        
        sub = this->create_subscription<rcraicer_msgs::msg::ChassisCommand>(topic, 10,  std::bind(&ARArduinoController::command_callback, this, std::placeholders::_1));

        chassisCommandSub_[mapIt.first] = sub;

        RCLCPP_INFO(this->get_logger(), "Subscribed");
    }    

    runstopSubscription = this->create_subscription<rcraicer_msgs::msg::RunStop>(
      "runstop", 10, std::bind(&ARArduinoController::runstop_callback, this, std::placeholders::_1));   // add queue size in later versions of ros2       
    
    serialPort = new SerialPort(portPath.as_string(), baudRate.as_int(), '#', 1);
    serialPort->registerDataCallback(std::bind(&ARArduinoController::serial_data_callback, this));

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

    // create comand timer
    chassisControlTimer = this->create_wall_timer(std::chrono::milliseconds(1000 / commandRate) , std::bind(&ARArduinoController::setChassisActuators, this));

    RCLCPP_INFO(this->get_logger(), "Node started.");                                        
    
}

ARArduinoController::~ARArduinoController()
{
    if (serialPort)
        delete serialPort;
}

void ARArduinoController::command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg)
{
    std::map<std::string, rcraicer_msgs::msg::ChassisCommand>::iterator mapIt;
    if((mapIt = chassisCommands_.find(msg->sender)) == chassisCommands_.end())
    {
        RCLCPP_ERROR(this->get_logger(), "AutoRallyChassis: Unknown controller " +
                            msg->sender + 
                            " attempting to control chassis, please add entry " +
                            " to chassisCommandPriorities.yaml");
    }
    {
        // if (mapIt->first == "joystick")
        //   RCLCPP_INFO(this->get_logger(), "ChassisCommand");

        mapIt->second = *msg;
    }
}

void ARArduinoController::runstop_callback(const rcraicer_msgs::msg::RunStop::SharedPtr msg)
{
    runstops_[msg->sender] = *msg;
}
 
void ARArduinoController::publishChassisState(float throttle, float steer, float steerAngle)
{
    rcraicer_msgs::msg::ChassisState state_msg = rcraicer_msgs::msg::ChassisState();
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.armed = isArmed;
    state_msg.throttle = throttle;
    state_msg.steer = steer;
    state_msg.steer_angle = steerAngle;

    statePublisher->publish(state_msg);
}

rcl_interfaces::msg::SetParametersResult ARArduinoController::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
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
        else if (param.get_name() == "brake_servo_points")
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY || param.as_integer_array().size() != 3)
            {
                result.successful = false;
                result.reason = "Wrong number of items specified, must be 3 (min, mid, max)";
            }
            else
            {
                brakeServoPoints = param;
            }            
        }        
        else if (param.get_name() == "steering_degress_per_tick")
        {            
            steeringDegreesPerTick = param;                  
        }
        else if (param.get_name() == "steering_reverse")
        {
            steeringReverseParam = param;
        }
        else if (param.get_name() == "throttle_reverse")
        {
            throttleReverseParam = param;
        }
        else if (param.get_name() == "brake_reverse")
        {
            brakeReverseParam = param;
        }
        else if (param.get_name() == "chassis_command_max_age")
        {
            chassisCommandMaxAgeParam = param;
        }
        else if (param.get_name() == "command_rate")
        {
            commandRateParam = param;
        }
        else if (param.get_name() == "runstop_command_max_age")
        {
            runstopMaxAgeParam = param;
        }        
        else if (param.get_name() == "chassis_command_priorities")
        {
            chassisCommandPrioritiesPraram = param;
        }
    }

    updateInternalParams();
    return result;
}

void ARArduinoController::serial_data_callback()
{
    //Any variables accessed in here and in other places in the code need to be mutex'd as this fires in a different
  //thread than the main thread. This is also a pretyt long callback, and ROS can shutdown underneath us, so check
  //if ROS system is still running any time anytseparatedhing ROS is used

  std::string data = "";
  size_t startPosition = 0;
  size_t endPosition = 0;

  //parse all messages from chassis
  do
  {
    serialPort->lock();
    //look for start and end of message
    startPosition = serialPort->m_data.find_first_of('#');
    endPosition = serialPort->m_data.find_first_of('\n', startPosition);

    //pull message out of buffer if start and end are found
    if(startPosition != std::string::npos && endPosition != std::string::npos)
    {
      //frame data if not framed
      if(startPosition != 0)
      {
        serialPort->m_data.erase(0, startPosition);
      }
      startPosition = 0;
      endPosition = serialPort->m_data.find_first_of('\n', startPosition);

      //cut out and erase mescDataFailCounter_essage from queue
      data = serialPort->m_data.substr(0, endPosition);
      serialPort->m_data.erase(0, endPosition);
    }
    serialPort->unlock();

    //process a complete messsage from the chassis with start ('#'), message type, and end ('\n') characters removed
    if(!data.empty())
    {
      processChassisMessage(data.substr(1,1), data.substr(2, endPosition-2));
      data.erase();
    }
  } while(endPosition != std::string::npos); //look for another message if we haven't looked at all data available yet
}


void ARArduinoController::updateInternalParams()
{
    if (wheelDiameter != wheelDiameterParam.as_double())
    {
        wheelDiameter = wheelDiameterParam.as_double();
        RCLCPP_INFO(this->get_logger(), "Wheel Diameter: %f", wheelDiameter);
    }

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

        actuatorConfig_["steering"].min = steeringServoPoints.as_integer_array()[0];
        actuatorConfig_["steering"].center = steeringServoPoints.as_integer_array()[1];
        actuatorConfig_["steering"].max = steeringServoPoints.as_integer_array()[2];
        actuatorConfig_["steering"].reverse = steeringReverseParam.as_bool();        

        RCLCPP_INFO(this->get_logger(), "Steering servo points: %i/%i/%i", steeringServoMin, steeringServoMid, steeringServoMax);
    }

    if (throttleServoMin != throttleServoPoints.as_integer_array()[0] || throttleServoMid != throttleServoPoints.as_integer_array()[1] || throttleServoMax != throttleServoPoints.as_integer_array()[2])    
    {
        throttleServoMin = throttleServoPoints.as_integer_array()[0];
        throttleServoMid = throttleServoPoints.as_integer_array()[1];
        throttleServoMax = throttleServoPoints.as_integer_array()[2];

        actuatorConfig_["throttle"].min = throttleServoPoints.as_integer_array()[0];
        actuatorConfig_["throttle"].center = throttleServoPoints.as_integer_array()[1];
        actuatorConfig_["throttle"].max = throttleServoPoints.as_integer_array()[2];
        actuatorConfig_["throttle"].reverse = throttleReverseParam.as_bool();        

        RCLCPP_INFO(this->get_logger(), "Throttle servo points: %i/%i/%i", throttleServoMin, throttleServoMid, throttleServoMax);
    }       

    if (throttleServoMin != throttleServoPoints.as_integer_array()[0] || throttleServoMid != throttleServoPoints.as_integer_array()[1] || throttleServoMax != throttleServoPoints.as_integer_array()[2])    
    {
        brakeServoMin = brakeServoPoints.as_integer_array()[0];
        brakeServoMid = brakeServoPoints.as_integer_array()[1];
        brakeServoMax = brakeServoPoints.as_integer_array()[2];

        actuatorConfig_["frontBrake"].min = brakeServoPoints.as_integer_array()[0];
        actuatorConfig_["frontBrake"].center = brakeServoPoints.as_integer_array()[1];
        actuatorConfig_["frontBrake"].max = brakeServoPoints.as_integer_array()[2];
        actuatorConfig_["frontBrake"].reverse = brakeServoPoints.as_bool();        

        RCLCPP_INFO(this->get_logger(), "Brake servo points: %i/%i/%i", brakeServoMin, brakeServoMid, brakeServoMax);
    }           

    if (commandRate != commandRateParam.as_int())
    {
        commandRate = commandRateParam.as_int();
        RCLCPP_INFO(this->get_logger(), "Command rate: %i", commandRate);
    }

    if (runstopMaxAge != runstopMaxAgeParam.as_double())
    {
        runstopMaxAge = runstopMaxAgeParam.as_double();
        RCLCPP_INFO(this->get_logger(), "Runstop Max Age: %f", runstopMaxAge);
    }

    if (commandMaxAge != chassisCommandMaxAgeParam.as_double())
    {
        commandMaxAge = chassisCommandMaxAgeParam.as_double();
        RCLCPP_INFO(this->get_logger(), "Chassis Command Max Age: %f", commandMaxAge);
    }

    loadChassisCommandPriorities();
}

float ARArduinoController::getSteeringAngle(int steerPWM)
{
    int pwmValue = steerPWM - steeringServoMid;    
    return pwmValue * steeringAngleCoefficient;
}

int32_t ARArduinoController::getSteeringPWM(float value)
{
    return getPWM(value, steeringInputFactor, steeringServoMin, steeringServoMid, steeringServoMax);    
}

int32_t ARArduinoController::getThrottlePWM(float value)
{    
    return getPWM(value, throttleInputFactor, throttleServoMin, throttleServoMid, throttleServoMax);
}

int32_t ARArduinoController::getPWM(float value, float inputFactor, int64_t min, int64_t mid, int64_t max)
{
    int32_t pwm = 1500;
    int64_t range = (max - min)/2;
    pwm = (value * inputFactor * range) + mid;

    return pwm;
}

void ARArduinoController::writeData(data_msg dmsg)
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

void ARArduinoController::processChassisMessage(std::string msgType, std::string msg)
{
    switch(msgType[0])
  {
    //wheel speeds data as comma separated doubles, units in m/s
    case 'w':
    {
      std::vector<std::string> data;
      tokenize(msg, ',', data);
      
      if(data.size() == 4)
      {
        rcraicer_msgs::msg::WheelSpeed ws_msg = rcraicer_msgs::msg::WheelSpeed();
        
        try
        {
            // Convert from rotations per second to m/s
            ws_msg.left_front = std::stod(data[0])*wheelDiameter*PI;
            ws_msg.right_front = std::stod(data[1])*wheelDiameter*PI;
            ws_msg.left_rear = std::stod(data[2])*wheelDiameter*PI;
            ws_msg.right_rear = std::stod(data[3])*wheelDiameter*PI;
          
            ws_msg.header.stamp = this->get_clock()->now();            
            wsPublisher->publish(ws_msg);
          
            // serialPort_.tick("wheelSpeeds data");
        } catch (std::exception& e )
        {
            RCLCPP_WARN(this->get_logger(), "Converting wheel speeds data failed");          
        }
      } else
      {
          RCLCPP_WARN(this->get_logger(), "Processing wheel speeds data failed");
            //serialPort_.diag_warn("Processing wheel speeds data failed");
      }
      break;
    }
    
    //Actuator controls from RC input, as comma separated us pulse width, currentl frontBrake is not controlled by RC
    case 'r':
    {
      std::vector<std::string> data;
      tokenize(msg, ',', data);
      
      if(data.size() == 4)
      {
          rcraicer_msgs::msg::ChassisCommand chassisCommand = rcraicer_msgs::msg::ChassisCommand();        
          try
            {
            //std::cout << std::stoi(data[0]) << " " << std::stoi(data[1]) << std::endl;
            chassisCommand.steer = actuatorUsToCmd(std::stoi(data[0]), "steering");
            chassisCommand.throttle = actuatorUsToCmd(std::stoi(data[1]), "throttle");
            chassisCommand.front_brake = 0.0; // setting to zero to make sure brake is neutral
            chassisCommand.sender = "RC";
            //this line is in here for compatibility with the old servoInterface
            chassisCommand.header.frame_id = "RC";
            chassisCommand.header.stamp = this->get_clock()->now();            
            
            rcMutex_.lock();
            mostRecentRc_["frontBrake"] = chassisCommand.front_brake;
            rcMutex_.unlock();
            
            cmdPublisher->publish(chassisCommand);            

            chassisEnableMutex_.lock();
            if(std::stoi(data[2]) > 1500)
            {
                autonomousEnabled_ = true;
            } else
            {
                autonomousEnabled_ = false;
            }
            throttleRelayEnabled_ = std::stoi(data[3]); //this value can be 0 or 1
            chassisEnableMutex_.unlock();

            //serialPort_.tick("RC data");
            } catch(std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Converting chassis cmd data failed");          
            }

      } else
      {
          RCLCPP_WARN(this->get_logger(), "Processing chassis cmd data failed");        
      }
      
      break;
    }
    //Castle Link ESC data, stored as 9, 2-byte shorts in message
    case 'c':
    {
      unsigned int tmp;
      double val;
      double divisor = 2042.0;
      
      if (msg.length() != 18)
      {
        escDataFailCounter_++;
        //Expected 18 bytes of ESC data, instead received " + std::to_string(msg.length()));
      }

      //std::cout << msg.length() << std::endl;
      for(size_t i = 0; i < msg.length()/2; i++)
      {
        tmp = ((((unsigned int)msg[2*i])&0xFF)<<8) + (((unsigned int)msg[2*i+1])&0xFF);
        
        //std::cout << escRegisterData_[i].second << " " << escRegisterData_[i].first << " " <<
        //             (((unsigned int)(msg[2*i]&0xFF))<<8) << " " << (int)(msg[2*i+1]&0xFF) << std::endl;
        
        val = (((double)tmp)/divisor)*escRegisterData_[i].second;
        
        // serialPort_.diag(escRegisterData_[i].first, std::to_string(val));
      }

      if (escDataFailCounter_ > 0)
        RCLCPP_WARN(this->get_logger(), "ESC data incorrect msg size counter" + std::to_string(escDataFailCounter_));      
      break;
    }    
    //error message as an ASCII string
    case 'e':
    {
        RCLCPP_WARN(this->get_logger(), "Error message: " + msg);
    //   serialPort_.tick("Error message");
    //   serialPort_.diag_error(msg);
        break;
    }

    default:
    {
        RCLCPP_WARN(this->get_logger(), "Unknown message type received from chassis:" + msgType);
        break;
    }
  }
}

double ARArduinoController::actuatorUsToCmd(int pulseWidth, std::string actuator)
{
  double cmd = std::numeric_limits<double>::quiet_NaN();
  //convert PWM pulse width in us back to actuator command message using the actuator config
    
  //if us value is outside normal servo ranges, complain and don't try to convert
  if(pulseWidth < 900 || pulseWidth > 2100)
  {
    if(invalidActuatorPulses_[actuator].first == true)
    {               
      //we've gone 2 cycles without a valid reading, disable RC control of this actuator
      cmd = -5.0;
    } else
    {
      //if we only get one invalid pulse width in a row, just use the previous one
      cmd = mostRecentRc_[actuator];
      //only increment invalid pulses when we get one in a row, not continuously

      if (invalidActuatorPulses_[actuator].second == 2)
      {
        RCLCPP_WARN(this->get_logger(), "Received multiple pulse widths out of valid range 900-2100ms in a row (" +
          std::to_string(pulseWidth) + ") from " + actuator );
      }
      
      invalidActuatorPulses_[actuator].second++;
    }
    invalidActuatorPulses_[actuator].first = true;
      
  } 
  else
  {
    invalidActuatorPulses_[actuator].first = false;

    int val = pulseWidth-actuatorConfig_[actuator].center;
    if(val < 0)
    {
      cmd = val/((double)actuatorConfig_[actuator].center-actuatorConfig_[actuator].min);
    } else
    {
      cmd = val/((double)actuatorConfig_[actuator].max-actuatorConfig_[actuator].center);
    }

    //save most recent valid actuator command    
    mostRecentRc_[actuator] = cmd;
  }
  // RCLCPP_WARN(this->get_logger(), actuator+ " single invalid pulse count %i", pulseWidth, 
  //                  std::to_string(invalidActuatorPulses_[actuator].second));
  return cmd;
}

void ARArduinoController::setChassisActuators()
{  
  rcraicer_msgs::msg::ChassisState chassisState = rcraicer_msgs::msg::ChassisState();
  
  chassisState.steering_commander = "";
  chassisState.steer = 0.0;

  chassisState.throttle_commander = "";
  chassisState.throttle = 0.0;

  chassisState.front_brake_commander = "";
  chassisState.front_brake = 0.0;

   rclcpp::Time currentTime = this->get_clock()->now();  

  //check if motion is enabled (all runstop message runstopMotionEnabled = true)
  if(runstops_.empty())
  {
    chassisState.runstop_motion_enabled = false;
  } else
  {    
    chassisState.runstop_motion_enabled = true;
    int validRunstopCount = 0;
    for(auto& runstop : runstops_)
    {
      rclcpp::Time rsTime = runstop.second.header.stamp;   

      if((currentTime.seconds()-rsTime.seconds()) < runstopMaxAge)
      {
        ++validRunstopCount;

        if(runstop.second.motion_enabled == 0)
        {        
          chassisState.runstop_motion_enabled = false;
          chassisState.throttle_commander = "runstop";
        }
      }
    }
    if(validRunstopCount == 0)
    {      
      chassisState.runstop_motion_enabled = false;
      chassisState.throttle_commander = "runstop";
      chassisState.throttle = 0.0;
    }
  }

  //find highest priority (lowest valuemostRecentRc_) command message for each actuator across all valid actuator commands
  for(auto & vecIt : chassisCommandPriorities_)
  {
    
    rclcpp::Time ccTime = chassisCommands_[vecIt.id].header.stamp;    

    if((currentTime.seconds()-ccTime.seconds()) < commandMaxAge)
    {
      
      //valid throttle commands are on [-1,1], only set throttle value if runstop is enabled
      if(chassisState.throttle_commander.empty() && chassisState.runstop_motion_enabled &&
         chassisCommands_[vecIt.id].throttle <= 1.0 &&
         chassisCommands_[vecIt.id].throttle >= -1.0)
      {
        chassisState.throttle_commander = chassisCommands_[vecIt.id].sender;
        chassisState.throttle = chassisCommands_[vecIt.id].throttle;
      }

      //valid steeringBrake commands are on [-1,1]
      if(chassisState.steering_commander.empty() &&
         chassisCommands_[vecIt.id].steer <= 1.0 &&
         chassisCommands_[vecIt.id].steer >= -1.0)
      {
        chassisState.steering_commander = chassisCommands_[vecIt.id].sender;
        chassisState.steer = chassisCommands_[vecIt.id].steer;
      }

      //valid frontBrake commands are on [0,1]
      if(chassisState.front_brake_commander.empty() &&
         chassisCommands_[vecIt.id].front_brake <= 1.0 &&
         chassisCommands_[vecIt.id].front_brake >= 0.0)
      {
        chassisState.front_brake_commander = chassisCommands_[vecIt.id].sender;
        chassisState.front_brake = chassisCommands_[vecIt.id].front_brake;
      }

    }
  }

  //send actuator commands down to chassis, sets to calibrated neutral if no valid commander
  sendCommandToChassis(chassisState);

  chassisEnableMutex_.lock();
  chassisState.throttle_relay_enabled = throttleRelayEnabled_;
  chassisState.autonomous_enabled = autonomousEnabled_;
  chassisEnableMutex_.unlock();

  //send diagnostic info about who is in control of each actuator
  if(chassisState.autonomous_enabled)
  {
    //if we're in autonomous mode, set all the information apppropriately 
    // serialPort_.diag("throttle commander", chassisState->throttleCommander);
    // serialPort_.diag("steering commander", chassisState->steeringCommander);
    // serialPort_.diag("frontBrake commander", chassisState->frontBrakeCommander);
  } else
  {
    //if we're in manual mode, send the most recentl RC command received from the chassis
    
    // serialPort_.diag("throttle commander", "RC - manual");
    // serialPort_.diag("steering commander", "RC - manual");
    // serialPort_.diag("frontBrake commander", "RC - manual");
    rcMutex_.lock();
    chassisState.throttle = mostRecentRc_["throttle"];
    chassisState.throttle_commander = "RC - manual";
    chassisState.steer = mostRecentRc_["steering"];
    chassisState.steering_commander = "RC - manual";
    chassisState.front_brake = mostRecentRc_["frontBrake"];
    chassisState.front_brake_commander = "RC - manual";
    rcMutex_.unlock();
  }

  //publish state message  
    chassisState.header.stamp = this->get_clock()->now();  
    chassisState.header.frame_id = "AutoRallyChassis";
    statePublisher->publish(chassisState);
  
//   serialPort_.tick("chassisState pub");
}

void ARArduinoController::sendCommandToChassis(rcraicer_msgs::msg::ChassisState state)
{
    /*
   * The message the chassis expects is 9 bytes:
   * byte 0: '#', the start delimiter
   * byte 1: 's', specifies message type
   * byte 2-3: steering pulse width in us packed into a short
   * byte 4-5: throttle pulse width in us packed into a short
   * byte 6-7: frontBrake pulse width in us packed into a short
   * byte 8: '\n', indicated end of message
   */

    //assemble send command message for chassis
    char actuatorCmd[9];
    short val;
    
    actuatorCmd[0] = '#';
    actuatorCmd[1] = 's';
    //steering
    val = actuatorCmdToMs(state.steer, "steering");
    actuatorCmd[2] = (char)((val&0xFF00)>>8);
    actuatorCmd[3] = (char)(val&0x00FF);

    //throttle
    val = actuatorCmdToMs(state.throttle, "throttle");
    actuatorCmd[4] = (char)((val&0xFF00)>>8);
    actuatorCmd[5] = (char)(val&0x00FF);

    //frontBrake
    val = actuatorCmdToMs(state.front_brake, "frontBrake");
    actuatorCmd[6] = (char)((val&0xFF00)>>8);
    actuatorCmd[7] = (char)(val&0x00FF);

    //message end signal
    actuatorCmd[8] = '\n';

    serialPort->writePort(actuatorCmd);
}

short ARArduinoController::actuatorCmdToMs(double actuatorValue, std::string actuator)
{
  //convert actuator command message to raw PWM pulse width in us using the actuator config

  //don't need to check if actuatorValue is on [-1, 1] because it was already done
  short val = actuatorConfig_[actuator].center;
  if(actuatorConfig_[actuator].reverse)
  {
    if(actuator == "frontBrake")
    {
      // flip range but still keep it in [0,1]
      actuatorValue = 1.0-actuatorValue;
    } else
    {
      // flip entire range for throttle and steering
      actuatorValue = -actuatorValue;
    }
  }
  if(actuatorValue < 0)
  {
    val += (short)((actuatorConfig_[actuator].center-actuatorConfig_[actuator].min)*actuatorValue);
  } else if(actuatorValue >= 0)
  {
    val += (short)((actuatorConfig_[actuator].max-actuatorConfig_[actuator].center)*actuatorValue);
  }
  return val;
}

void ARArduinoController::loadChassisCommandPriorities()
{
    std::vector<std::string> cmdPriorities;
    tokenize(chassisCommandPrioritiesPraram.as_string(), ';', cmdPriorities);

    std::vector<std::string>::iterator it;

    chassisCommandPriorities_.clear();

    int priority = 1;

    for (it=cmdPriorities.begin(); it != cmdPriorities.end(); it++)
    {
        priorityEntry toAdd;
        toAdd.id = *it;
        toAdd.priority = priority++;
        chassisCommandPriorities_.push_back(toAdd);
        chassisCommands_[toAdd.id] = rcraicer_msgs::msg::ChassisCommand();        
    }

    //sort the loaded commanders according to their priority
    std::sort(chassisCommandPriorities_.begin(),
                chassisCommandPriorities_.end(),
                priorityComparator());

    std::vector<priorityEntry>::const_iterator vecIt;
    for(vecIt = chassisCommandPriorities_.begin();
        vecIt != chassisCommandPriorities_.end();
        vecIt++)
    {
        RCLCPP_INFO(this->get_logger(), "loaded commander %s with prioirity %i", vecIt->id.c_str(), vecIt->priority);        
    }
        
    RCLCPP_INFO(this->get_logger(), "Updated Chassis command priorities with %i entries", priority - 1);
}


 
void ARArduinoController::tokenize(std::string const &str, const char delim,
            std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;
 
    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ARArduinoController>());
    rclcpp::shutdown();
    return 0;
}
#include "../include/rcraicer/runstop.h"

using namespace std::chrono_literals;

RunStop::RunStop() : Node("runstop")
{    
    this->declare_parameter<std::string>("serial_port", "/dev/arRunStop");
    this->declare_parameter<int>("runstop_rate", 5);
    this->declare_parameter<int>("baud_rate", 57600);
    
    port_param = this->get_parameter("serial_port");    
    baud_rate_param = this->get_parameter("baud_rate");    
    rate_param = this->get_parameter("runstop_rate");        

    rsPublisher = this->create_publisher<rcraicer_msgs::msg::RunStop>("runstop", 10);      

    serialPort = new SerialPort(port_param.as_string(), baud_rate_param.as_int(), 0, 1);    

     if (serialPort->isConnected())
    {
        RCLCPP_INFO(this->get_logger(), "Connected on %s @ %i", port_param.as_string().c_str(), 
                                    baud_rate_param.as_int());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error connecting on %s @ %i. Error: %s", port_param.as_string().c_str(), 
                                    baud_rate_param.as_int(), serialPort->getErrorString().c_str());        
    }    

        // create comand timer
    rsControlTimer = this->create_wall_timer(std::chrono::milliseconds(1000 / rate_param.as_int()) , 
            std::bind(&RunStop::doWorkTimerCallback, this));
}

RunStop::~RunStop()
{
    
}

bool RunStop::processData()
{
    //frame data, and parse if an entire message is waiting
    int startPosition = serialPort->m_data.find("#");
    if(startPosition > 0)
    {
        serialPort->m_data.erase(0, startPosition);
    }
    startPosition = serialPort->m_data.find("#");

    size_t endPosition = serialPort->m_data.find("\r\n");
    //std::cout << startPosition << " " << endPosition << std::endl;
    if(startPosition == 0 && endPosition!=std::string::npos)
    {
        lastMessageTime_ = this->get_clock()->now();
        std::string message = serialPort->m_data.substr(0,endPosition);

        //std::cout << "message:" << message.c_str() << std::endl;
        int statusStart = message.find(":");
        state_ = message.substr(statusStart+1,std::string::npos);

        serialPort->m_data.erase(0, endPosition+1);
        return true;
    } else
    {
        if(startPosition > 0)
        {
            serialPort->m_data.erase(0, startPosition);
        }
        return false;
    }
}

void RunStop::doWorkTimerCallback()
{
  //get out all the messages
 	while(processData())
 	{ 	
 	}

    rcraicer_msgs::msg::RunStop rsMsg;
    rsMsg.header.stamp = this->get_clock()->now();
    rsMsg.header.frame_id="RUNSTOP";
    rsMsg.sender = "RUNSTOP";

    if(state_ == "GREEN")
    {
        rsMsg.motion_enabled = true;
    } else if(state_ == "YELLOW")
    {
        rsMsg.motion_enabled = false;
    } else
    {
        rsMsg.motion_enabled = false;
    }

    //if no recent message, runstop is false
    if(this->get_clock()->now().seconds() -lastMessageTime_.seconds() > 1.0)
    {
        RCLCPP_ERROR(this->get_logger(), "No recent data from runstop box");
        rsMsg.motion_enabled = false;
    }

    rsPublisher->publish(rsMsg);    
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RunStop>());
    rclcpp::shutdown();
    return 0;
}
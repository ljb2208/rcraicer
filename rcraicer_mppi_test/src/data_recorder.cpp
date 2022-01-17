#include "../include/rcraicer_mppi_test/data_recorder.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


DataRecorder::DataRecorder() : Node("data_recorder_node")
{
    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&DataRecorder::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();
 
    ssSubscription = this->create_subscription<rcraicer_msgs::msg::SimState>(
      "sim_state", 10, std::bind(&DataRecorder::sim_state_callback, this, std::placeholders::_1));       

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&DataRecorder::joy_callback, this, std::placeholders::_1));    

    fileName = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/training_data/dynamics_data.csv";

    // std::filesystem::status stat(fileName);

    if (!std::filesystem::exists(std::filesystem::status(fileName)))
        writeHeader = true;


    if (deleteFileOnStart)
    {
        std::scoped_lock lock(fileMutex);          

        if (std::filesystem::exists(std::filesystem::status(fileName)))
        {            
            remove(fileName.c_str());
            RCLCPP_INFO(this->get_logger(), "File deleted. %s", fileName.c_str());
            writeHeader = true;
        }
        
    }    

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

DataRecorder::~DataRecorder()
{
    closeFile();
}


rcl_interfaces::msg::SetParametersResult DataRecorder::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
    }

    updateInternalParams();
    return result;
}


void DataRecorder::updateInternalParams()
{    
}

void DataRecorder::openFile()
{
    std::scoped_lock lock(fileMutex);  

    csvFile.open(fileName, std::ios::out | std::ios::app);

    //write header
    if (writeHeader)
    {
        // csvFile << "x_pos,y_pos,z_pos,roll,pitch,yaw,u_x,u_y,u_z,comp_u_x,comp_u_y,roll_mder,pitch_mder,yaw_mder,steer,throttle,brake,time_step,act_x_pos,act_y_pos,act_z_pos,act_roll,act_pitch,act_yaw,act_u_x,act_u_y,act_u_z,act_comp_u_x,act_comp_u_y,act_roll_mder,act_pitch_mder,act_yaw_mder\n";
        csvFile << "x_pos,y_pos,z_pos,roll,pitch,yaw,u_x,u_y,u_z,comp_u_x,comp_u_y,roll_mder,pitch_mder,yaw_mder,steer,throttle,brake,time_step,act_x_pos,act_y_pos,act_z_pos,act_roll,act_pitch,act_yaw,act_u_x,act_u_y,act_u_z,act_comp_u_x,act_comp_u_y,act_roll_mder,act_pitch_mder,act_yaw_mder,accel_roll,accel_x,accel_y,accel_yaw, accel2_x,accel2_y,accel2_yaw\n";
        writeHeader = false;
    }
    csvFile.precision(10);
}

void DataRecorder::closeFile()
{
    std::scoped_lock lock(fileMutex);  

    if (csvFile.is_open())
        csvFile.close();
}

void DataRecorder::sim_state_callback(const rcraicer_msgs::msg::SimState::SharedPtr msg)
{        
    if (!recordMode)
        return;

    double roll, pitch, yaw;
    double tsDelta = 0.0;

    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    float adjYaw = 0.0;

    if (priorYaw > 2.5 and yaw < -2.5)
        headingMultipler += 1;
    else if(priorYaw < -2.5 and yaw > 2.5)
        headingMultipler -= 1;
    else
        headingMultipler = 0;
            
    adjYaw = yaw + headingMultipler*2*M_PI;



    if (priorMessageAvailable)
    {                   
        rclcpp::Time ts = msg->header.stamp;
        rclcpp::Time priorTs = priorMsg.header.stamp;

        tsDelta = ts.seconds() - priorTs.seconds();        

        float distance = distanceTravelled(msg->env_position.x, msg->env_position.y, priorMsg.env_position.x, priorMsg.env_position.y);

        if (outputEnabled && distance >= minDistance)
        {      
            std::scoped_lock lock(fileMutex);              
            // csvFile << priorMsg.brake;
            // csvFile << ",";
            csvFile << priorMsg.env_position.x;
            csvFile << ",";
            csvFile << priorMsg.env_position.y;
            csvFile << ",";
            csvFile << priorMsg.env_position.z;
            csvFile << ",";            
            csvFile << priorRoll;
            csvFile << ",";
            csvFile << priorPitch;
            csvFile << ",";
            csvFile << priorYaw;
            csvFile << ",";
            csvFile << priorMsg.linear_velocity.x;
            csvFile << ",";
            csvFile << priorMsg.linear_velocity.y;                        
            csvFile << ",";
            csvFile << priorMsg.linear_velocity.z;                        
            csvFile << ",";            

            float priorXVel = cos(priorYaw)*priorMsg.linear_velocity.x + sin(priorYaw)*priorMsg.linear_velocity.y;
            float priorYVel = -sin(priorYaw)*priorMsg.linear_velocity.x + cos(priorYaw)*priorMsg.linear_velocity.y;             

            csvFile << priorXVel;            
            csvFile << ",";
            csvFile << priorYVel;
            csvFile << ",";            
            csvFile << priorMsg.angular_velocity.x;
            csvFile << ",";
            csvFile << priorMsg.angular_velocity.y;
            csvFile << ",";
            csvFile << -priorMsg.angular_velocity.z;
            csvFile << ",";            
            csvFile << priorMsg.steering;
            csvFile << ",";
            csvFile << priorMsg.throttle;
            csvFile << ",";
            csvFile << priorMsg.brake;
            csvFile << ",";
            csvFile << tsDelta;
            csvFile << ",";

            // float priorYawmder = yawChange(priorYaw, priorYaw2) / priorTsDelta;
            // csvFile << priorYawmder;            

            csvFile << msg->env_position.x;
            csvFile << ",";
            csvFile << msg->env_position.y;
            csvFile << ",";
            csvFile << msg->env_position.z;            
            csvFile << ",";
            csvFile << roll;
            csvFile << ",";
            csvFile << pitch;            
            csvFile << ",";
            csvFile << yaw;
            csvFile << ",";
            csvFile << msg->linear_velocity.x;
            csvFile << ",";
            csvFile << msg->linear_velocity.y;
            csvFile << ",";
            csvFile << msg->linear_velocity.z;
            csvFile << ",";      

            float xVel = cos(adjYaw)*msg->linear_velocity.x + sin(adjYaw)*msg->linear_velocity.y;            
            float yVel = -sin(adjYaw)*msg->linear_velocity.x + cos(adjYaw)*msg->linear_velocity.y;

            csvFile << xVel;            
            csvFile << ",";
            csvFile << yVel;             
            csvFile << ",";                                    
            csvFile << msg->angular_velocity.x;
            csvFile << ",";
            csvFile << msg->angular_velocity.y;
            csvFile << ",";
            csvFile << -msg->angular_velocity.z;            


            float rollVel = (roll - priorRoll) / tsDelta;
            float xAccel = (xVel - priorXVel) / tsDelta;
            float yAccel = (yVel - priorYVel) / tsDelta;
            float yawAccel = -(msg->angular_velocity.z - priorMsg.angular_velocity.z) / tsDelta;

            csvFile << ",";
            csvFile << rollVel;
            csvFile << ",";
            csvFile << xAccel;
            csvFile << ",";
            csvFile << yAccel;
            csvFile << ",";
            csvFile << yawAccel;

            csvFile << ",";                                    
            csvFile << msg->angular_acceleration.x;
            csvFile << ",";
            csvFile << msg->angular_acceleration.y;
            csvFile << ",";
            csvFile << -msg->angular_acceleration.z;            

            // float yawmder = yawChange(yaw, priorYaw) / tsDelta;            
            // csvFile << yawmder;        
            csvFile << "\n";

            // std::cout << "yawmder: " << yawmder << " ang z: " << msg->angular_velocity.z << " prior: " << priorM\n";

            // if (fabs(yawmder) > M_PI_2/2)
            // {
            //     std::cout << "yaw: " << yaw << " prior yaw: " << priorYaw << " tsDelta: " << tsDelta << " priorYaw2: " << priorYaw2 << " priorTsDelta: " << priorTsDelta << " priorYawmder: " << priorYawmder << " yawmderr: " << yawmder << "\n";    
            // }
            // std::cout << "yaw: " << yaw << " prior yaw: " << priorYaw << " tsDelta: " << tsDelta << " priorYaw2: " << priorYaw2 << " priorTsDelta: " << priorTsDelta << " priorYawmder: " << priorYawmder << " yawmderr: " << yawmder << "\n";

            recordFlushCount++;
        }
    }

    if (recordFlushCount >= 100)
    {
        std::scoped_lock lock(fileMutex);  
        csvFile.flush();
        recordFlushCount = 0;
    }

    if (priorMessageAvailable)
        outputEnabled = true;    

    priorTsDelta = tsDelta;
    priorYaw2 = priorYaw;

    priorRoll = roll;
    priorPitch = pitch;
    priorYaw = yaw;
    copyMessage(msg);
}

void DataRecorder::copyMessage(const rcraicer_msgs::msg::SimState::SharedPtr msg)
{
    priorMsg.header = msg->header;
    priorMsg.env_position = msg->env_position;
    priorMsg.linear_acceleration = msg->linear_acceleration;
    priorMsg.linear_velocity = msg->linear_velocity;
    priorMsg.angular_velocity = msg->angular_velocity;
    priorMsg.throttle = msg->throttle;
    priorMsg.steering = msg->steering;
    priorMsg.brake = msg->brake;

    priorMessageAvailable = true;
}

float DataRecorder::yawChange(float yaw1, float yaw2)
{        
    float deltaYaw = fabs(yaw1 - yaw2);

    if (deltaYaw > M_PI_2)
    {
        if (yaw1 < -M_PI_2 && yaw2 > M_PI_2)
            yaw1 = -yaw1;
        
        if (yaw1 > M_PI_2 && yaw2 < -M_PI_2)
            yaw2 = -yaw2;
    }        

    return yaw1 - yaw2;
}

float DataRecorder::distanceTravelled(float x, float y, float x1, float y1)
{
    float deltaX = x - x1;
    float deltaY = y - y1;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

void DataRecorder::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{    
    rclcpp::Time ts = msg->header.stamp;    

    if ((ts.seconds() - recordModeStamp) < 1.0)
        return;

    if (msg->buttons[recordButtonId] == 1)
    {
        if (!recordMode)
        {
            recordMode = true;
            priorMessageAvailable = false;
            outputEnabled = false;

            openFile();
            RCLCPP_INFO(this->get_logger(), "Recording started");            
        }
        else
        {
            closeFile();
            recordMode = false;
            RCLCPP_INFO(this->get_logger(), "Recording stopped");            
        }

        recordModeStamp = ts.seconds();
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRecorder>());
    rclcpp::shutdown();
    return 0;
}

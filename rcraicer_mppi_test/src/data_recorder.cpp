#include "../include/rcraicer_mppi_test/data_recorder.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


DataRecorder::DataRecorder() : Node("data_recorder")
{
    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&DataRecorder::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();

    odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "pose", 10, std::bind(&DataRecorder::odom_callback, this, std::placeholders::_1));
 
    chassisStateSubscription = this->create_subscription<rcraicer_msgs::msg::ChassisState>(
        "chassis_state", 10, std::bind(&DataRecorder::chassis_state_callback, this, std::placeholders::_1));    

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
        csvFile << "roll,comp_u_x,comp_u_y,yaw_mder,steer,throttle,brake,time_step,act_roll_mder,accel_x,accel_y,accel_yaw\n";
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

void DataRecorder::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!recordMode)
        return;


    double roll, pitch, yaw;
    double tsDelta = 0.0;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
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

    if (priorMessageAvailable && chassisStateValid)
    {                   
        rclcpp::Time ts = msg->header.stamp;
        rclcpp::Time priorTs = priorMsg.header.stamp;

        tsDelta = ts.seconds() - priorTs.seconds();        

        float distance = distanceTravelled(msg->pose.pose.position.x, msg->pose.pose.position.y, priorMsg.pose.pose.position.x, priorMsg.pose.pose.position.y);

        if (outputEnabled && distance >= minDistance)
        {      
            std::scoped_lock lock(fileMutex);              
            
            csvFile << priorRoll;
            csvFile << ",";
            csvFile << priorMsg.twist.twist.linear.x;
            csvFile << ",";
            csvFile << priorMsg.twist.twist.linear.y;                        
            csvFile << ",";
            

            float priorXVel = cos(priorYaw)*priorMsg.twist.twist.linear.x + sin(priorYaw)*priorMsg.twist.twist.linear.y;
            float priorYVel = -sin(priorYaw)*priorMsg.twist.twist.linear.x + cos(priorYaw)*priorMsg.twist.twist.linear.y;             

            csvFile << priorXVel;            
            csvFile << ",";
            csvFile << priorYVel;
            csvFile << ",";            
            csvFile << -priorMsg.twist.twist.angular.z;
            csvFile << ",";            
            csvFile << steering;
            csvFile << ",";
            csvFile << throttle;
            csvFile << ",";
            csvFile << brake;
            csvFile << ",";
            csvFile << tsDelta;
            csvFile << ",";

            float rollVel = (roll - priorRoll) / tsDelta;
            
            csvFile << ",";
            csvFile << rollVel;

            csvFile << ",";                                    
            csvFile << msg->twist.twist.angular.x;
            csvFile << ",";
            csvFile << msg->twist.twist.angular.y;
            csvFile << ",";
            csvFile << -msg->twist.twist.angular.z;            
        
            csvFile << "\n";
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

void DataRecorder::copyMessage(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    priorMsg.header = msg->header;
    priorMsg.twist = msg->twist;
    priorMsg.pose = msg->pose;
    
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

void DataRecorder::chassis_state_callback(const rcraicer_msgs::msg::ChassisState::SharedPtr msg)
{
    rclcpp::Time ts = msg->header.stamp;

    chassisStateValid = true;
    steering = msg->steer;
    throttle = msg->throttle;
    brake = msg->front_brake;
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

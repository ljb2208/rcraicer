#include "../include/rcraicer_sim/sim_debug_node.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


SimDebugNode::SimDebugNode() : Node("sim_debug_node")
{
    gtMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("gt_marker", 10);
    odomMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("odom_marker", 10);

    odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SimDebugNode::onOdomMessage, this, std::placeholders::_1));   

    simSubscription = this->create_subscription<rcraicer_msgs::msg::SimState>(
      "sim_state", 10, std::bind(&SimDebugNode::onSimMessage, this, std::placeholders::_1));   

}

SimDebugNode::~SimDebugNode()
{

}


void SimDebugNode::onOdomMessage(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    visualization_msgs::msg::Marker dMsg = visualization_msgs::msg::Marker();

    dMsg.header.frame_id = "map";
    dMsg.header.stamp = this->get_clock()->now();
    dMsg.id = odomMarkerId++;
    dMsg.type = visualization_msgs::msg::Marker::ARROW;
    dMsg.action = visualization_msgs::msg::Marker::ADD;
    dMsg.pose = msg->pose.pose;
    dMsg.scale.x = 0.5;
    dMsg.scale.y = 0.5;
    dMsg.scale.z = 0.5;
    dMsg.color.r = 1.0;
    dMsg.color.g = 0.0;
    dMsg.color.b = 0.0;
    dMsg.color.a = 1.0;

    dMsg.frame_locked = false;        

    odomMarkerPublisher->publish(dMsg);
}

void SimDebugNode::onSimMessage(const rcraicer_msgs::msg::SimState::SharedPtr msg)
{
    visualization_msgs::msg::Marker dMsg = visualization_msgs::msg::Marker();

    dMsg.header.frame_id = "map";
    dMsg.header.stamp = this->get_clock()->now();
    dMsg.id = gtMarkerId++;
    dMsg.type = visualization_msgs::msg::Marker::ARROW;
    dMsg.action = visualization_msgs::msg::Marker::ADD;
    dMsg.pose.position.x = msg->env_position.x; 
    dMsg.pose.position.y = msg->env_position.y; 
    dMsg.pose.position.z = msg->env_position.z; 
    dMsg.pose.orientation = msg->orientation;
    dMsg.scale.x = 0.5;
    dMsg.scale.y = 0.5;
    dMsg.scale.z = 0.5;
    dMsg.color.r = 0.0;
    dMsg.color.g = 1.0;
    dMsg.color.b = 0.0;
    dMsg.color.a = 1.0;

    dMsg.frame_locked = false;        

    gtMarkerPublisher->publish(dMsg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDebugNode>());
    rclcpp::shutdown();
    return 0;
}
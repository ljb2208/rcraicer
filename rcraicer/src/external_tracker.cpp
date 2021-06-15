#include "../include/rcraicer/external_tracker.h"

ExternalTracker::ExternalTracker() : Node("external_tracker")
{
    this->get_parameter_or("camera_path", cameraPath, rclcpp::Parameter("camera_path", "/dev/ttyUSB1"));
}

ExternalTracker::~ExternalTracker()
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExternalTracker>());
    rclcpp::shutdown();
    return 0;
}
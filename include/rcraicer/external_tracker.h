#include "rclcpp/rclcpp.hpp"

class ExternalTracker : public rclcpp::Node
{
    public:
        ExternalTracker();
        ~ExternalTracker();

    private:
        rclcpp::Parameter cameraPath;
};
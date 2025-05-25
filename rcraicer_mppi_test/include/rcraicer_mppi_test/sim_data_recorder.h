#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <mutex>
#include <filesystem>

#include "cnpy.h"

# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */

class SimDataRecorder : public rclcpp::Node
{
    public:
        SimDataRecorder();
        ~SimDataRecorder();        

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
        rclcpp::Subscription<rcraicer_msgs::msg::SimState>::SharedPtr ssSubscription;
        
        rclcpp::TimerBase::SharedPtr publishTimer;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);             
        void sim_state_callback(const rcraicer_msgs::msg::SimState::SharedPtr msg);
        void updateInternalParams();                

        void copyMessage(const rcraicer_msgs::msg::SimState::SharedPtr msg);
        void openFile();
        void closeFile();
        float distanceTravelled(float x, float y, float x1, float y1);
        float yawChange(float yaw1, float yaw2);

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;                

        std::ofstream csvFile;
        std::string fileName;

        bool recordMode {false};
        int recordButtonId {4};
        double recordModeStamp {0.0};        

        bool priorMessageAvailable {false};
        bool outputEnabled {false};

        int recordFlushCount {0};
        int deleteFileOnStart {false};
        bool writeHeader {false};

        float minDistance {0.05};
  
        

        rcraicer_msgs::msg::SimState priorMsg;
        float priorRoll;
        float priorPitch;
        float priorYaw;

        float priorYaw2;
        float priorTsDelta;
        int headingMultipler;

        std::mutex fileMutex;

};
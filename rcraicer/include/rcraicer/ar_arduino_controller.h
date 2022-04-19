#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>
#include <vector>

#include "serial_port.h"
#include "serial_data_msg.h"

#include "rcraicer_msgs/msg/encoder.hpp"
#include "rcraicer_msgs/msg/arduino_status.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/run_stop.hpp"


class ARArduinoController : public rclcpp::Node 
{
    public: 
        /*
        * @struct ActuatorConfig 
        * @brief calibrated min, center, and max for the pulse witdth, in us, for an actuator
        *        
        * @note reverse is currently not used
        */
        struct ActuatorConfig
        {
            unsigned short center; ///< calibrated zero of servo in us
            unsigned short min;    ///< calibrated minimum of servo in us (left)
            unsigned short max;    ///< calibrated maximum of servo in us (right)
            bool reverse;          ///< if the servo should be reversed

            ActuatorConfig():
            center(1500),
            min(1000),
            max(2000),
            reverse(false)
            {}
        };
        ARArduinoController();        
        ~ARArduinoController();

    private:
        /*
        * @struct priorityEntry 
        * @brief Entry for each chassis commander loaded from the commanders file. The highest priority is 0.
        *        
        * @note the id must be the same as the sender in received chassisCommand messages
        */
        struct priorityEntry
        {
            std::string id; ///< Unique identifying string for a program that will control the chassis
            unsigned int priority; ///< Priority of the commander, 0 is highest priotity
        };
        
        struct priorityComparator
        {
            bool operator() (const priorityEntry& a, const priorityEntry& b)
            {
            return a.priority < b.priority;
            }
        };

        void sendCommandToChassis(rcraicer_msgs::msg::ChassisState state);
        void processChassisMessage(std::string msgType, std::string msg);
        void tokenize(std::string const &str, const char delim, std::vector<std::string> &out);
        double actuatorUsToCmd(int pulseWidth, std::string actuator);
        short actuatorCmdToMs(double actuatorValue, std::string actuator);


        void loadChassisCommandPriorities();
        void setChassisActuators();

        rclcpp::Publisher<rcraicer_msgs::msg::WheelSpeed>::SharedPtr wsPublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisState>::SharedPtr statePublisher;
        rclcpp::Publisher<rcraicer_msgs::msg::ChassisCommand>::SharedPtr cmdPublisher;

        int escDataFailCounter_;  
        std::map<std::string, std::pair<bool, int> > invalidActuatorPulses_;
        std::map<std::string, double> mostRecentRc_;
        double mostRecentRcSteering_; ///< Most recent RC steering command received from the chassis
        double mostRecentRcThrottle_; ///< Most recent RC throttle command received from the chassis
        double mostRecentRcFrontBrake_; ///< Most recent RC front brake command received from the chassis
        std::map<std::string, ActuatorConfig> actuatorConfig_; ///< Map of actuator configs (min, center, max) for each

        std::mutex rcMutex_; ///< mutex for most recent RC actuator values

        std::mutex chassisEnableMutex_; ///< mutex for accessing chassis state variables

        bool throttleRelayEnabled_; ///< indicated whther the throttle relay is engaged or not
        bool autonomousEnabled_; ///< indicates if the chassis is in autonomous or manual mode

        int64_t commandRate;
        double commandMaxAge;
        double runstopMaxAge;

        std::map<std::string, rcraicer_msgs::msg::RunStop::SharedPtr> runstops_; ///< Map of the most recently received runstop message from
                                                            ///< all nodes publishing the message

        std::map<std::string, rcraicer_msgs::msg::ChassisCommand::SharedPtr> chassisCommands_; ///< Map of the most recently received chassis
                        ///< command from each commander

        rclcpp::TimerBase::SharedPtr chassisControlTimer;

        std::vector<priorityEntry> chassisCommandPriorities_; ///< Priority list used to choose which commander controls the
                                                        ///< actuators

        ///< Text descriptions and multiplier values for each data register received from the ESC. This information comes from
        ///< the Castl Serial Link documenation
        std::vector<std::pair<std::string, double> > escRegisterData_ =
                { {"ESC Input Voltage", 20.0},
                {"ESC Input Ripple Voltage", 4.0},
                {"ESC Current", 50.0},
                {"Throttle ms", 1.0},
                {"Output Power %", 0.2502},
                {"Motor RPM", 20416.66},
                {"Temperature", 30.0},
                {"BEC Voltage", 4.0},
                {"BEC Current", 4.0}, };

        // old code below

        
        void command_callback(const rcraicer_msgs::msg::ChassisCommand::SharedPtr msg);        
        void runstop_callback(const rcraicer_msgs::msg::RunStop::SharedPtr msg);        
        void serial_data_callback();        
        void publishChassisState(float throttle, float steer, float steerAngle);
        
        void writeData(data_msg dmsg);                

        void updateInternalParams();

        float getSteeringAngle(int steerPWM);
        int32_t getSteeringPWM(float value);
        int32_t getThrottlePWM(float value);
        int32_t getPWM(float value, float inputFactor, int64_t min, int64_t mid, int64_t max);        
        
        rclcpp::Publisher<rcraicer_msgs::msg::ArduinoStatus>::SharedPtr statusPublisher;
                
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr paramSubscription;        
        rclcpp::Subscription<rcraicer_msgs::msg::RunStop>::SharedPtr runstopSubscription;        

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;

        SerialPort* serialPort;

        rclcpp::Parameter portPath;
        rclcpp::Parameter baudRate;        
        rclcpp::Parameter throttleReverseParam;
        rclcpp::Parameter steeringReverseParam;
        rclcpp::Parameter brakeReverseParam;
        rclcpp::Parameter steeringServoPoints;
        rclcpp::Parameter throttleServoPoints;
        rclcpp::Parameter brakeServoPoints;
        rclcpp::Parameter steeringDegreesPerTick;
        rclcpp::Parameter wheelDiameterParam;
        rclcpp::Parameter commandRateParam;
        rclcpp::Parameter chassisCommandMaxAgeParam;
        rclcpp::Parameter runstopMaxAgeParam;
        rclcpp::Parameter chassisCommandPrioritiesPraram;


        double steeringAngleCoefficient;
        
        float steeringInputFactor;
        float throttleInputFactor;

        double wheelDiameter;

        int64_t steeringServoMin;
        int64_t steeringServoMax;
        int64_t steeringServoMid;

        int64_t throttleServoMin;
        int64_t throttleServoMax;
        int64_t throttleServoMid;

        int64_t brakeServoMin;
        int64_t brakeServoMax;
        int64_t brakeServoMid;           

        

        bool cmdReceived;
        float lastCommandTime;

        bool isArmed;
        int64_t armButtonValue;
        uint16_t invalidCRC;
        uint16_t unknownMsg;

        const double PI = 3.14159265; ///< Value for pi

  
};
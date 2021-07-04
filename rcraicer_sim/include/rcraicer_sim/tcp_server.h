#ifndef _TCP_SERVER_H
#define _TCP_SERVER_H

#include <string>

#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <vector>

#define RAPIDJSON_HAS_STDSTRING 1

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/image.hpp"


#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>


#define BUFFER_SIZE 64128
#define MSG_BUFFER_SIZE 64128 * 10
#define GRAVITY (9.80665)

#define SYNC_CHAR '{'
#define END_SYNC_CHAR1 '}'
#define END_SYNC_CHAR2 '\n'

/* Decoder state */
typedef enum {
	DECODE_SYNC = 0,		    
//	DECODE_PAYLOAD,
	DECODE_END_SYNC1,
    DECODE_END_SYNC2,

} decode_state_t;

typedef enum {
    SCENE_SELECTION_READY = 0,
    SCENE_LIST,
    CAR_LOADED,
    CROSS_START,
    RACE_START,
    RACE_STOP,
    DQ,
    PING,
    ABORTED,
    MISSED_CHECKPOINT,
    NEED_CAR_CONFIG
} sim_event_t;

class TcpServer
{
    public:
        TcpServer(std::string ipAddress, std::string port);

        ~TcpServer();

        void enableImagePublishing();

        bool connectToSocket();
        int writePort(const unsigned char* data, unsigned int length);
        int writePortTry(const unsigned char* data, unsigned int length);

        void lock();
        bool tryLock();
        void unlock();

        typedef std::function<void(sim_event_t simEvent)> EventCallback;
        typedef std::function<void(rcraicer_msgs::msg::WheelSpeed wsMsg, rcraicer_msgs::msg::ChassisState csMsg, sensor_msgs::msg::Imu imuMsg, sensor_msgs::msg::NavSatFix fixMsg)> TelemetryCallback;
        typedef std::function<void(sensor_msgs::msg::Image imageMsg)> ImageCallback;
        void registerEventCallback(EventCallback callback);
        void registerImageCallback(ImageCallback callback);
        void registerTelemetryCallback(TelemetryCallback callback);
        void clearTelemetryCallback();
        void clearImageCallback();
        void clearEventCallback();
        void waitForData();

        std::vector<std::string> getScenes();
        bool sendSceneSelection(std::string scene_name);           
        bool sendSceneList();

        void setControls(float throttle, float steering, float brakes);        

        bool isConnected()
        {
            return connected;
        }

        std::string getErrorString()
        {
            return port_setting_error;
        }

    private:        

        void run();

        int writePortInternal(const unsigned char* data, unsigned int length) const;        
        void processData(const unsigned char* data, unsigned int length);
        void decodeInit();
        void addToMsgBuffer(const unsigned char data);
        void processMessage();
        bool sendControls();     

        std::string ipAddress;
        std::string port;
        int port_fd; // file descriptor for serial port
        std::string port_setting_error;
        bool connected;
        std::vector<char> dataBuffer;

        std::shared_ptr<std::thread> runThread; ///< pointer to the read thread
        std::mutex dataMutex; ///< mutex for accessing incoming data
        std::mutex writeMutex; ///< mutex for writing serial data
        std::mutex waitMutex; ///< mutex for thread synchronization

        TelemetryCallback telemCallback; ///< Callback triggered when new data arrives
        ImageCallback imageCallback;
        EventCallback eventCallback;

        volatile bool alive;

        uint8_t msgBuffer[MSG_BUFFER_SIZE];
        int msgIndex {0};
        decode_state_t decode_state {DECODE_SYNC};

        rapidjson::Document jsonDoc;
        rapidjson::Document controlDoc;
        rapidjson::StringBuffer controlBuffer;
        // rapidjson::Writer<rapidjson::StringBuffer> controlWriter(controlBuffer);

        rcraicer_msgs::msg::ChassisState csMsg;
        rcraicer_msgs::msg::WheelSpeed wsMsg;
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::MagneticField magMsg;
        sensor_msgs::msg::NavSatFix fixMsg;

        double latitude {41.00469};
        double longitude {-74.08575};
        double altitude {20.0};

        float throttle {0.0};
        float steering {0.0};
        float brakes {0.0};

        bool publishImages {false};
        uint8_t txBuffer[BUFFER_SIZE];

        GeographicLib::Geodesic* geod;

        std::vector<std::string> sceneNames;
        
};

#endif
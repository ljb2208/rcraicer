#include <string>

#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <vector>

#include "diagnostics.h"


class SerialPort : public Diagnostics
{
    public:
        SerialPort();

        SerialPort(rclcpp::Node::SharedPtr node,
                        const std::string& nodeName,
                        const std::string& hardwareID,
                        const std::string& port,
                        int baudRate, uint8_t messageDelim, uint8_t connectionType);

        ~SerialPort();

        bool connect(std::string port, int baudRate);
        int writePort(const unsigned char* data, unsigned int length);
        int writePort(const std::string data);
        int writePortTry(const unsigned char* data, unsigned int length);

        void lock();
        bool tryLock();
        void unlock();

        typedef std::function<void(const uint8_t)> ByteDataCallback;
        typedef std::function<void()> DataCallback;
        void registerDataCallback(DataCallback callback);
        void clearDataCallback();

        void registerByteDataCallback(ByteDataCallback callback);
        void clearByteDataCallback();

        bool getNextMessage(unsigned char* data, int dataLength, int& length);

        void waitForData();

        bool isConnected()
        {
            return connected;
        }

        std::string getErrorString()
        {
            return port_setting_error;
        }

        std::string m_data;

    private:        

        void run();

        int writePortInternal(const unsigned char* data, unsigned int length) const;
        int writePortInternal(const char* data, unsigned int length) const;

        uint8_t connectionType;
        uint8_t messageDelim;
        std::string port;
        int port_fd; // file descriptor for serial port
        std::string port_setting_error;
        bool connected;
        std::vector<char> dataBuffer;

        std::shared_ptr<std::thread> runThread; ///< pointer to the read thread
        std::mutex dataMutex; ///< mutex for accessing incoming data
        std::mutex writeMutex; ///< mutex for writing serial data
        std::mutex waitMutex; ///< mutex for thread synchronization

        ByteDataCallback byteDataCallback;
        DataCallback dataCallback; ///< Callback triggered when new data arrives
        volatile bool alive;

        void diagnosticStatus();
};

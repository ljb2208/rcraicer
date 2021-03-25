#include <string>

#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <vector>

#include "../mavlink/common/mavlink.h"


class MavlinkSerialPort
{
    public:
        MavlinkSerialPort(std::string port, int baudRate);

        ~MavlinkSerialPort();

        bool connect(std::string port, int baudRate);
        int writePort(const unsigned char* data, unsigned int length);
        int writePortTry(const unsigned char* data, unsigned int length);

        void lock();
        bool tryLock();
        void unlock();

        typedef std::function<void()> DataCallback;
        void registerDataCallback(DataCallback callback);
        void clearDataCallback();
        void waitForData();

        mavlink_message_t* getMessage();
        mavlink_status_t* getStatus();

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

        mavlink_message_t msg;
        mavlink_status_t status;

        std::string port;
        int port_fd; // file descriptor for serial port
        std::string port_setting_error;
        bool connected;
        std::vector<char> dataBuffer;

        std::shared_ptr<std::thread> runThread; ///< pointer to the read thread
        std::mutex dataMutex; ///< mutex for accessing incoming data
        std::mutex writeMutex; ///< mutex for writing serial data
        std::mutex waitMutex; ///< mutex for thread synchronization

        DataCallback dataCallback; ///< Callback triggered when new data arrives
        volatile bool alive;
};

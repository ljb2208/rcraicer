#ifndef _TCP_SERVER_H
#define _TCP_SERVER_H

#include <string>

#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <vector>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#define BUFFER_SIZE 64128
#define MSG_BUFFER_SIZE 64128 * 10

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

class TcpServer
{
    public:
        TcpServer(std::string ipAddress, std::string port);

        ~TcpServer();

        bool connectToSocket();
        int writePort(const unsigned char* data, unsigned int length);
        int writePortTry(const unsigned char* data, unsigned int length);

        void lock();
        bool tryLock();
        void unlock();

        typedef std::function<void(const uint8_t)> DataCallback;
        void registerDataCallback(DataCallback callback);
        void clearDataCallback();
        void waitForData();                

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

        DataCallback dataCallback; ///< Callback triggered when new data arrives
        volatile bool alive;

        uint8_t msgBuffer[MSG_BUFFER_SIZE];
        int msgIndex {0};
        decode_state_t decode_state {DECODE_SYNC};

        rapidjson::Document jsonDoc;
};

#endif
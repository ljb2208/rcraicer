#include "../include/rcraicer_sim/tcp_server.h"

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <iostream>
#include <sstream>
#include <cstring>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

TcpServer::TcpServer(std::string ipAddress, std::string port): port_fd(-1), port_setting_error(""), connected(false), dataCallback(NULL), alive(false) 
{
    this->port = port;    
    this->ipAddress = ipAddress;
}

TcpServer::~TcpServer()
{
  if (alive)
  {
    alive = false;
    runThread->join();

  }
}

void TcpServer::run()
{  
  uint8_t data[BUFFER_SIZE];      

  volatile int received;    

  if (!isConnected())
  {
    return;
  }

  while (alive)
  {    
    /* Wait up to one seconds. */    
    received = recv(port_fd, &data, BUFFER_SIZE, MSG_DONTWAIT);    

    if (received > 0)
    {        
          dataMutex.lock();          

          processData(data, received);          

        //   for (int i=0; i < received; i++)
        //   {            
        //     if (dataCallback != NULL)
        //       dataCallback(data[i]);                                                                          
        //   }

          dataMutex.unlock();                                     
    }
  }
}

void TcpServer::processData(const unsigned char* data, unsigned int length)
{
    for (unsigned int i=0; i < length; i++)
    {
      switch(decode_state)
      {
        case DECODE_SYNC:
          if (data[i] == SYNC_CHAR)
          {
            addToMsgBuffer(data[i]);
            decode_state = DECODE_END_SYNC1;
          }
          
          break;          
        case DECODE_END_SYNC1:
          if (data[i] == END_SYNC_CHAR1)
            decode_state = DECODE_END_SYNC2;          
          
          addToMsgBuffer(data[i]);
          
          break;
        
        case DECODE_END_SYNC2:
          if (data[i] == END_SYNC_CHAR2)
          {
            addToMsgBuffer(data[i]);
            processMessage();
          }
            // decode_state = DECODE_END_SYNC2;
          
          decodeInit();
          
          break;
      }
    }
}

void TcpServer::addToMsgBuffer(const unsigned char data)
{  
  if (msgIndex >= MSG_BUFFER_SIZE)
  {
    std::cout << "Message buffer length error\r\n";
    decodeInit();
    return;
  }
  msgBuffer[msgIndex++] = data;
}

void TcpServer::decodeInit()
{
  msgIndex = 0;  
  decode_state = DECODE_SYNC;
}

void TcpServer::processMessage()
{    
    msgBuffer[msgIndex] = '\0';

    std::string output((char*) msgBuffer, msgIndex);
    std::cout << "Received: " << output.c_str() << "\r\n";

    if (msgIndex > 3)
    {
      jsonDoc.ParseInsitu((char*)msgBuffer);
      std::cout << "Message type: " <<  jsonDoc["msg_type"].GetString() << "\r\n";
    }        
}

void TcpServer::lock()
{
  dataMutex.lock();
}

bool TcpServer::tryLock()
{
  return dataMutex.try_lock();
}

void TcpServer::unlock()
{
  dataMutex.unlock();
}

void TcpServer::registerDataCallback(DataCallback callback)
{
  dataMutex.lock();          
  dataCallback = callback;
  dataMutex.unlock();                             
}

void TcpServer::clearDataCallback()
{
  dataMutex.lock();          
  dataCallback = NULL;
  dataMutex.unlock();                             
}

bool TcpServer::connectToSocket()
{
    connected = false;
    addrinfo hints, *p;
    
    
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags    = AI_PASSIVE;

    memset(&hints, 0, sizeof(hints));
    int gAddRes = getaddrinfo(ipAddress.c_str(), port.c_str(), &hints, &p);
    if (gAddRes != 0) {        
        return false;
    }

    if (p == NULL)
    {
        return false;
    }

    port_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);

    if (port_fd == -1)
    {
        port_setting_error = "could not open port";
        return false;
    }

    int connectR = connect(port_fd, p->ai_addr, p->ai_addrlen);

    if (connectR == -1)
    {
        return false;
    }

    connected = true;

    alive = true;
    runThread = std::shared_ptr<std::thread>(new std::thread(std::bind(&TcpServer::run, this)));
    return true;
}

int TcpServer::writePortInternal(const unsigned char* data, unsigned int length) const
{  
  if (!connected)
    return -2;

  int n;
  n=write(port_fd, data, length);

  if(n < 0)
  {      
    return -1;
  }    

  return n;
}

int TcpServer::writePort(const unsigned char* data, unsigned int length)
{
  if (isConnected())
  {
    std::unique_lock<std::mutex> lock(writeMutex);
    return writePortInternal(data, length);
  }

  return -1;
}

int TcpServer::writePortTry(const unsigned char* data, unsigned int length)
{
  if (isConnected())
  {
    std::unique_lock<std::mutex> lock(writeMutex, std::try_to_lock);

    if (lock)
      return writePortInternal(data, length);
  }

  return -1;
}
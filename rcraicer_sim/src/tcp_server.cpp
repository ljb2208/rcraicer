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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

TcpServer::TcpServer(std::string ipAddress, std::string port): port_fd(-1), port_setting_error(""), connected(false), telemCallback(NULL), alive(false), geod(NULL)
{
    this->port = port;    
    this->ipAddress = ipAddress;

    geod = new GeographicLib::Geodesic(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    earth = new GeographicLib::Geocentric(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    proj = new GeographicLib::LocalCartesian(latitude, longitude, altitude, *earth);

    // create control json doc
    controlDoc.SetObject();
    rapidjson::Document::AllocatorType& allocator = controlDoc.GetAllocator();
    // controlWriter = rapidjson::Writer<rapidjson::StringBuffer>(controlBuffer);
    // controlDoc.Accept(controlWriter);

    controlDoc.AddMember("msg_type", "control", allocator);
    controlDoc.AddMember("throttle", 0.0, allocator);
    controlDoc.AddMember("steering", 0.0, allocator);
    controlDoc.AddMember("brake", 0.0, allocator);

}

TcpServer::~TcpServer()
{
  if (alive)
  {
    alive = false;
    runThread->join();

  }

  if (geod != NULL)
    delete geod;
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

    if (msgIndex <= 3)
      return;

    // std::string output((char*) msgBuffer, msgIndex);
    // std::cout << "Message: " << output.c_str() << "\r\n";
    
    jsonDoc.ParseInsitu((char*)msgBuffer);
    std::string msgType = jsonDoc["msg_type"].GetString();

    if (msgType.compare("telemetry") == 0)
    {
      wsMsg.left_rear = jsonDoc["ws_lr"].GetFloat();
      wsMsg.right_rear = jsonDoc["ws_rr"].GetFloat();
      wsMsg.left_front = jsonDoc["ws_lf"].GetFloat();
      wsMsg.right_front = jsonDoc["ws_rf"].GetFloat();
      // wsMsg.right_front = jsonDoc["speed"].GetFloat();

      csMsg.throttle = jsonDoc["throttle"].GetFloat();
      csMsg.steer = jsonDoc["steer"].GetFloat();
      csMsg.steer_angle = jsonDoc["steer_angle"].GetFloat();

      // Unity coords forward = z, right = x, up = y
      // ros coords forward =x, left = y, up = z

      tf2::Quaternion gyro(-jsonDoc["gyro_z"].GetFloat(), jsonDoc["gyro_x"].GetFloat(), -jsonDoc["gyro_y"].GetFloat(), jsonDoc["gyro_w"].GetFloat());
      tf2::Matrix3x3 mGyro(gyro);
      mGyro.getRPY(imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z);

      imuMsg.linear_acceleration.x = jsonDoc["accel_z"].GetFloat();
      imuMsg.linear_acceleration.y = -(jsonDoc["accel_x"].GetFloat());
      imuMsg.linear_acceleration.z = jsonDoc["accel_y"].GetFloat() + GRAVITY;

      imuMsg.orientation.x = -jsonDoc["orient_z"].GetFloat();
      imuMsg.orientation.y = jsonDoc["orient_x"].GetFloat();
      imuMsg.orientation.z = -jsonDoc["orient_y"].GetFloat();
      imuMsg.orientation.w = jsonDoc["orient_w"].GetFloat();

      double roll, pitch, yaw;
      tf2::Quaternion orient(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
      tf2::Matrix3x3 mOrient(orient);
      mOrient.getRPY(roll, pitch, yaw);

      double posx = jsonDoc["pos_z"].GetDouble();
      double posy = -jsonDoc["pos_x"].GetDouble();
      double posz = jsonDoc["pos_y"].GetDouble();

      if (!initialPosReceived)
      {
        initPosX = posx;
        initPosY = posy;
        initPosZ = posz;

        priorPosX = posx;
        priorPosY = posy;
        priorPosZ = posz;
        initialPosReceived = true;

        fixMsg.latitude = latitude;
        fixMsg.longitude = longitude;
        fixMsg.altitude = altitude + posz;
        fixMsg.status.status = 2;
        fixMsg.status.service = 11;
      }
      else
      {

        double dposx = posx - priorPosX;
        double dposy = posy - priorPosY;
        double dposz = posz - priorPosZ;

        double ddposx = posx - initPosX;
        double ddposy = posy - initPosY;
        double ddposz = posz - initPosZ;

        double nlat, nlong, nalt;
        
        proj->Reverse(ddposx, ddposy, ddposz, latitude, longitude, altitude);


        // std::cout << "New Calc: lat: " << std::fixed << nlat << " lon: " << nlong << " alt: " << nalt << "\r\n";

        // double distance = sqrt(dposx*dposx + dposy*dposy);
        // double latitude2, longitude2;
        // geod->Direct(latitude, longitude, yaw * 180.0 / PI, distance, latitude2, longitude2);

        fixMsg.latitude = latitude;
        fixMsg.longitude = longitude;
        fixMsg.altitude = altitude;
        fixMsg.status.status = 2;
        fixMsg.status.service = 11;

        // latitude = latitude2;
        // longitude = longitude2;      

        // llarToWorld(latitude, longitude, altitude + posz);

        // std::cout.precision(17);
        // std::cout << "GPS: lat: " <<  std::fixed << fixMsg.latitude << " lon: " << std::fixed << fixMsg.longitude << " alt: " << std::fixed << fixMsg.altitude << "\r\n";
        // std::cout << "GPS: distance: " <<  std::fixed << distance << " yaw: " << std::fixed << yaw * 180 / PI << "\r\n";

        priorPosX = posx;
        priorPosY = posy;
        priorPosZ = posz;        
      }      
      
      if (telemCallback != NULL)
        telemCallback(wsMsg, csMsg, imuMsg, fixMsg);

      sendControls();

      if (publishImages)
      {
          std::string img = jsonDoc["image"].GetString();          
      }
    }
    else if (msgType.compare("scene_names") == 0)
    {
      sceneNames.clear();

      const rapidjson::Value& scenes = jsonDoc["scene_names"];

      for (rapidjson::SizeType i=0; i < scenes.Size(); i++)
        sceneNames.push_back(scenes[i].GetString());


      if (eventCallback != NULL)
        eventCallback(SCENE_LIST);
    }
    else if (msgType.compare("scene_selection_ready") == 0)
    {
      if (eventCallback != NULL)
        eventCallback(SCENE_SELECTION_READY);
    }
    else if (msgType.compare("car_loaded") == 0)
    {
      if (eventCallback != NULL)
        eventCallback(CAR_LOADED);
    }
    else if (msgType.compare("aborted") == 0)
    {
      if (eventCallback != NULL)
        eventCallback(ABORTED);
    }
    else if (msgType.compare("need_car_config") == 0)
    {
      if (eventCallback != NULL)
        eventCallback(NEED_CAR_CONFIG);
    }
    else
    {
      std::cout << "Message received " << msgType.c_str() << "\r\n";
    }
}

void TcpServer::setControls(float throttle, float steering, float brakes)
{
    this->throttle = throttle;
    this->steering = steering;
    this->brakes = brakes;
}

std::vector<std::string> TcpServer::getScenes()
{
  return sceneNames;
}

bool TcpServer::sendSceneSelection(std::string scene_name)
{
  rapidjson::Document doc;
  rapidjson::StringBuffer buffer;

  doc.SetObject();
  rapidjson::Document::AllocatorType& alloc = doc.GetAllocator();

  doc.AddMember("msg_type", "load_scene", alloc);
  doc.AddMember("scene_name", rapidjson::StringRef(scene_name), alloc);

  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);

  std::string cmd = buffer.GetString();

  strcpy((char *) txBuffer, cmd.c_str());
  if (writePort(txBuffer, cmd.size()) > 0)
    return true;

  return false;
}

void TcpServer::llarToWorld(double lat, double lon, double alt)
{
  double f = 0;
  double rad = 0;
  double ls = atan((1 - f) * (1 - f) * tan(lat));
  double x = rad * cos(ls) * cos(lon) + alt * cos(lat) * cos(lon);
  double y = rad * cos(ls) * sin(lon) + alt * cos(lat) * sin (lon);
  double z = rad * sin(ls) + alt + sin(lat);

  std::cout << "Converted x/y/z: " << std::fixed << x << "/" << y << "/" << z << "\r\n";
} 

bool TcpServer::sendSceneList()
{
  rapidjson::Document doc;
  rapidjson::StringBuffer buffer;

  doc.SetObject();
  rapidjson::Document::AllocatorType& alloc = doc.GetAllocator();

  doc.AddMember("msg_type", "get_scene_names", alloc);  

  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);

  std::string cmd = buffer.GetString();

  strcpy((char *) txBuffer, cmd.c_str());
  if (writePort(txBuffer, cmd.size()) > 0)
    return true;

  return false;
}

bool TcpServer::sendControls()
{
  bool ret = true;

  controlBuffer.Clear();

  controlDoc["throttle"].SetFloat(throttle);
  controlDoc["steering"].SetFloat(steering);
  controlDoc["brake"].SetFloat(brakes);

  rapidjson::Writer<rapidjson::StringBuffer> controlWriter(controlBuffer);
  controlDoc.Accept(controlWriter);

  std::string ctrl = controlBuffer.GetString();

  strcpy((char *) txBuffer, ctrl.c_str());
  writePort(txBuffer, ctrl.size());

  return ret;
}

void TcpServer::enableImagePublishing()
{
  publishImages = true;
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

void TcpServer::registerTelemetryCallback(TelemetryCallback callback)
{
  dataMutex.lock();          
  telemCallback = callback;
  dataMutex.unlock();                             
}


void TcpServer::registerEventCallback(EventCallback callback)
{
  dataMutex.lock();          
  eventCallback = callback;
  dataMutex.unlock();                             
}

void TcpServer::clearTelemetryCallback()
{
  dataMutex.lock();          
  telemCallback = NULL;
  dataMutex.unlock();                             
}

void TcpServer::registerImageCallback(ImageCallback callback)
{
  dataMutex.lock();          
  imageCallback = callback;
  dataMutex.unlock();                             
}

void TcpServer::clearImageCallback()
{
  dataMutex.lock();          
  imageCallback = NULL;
  dataMutex.unlock();                             
}


void TcpServer::clearEventCallback()
{
  dataMutex.lock();          
  eventCallback = NULL;
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
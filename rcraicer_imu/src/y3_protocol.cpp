
#include "../include/rcraicer_imu/y3_protocol.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>

Y3Protocol::Y3Protocol(uint32_t update_freq) : serialPort(NULL)
{
    this->update_freq = update_freq;
    decodeInit();
}

Y3Protocol::~Y3Protocol()
{
    
}

bool Y3Protocol::openPort(std::string portPath, int baudRate)
{
    serialPort = new SerialPort(portPath, baudRate);
    serialPort->registerDataCallback(std::bind(&Y3Protocol::serial_data_callback, this, std::placeholders::_1));    

    if (serialPort->connect())
        return true;

    return false;
}

bool Y3Protocol::closePort()
{
    if (serialPort != NULL && serialPort->isConnected()) {
        serialPort = NULL;
        return true;
    }

    return false;
}

bool Y3Protocol::configure()
{
    bool ret = true;

    // configure response header
    memset(&tx_buf.y3_tx_response_header, 0, sizeof(tx_buf.y3_tx_response_header));

    tx_buf.y3_tx_response_header.header_config = Y3_HEADER_BITS;

    ret = sendMessage(Y3_CMD_RESP_HEADER, (uint8_t*)&tx_buf, sizeof(tx_buf.y3_tx_response_header));      

    if (!ret)    
        return ret;    

    // get calibs
    ret = sendMessage(Y3_CMD_GET_CALIB_ACCEL, NULL, 0);      

    if (!ret)    
        return ret;            

    //configure axis direction
    memset(&tx_buf.y3_tx_axis_direction, 0, sizeof(tx_buf.y3_tx_axis_direction));
    tx_buf.y3_tx_axis_direction.axis_direction = Y3_AXIS_DIRECTION;

    ret = sendMessage(Y3_CMD_AXIS_DIRECTION, (uint8_t*)&tx_buf, sizeof(tx_buf.y3_tx_axis_direction));


    if (!ret)    
        return ret;

    // configure streaming slots
    memset(&tx_buf.y3_tx_slots, 0, sizeof(tx_buf.y3_tx_slots));
    tx_buf.y3_tx_slots.slot1 = Y3_CMD_TARED_ORIENTATION_Q;
    tx_buf.y3_tx_slots.slot2 = Y3_CMD_CORR_SENSOR_DATA;// Y3_CMD_CORR_SENSOR_DATA;
    tx_buf.y3_tx_slots.slot3 = Y3_CMD_TEMP_C;
    tx_buf.y3_tx_slots.slot4 = 0xFF;
    tx_buf.y3_tx_slots.slot5 = 0xFF;
    tx_buf.y3_tx_slots.slot6 = 0xFF;
    tx_buf.y3_tx_slots.slot7 = 0xFF;
    tx_buf.y3_tx_slots.slot8 = 0xFF;

    ret = sendMessage(Y3_CMD_SET_STREAM_SLOTS, (uint8_t*)&tx_buf, sizeof(tx_buf.y3_tx_slots));

    if (!ret)
        return ret;

    // configure timing
    // uint32_t timing = 1000000;
    uint32_t timing = 1000000 / update_freq;
    memset(&tx_buf.y3_tx_timing, 0, sizeof(tx_buf.y3_tx_timing));
    

    tx_buf.y3_tx_timing.interval = Y3Protocol::flipUint32((uint8_t*)&timing);
    tx_buf.y3_tx_timing.duration = 0xFFFFFFFF; // continue idefinitely
    tx_buf.y3_tx_timing.delay = 0;

    ret = sendMessage(Y3_CMD_SET_STREAM_TIMING, (uint8_t*)&tx_buf, sizeof(tx_buf.y3_tx_timing));

    // startStreaming();    

    return ret;

}

bool Y3Protocol::startStreaming()
{    
    bool ret = sendMessage(Y3_CMD_START_STREAM, NULL, 0);
    return ret;
}

bool Y3Protocol::stopStreaming()
{
    bool ret = sendMessage(Y3_CMD_STOP_STREAM, NULL, 0);
    return ret;
}

bool Y3Protocol::sendMessage(const uint8_t msg, const uint8_t *payload, const uint16_t length)
{
    y3_tx_header_t header = {Y3_CMD_WITH_RESP_HEADER, msg};
    uint8_t chksum = 0;

    // Calculate checksum
	calcChecksum(((uint8_t *)&header) + 1, sizeof(header) - 1, chksum); // skip 1 cmd bytes    
    
    if (payload != nullptr) {
		calcChecksum(payload, length, chksum);
	}

    	// Send message
	if (serialPort->writePort((uint8_t *)&header, sizeof(header)) != sizeof(header)) {
		return false;
	}

	if (payload && serialPort->writePort((uint8_t *)payload, length) != length) {
		return false;
	}

	if (serialPort->writePort((uint8_t *)&chksum, sizeof(chksum)) != sizeof(chksum)) {
		return false;
	}

	return true;

}

void Y3Protocol::serial_data_callback(const uint8_t data)
{
    parseChar(data);
}

int Y3Protocol::parseChar(const uint8_t b)
{
    int ret = 0;    

    // std::cout << std::hex << std::setfill('0')  << std::hex << std::setw(2) << static_cast<int>(b) << " : " << decode_state << "\n";

    switch(decode_state)
    {
        case Y3_DECODE_SYNC:
            if (b == Y3_SUCCESS) {
                decode_state = Y3_DECODE_ID;
            }
            break;

        case Y3_DECODE_ID:            
            rx_msg = b;
            decode_state = Y3_DECODE_CHKSUM;
            break;
        case Y3_DECODE_CHKSUM:
            rx_ck = b;
            decode_state = Y3_DECODE_LENGTH;
            break;
        case Y3_DECODE_LENGTH:            
            decode_state = Y3_DECODE_PAYLOAD;
            rx_payload_length = b;

            if (payloadRxInit() != 0){
                decodeInit();
            } else {
                if (rx_payload_length > 0)
                    decode_state = Y3_DECODE_PAYLOAD;
                else
                {
                    if (validateChecksum())
                        ret = payloadRxDone();                    
                    
                    decodeInit();                    
                }                
            }

            break;
        case Y3_DECODE_PAYLOAD:
            addByteToChecksum(b);            
            ret = payloadRxAdd(b);            

            if (ret < 0) {
                decodeInit();
            }
            else if (ret > 0) {
                if (validateChecksum())
                    ret = payloadRxDone();
                
                decodeInit();                
            }

            break;        
    }

    return ret;
}


void Y3Protocol::addByteToChecksum(const uint8_t b)
{
    ck = ck + b;
}

bool Y3Protocol::validateChecksum()
{
    if (ck == rx_ck)
        return true;
    
    return false;    
}
    
void Y3Protocol::decodeInit()
{
    decode_state = Y3_DECODE_SYNC;    
    ck = 0;
    rx_ck = 0;	
	rx_payload_length = 0;
	rx_payload_index = 0;
}

int Y3Protocol::payloadRxInit()  // -1 = abort, 0 = continue
{
    int ret = 0;

    // std::cout << "RxInit " << rx_payload_length << "\r\n";
    // std::cout << "Msg: " << rx_msg << "\r\n";

    rx_state = UBX_RXMSG_HANDLE;	// handle by default

    switch(rx_msg)
    {
        case Y3_CMD_RESP_HEADER:
        case Y3_CMD_SET_STREAM_SLOTS:
        case Y3_CMD_SET_STREAM_TIMING:
        case Y3_CMD_START_STREAM:
        case Y3_CMD_STOP_STREAM:                    
        case Y3_CMD_AXIS_DIRECTION:
            if (rx_payload_length != 0) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;
        case Y3_CMD_GET_CALIB_ACCEL:
            if (rx_payload_length != sizeof(y3_payload_rx_calib_t)){
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else
            {
                rx_state = UBX_RXMSG_HANDLE;
            }
            break;
        case Y3_CMD_GET_STREAM_TIMING:
            if (rx_payload_length != sizeof(y3_payload_rx_timing_t)) {                
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;
        case Y3_CMD_TARED_ORIENTATION_A:
            if (rx_payload_length != sizeof(y3_payload_rx_quaternion_t)) {                
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;
        case Y3_CMD_TEMP_C:
            if (rx_payload_length != sizeof(y3_payload_rx_temperature_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;
        
        case Y3_CMD_STREAM:
            if (rx_payload_length != sizeof(y3_payload_rx_stream_data_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;
            
        case Y3_CMD_RAW_SENSOR_DATA:            
            if (rx_payload_length != sizeof(y3_payload_rx_sensor_data_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }
            else {
                rx_state = UBX_RXMSG_HANDLE;    
            }
            break;                        
        default:
            rx_state = UBX_RXMSG_IGNORE;	// ignore all other messages
            break;
    }

    switch (rx_state)
    {
        case UBX_RXMSG_HANDLE:	// handle message        
            ret = 0;
            break;

        case UBX_RXMSG_IGNORE:	// ignore message
        // std::cout << "Message ignore error\r\n";
            ret = -1;
            break;
        
        case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
            std::cout << "Message length error\r\n";
            ret = -1;	// return error, abort handling this message
            break;

        default:	// invalid message state
            ret = -1;	// return error, abort handling this message
            break;
    }

    return ret;

}

int Y3Protocol::payloadRxAdd(const uint8_t b)
{
    // return 0;

    int ret = 0;

    uint8_t *p_buf = (uint8_t *)&buf;

    p_buf[rx_payload_index] = b;

	if (++rx_payload_index >= rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;

}

int Y3Protocol::payloadRxDone()
{
    int ret = 0;    
    float yValue = 0.0;

    // return if no message handled
	if (rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

    switch (rx_msg){
        case Y3_CMD_RESP_HEADER:
            std::cout << "Y3_CMD_RESP_HEADER" << "\r\n";
            break;
        case Y3_CMD_TARED_ORIENTATION_A:
            std::cout << "Y3_CMD_TARED_ORIENTATION_A" << "\r\n";
            break;

        case Y3_CMD_SET_STREAM_SLOTS:
            std::cout << "Y3_CMD_SET_STREAM_SLOTS" << "\r\n";
            break;
        case Y3_CMD_SET_STREAM_TIMING:
            std::cout << "Y3_CMD_SET_STREAM_TIMING" << "\r\n";
            break;
        case Y3_CMD_START_STREAM:            
            std::cout << "Y3_CMD_START_STREAM" << "\r\n";
            break;
        case Y3_CMD_STOP_STREAM:            
            std::cout << "Y3_CMD_STOP_STREAM" << "\r\n";
            break;
        case Y3_CMD_AXIS_DIRECTION:
            std::cout << "Y3_CMD_AXIS_DIRECTION" << "\r\n";
            break;
        case Y3_CMD_TEMP_C:
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_temperature.temperature);
            std::cout << "Y3_CMD_TEMP_C : " << yValue << "\r\n";
            break;
        case Y3_CMD_GET_STREAM_TIMING:
            std::cout << "Y3_CMD_GET_STREAM_TIMING " << Y3Protocol::flipUint32((uint8_t*)&buf.payload_rx_timing.interval) << ":" << buf.payload_rx_timing.duration << ":" << buf.payload_rx_timing.delay << "\r\n";
            break;
        case Y3_CMD_GET_CALIB_ACCEL:
            std::cout << "Y3_CMD_GET_CALIB_ACCEL" << "\r\n";
            std::cout << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat0_0) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat0_1) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat0_2) << "\r\n";
            std::cout << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat1_0) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat1_1) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat1_2) << "\r\n";
            std::cout << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat2_0) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat2_1) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.mat2_2) << "\r\n";
            std::cout << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.bias1) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.bias2) << ":" << Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_calib.bias3) << "\r\n";
            break;

        case Y3_CMD_RAW_SENSOR_DATA:
            std::cout << "Y3_CMD_RAW_SENSOR_DATA" << "\r\n";
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_sensor_data.accel_x);
            std::cout << " : " << yValue << " : ";
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_sensor_data.accel_y);
            std::cout << " : " << yValue << " : ";
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_sensor_data.accel_z);
            std::cout << " : " << yValue << " : " << "\r\n";
            std::cout << buf.payload_rx_sensor_data.accel_x << "/" << buf.payload_rx_sensor_data.accel_y << "/" << buf.payload_rx_sensor_data.accel_z << "\r\n";

            break;
        case Y3_CMD_STREAM:
            // get temperature
            tempMsg.temperature = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.temp);

            // get magnetic field (convert gauss to tesla)
            magMsg.magnetic_field.x = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.mag_x) / 10000;
            magMsg.magnetic_field.y = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.mag_y) / 10000;
            magMsg.magnetic_field.z = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.mag_z) / 10000;

            // get raw IMU measurements
            imuMsg.orientation.x = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.orient_x);
            imuMsg.orientation.y = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.orient_y);
            imuMsg.orientation.z = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.orient_z);
            imuMsg.orientation.w = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.orient_w);

            imuMsg.linear_acceleration.x = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_x) * GRAVITY;
            imuMsg.linear_acceleration.y = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_y) * GRAVITY;
            imuMsg.linear_acceleration.z = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_z) * GRAVITY;

            imuMsg.angular_velocity.x = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.gyro_x);
            imuMsg.angular_velocity.y = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.gyro_y);
            imuMsg.angular_velocity.z = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.gyro_z);

            if (sensorMessagesCallback != NULL)
                sensorMessagesCallback(imuMsg, magMsg, tempMsg);

            std::cout << "Y3_CMD_STREAM  : " << yValue;
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_x);
            std::cout << " : " << yValue << " : ";
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_y);
            std::cout << " : " << yValue << " : ";
            yValue = Y3Protocol::getFloat((uint8_t*) &buf.payload_rx_stream_data.accel_z);
            std::cout << " : " << yValue << " : " << "\r\n";
            std::cout << " " << buf.payload_rx_stream_data.accel_x << "/" << buf.payload_rx_stream_data.accel_y << "/" << buf.payload_rx_stream_data.accel_z << "\r\n";
            break;


    }

    return ret;
}

void Y3Protocol::registerSensorMessagesCallback(SensorMessagesCallback callback)
{
    sensorMessagesCallback = callback;
}

void Y3Protocol::calcChecksum(const uint8_t *buffer, const uint16_t length, uint8_t &checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum = checksum + buffer[i];		
	}
}
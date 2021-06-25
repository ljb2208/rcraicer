#ifndef Y3PROTOCOL_H_
#define Y3PROTOCOL_H_


#include <stdint.h>
#include "serial_port.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"


#define GRAVITY (9.80665)

#define Y3_SYNC                         0xF8

#define Y3_SUCCESS                      0x00
#define Y3_FAIL                         0x01



#define Y3_CMD                          0xF7
#define Y3_CMD_WITH_RESP_HEADER         0xF9
#define Y3_CMD_MSG_SIZE                 0x02

#define Y3_CMD_STREAM                   0xFF

#define Y3_CMD_TARED_ORIENTATION_Q      0x00
#define Y3_CMD_TARED_ORIENTATION_E      0x01
#define Y3_CMD_TARED_ORIENTATION_R      0x02
#define Y3_CMD_TARED_ORIENTATION_A      0x03

#define Y3_CMD_UNTARED_ORIENTATION_Q    0x06
#define Y3_CMD_UNTARED_ORIENTATION_E    0x07
#define Y3_CMD_UNTARED_ORIENTATION_R    0x08
#define Y3_CMD_UNTARED_ORIENTATION_A    0x09

#define Y3_CMD_NORM_SENSOR_DATA         0x20
#define Y3_CMD_NORM_GYRO                0x21
#define Y3_CMD_NORM_ACCEL               0x22
#define Y3_CMD_NORM_COMPASS             0x23

#define Y3_CMD_CORR_SENSOR_DATA         0x25
#define Y3_CMD_CORR_GYRO                0x26
#define Y3_CMD_CORR_ACCEL               0x27
#define Y3_CMD_CORR_COMPASS             0x28
#define Y3_CMD_CORR_LINEAR_ACCEL        0x29
#define Y3_CMD_CORR_RAW_GYRO            0x30
#define Y3_CMD_CORR_RAW_ACCEL           0x31
#define Y3_CMD_CORR_RAW_COMPASS         0x32

#define Y3_CMD_TEMP_C                   0x2B
#define Y3_CMD_TEMP_F                   0x2C
#define Y3_CMD_CONFIDENCE               0x2D

#define Y3_CMD_RAW_SENSOR_DATA          0x40
#define Y3_CMD_RAW_GYRO                 0x41
#define Y3_CMD_RAW_ACCEL                0x42
#define Y3_CMD_RAW_COMPASS              0x43

#define Y3_CMD_SET_STREAM_SLOTS         0x50
#define Y3_CMD_GET_STREAM_SLOTS         0x51
#define Y3_CMD_SET_STREAM_TIMING        0x52
#define Y3_CMD_GET_STREAM_TIMING        0x53
#define Y3_CMD_GET_STREAM_BATCH         0x54
#define Y3_CMD_START_STREAM             0x55
#define Y3_CMD_STOP_STREAM              0x56
#define Y3_CMD_SET_TIMESTAMP            0x5F

#define Y3_CMD_GET_CALIB_COMPASS        0xA2
#define Y3_CMD_GET_CALIB_ACCEL          0xA3
#define Y3_CMD_GET_CALIB_GYRO           0xA4

#define Y3_CMD_GYRO_CALIBRATE           0xA6

#define Y3_CMD_AXIS_DIRECTION           0x74

#define Y3_CMD_RESP_HEADER              0xDD

#define Y3_CMD_RESET                    0xE2

// #define Y3_HEADER_BITS                  0x0000004F // set checksum, success/failure, command excho and data length
#define Y3_HEADER_BITS                  0x4D000000 // set checksum, success/failure, command echo and data length

#define Y3_AXIS_DIRECTION               0b00010011 // x - forward, y - left, z - up 

#define _swap(a,b,c) c=a;a=b;b=c;


/* Decoder state */
typedef enum {
	Y3_DECODE_SYNC = 0,	
	Y3_DECODE_ID,
    Y3_DECODE_LENGTH,    
	Y3_DECODE_PAYLOAD,
	Y3_DECODE_CHKSUM
} y3_decode_state_t;

/* Rx message state */
typedef enum {
	UBX_RXMSG_IGNORE = 0,
	UBX_RXMSG_HANDLE,	
	UBX_RXMSG_ERROR_LENGTH
} y3_rxmsg_state_t;



typedef struct {
	uint32_t temperature;
} y3_payload_rx_temperature_t;

typedef struct {
	uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t w;
} y3_payload_rx_quaternion_t;

typedef struct {
	uint32_t x;
    uint32_t y;
    uint32_t z;    
} y3_payload_rx_3dvector_t;

typedef struct {
    uint32_t orient_x;
    uint32_t orient_y;
    uint32_t orient_z;
    uint32_t orient_w;
    uint32_t gyro_x;
    uint32_t gyro_y;
    uint32_t gyro_z;
    uint32_t accel_x;
    uint32_t accel_y;
    uint32_t accel_z;
    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;    
    uint32_t temp;
} y3_payload_rx_stream_data_t;

typedef struct {
    uint32_t gyro_x;
    uint32_t gyro_y;
    uint32_t gyro_z;
    uint32_t accel_x;
    uint32_t accel_y;
    uint32_t accel_z;
    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;        
} y3_payload_rx_sensor_data_t;

typedef struct {
    uint32_t interval;
    uint32_t duration;
    uint32_t delay;

} y3_payload_rx_timing_t;

typedef struct {
    uint32_t mat0_0;
    uint32_t mat0_1;
    uint32_t mat0_2;
    uint32_t mat1_0;
    uint32_t mat1_1;
    uint32_t mat1_2;
    uint32_t mat2_0;
    uint32_t mat2_1;
    uint32_t mat2_2;
    uint32_t bias1;
    uint32_t bias2;
    uint32_t bias3;
} y3_payload_rx_calib_t;


/* General message and payload buffer union */
typedef union {
	y3_payload_rx_temperature_t         payload_rx_temperature;	
    y3_payload_rx_quaternion_t          payload_rx_orientation_q;	
    y3_payload_rx_3dvector_t            payload_rx_gyro;
    y3_payload_rx_stream_data_t         payload_rx_stream_data;
    y3_payload_rx_sensor_data_t         payload_rx_sensor_data;
    y3_payload_rx_timing_t              payload_rx_timing;
    y3_payload_rx_calib_t               payload_rx_calib;

} y3_rx_buf_t;



typedef struct {
    uint8_t cmd_sync;
    uint8_t cmd;
} y3_tx_header_t;

typedef struct {    
    uint8_t slot1;
    uint8_t slot2;
    uint8_t slot3;
    uint8_t slot4;
    uint8_t slot5;
    uint8_t slot6;
    uint8_t slot7;
    uint8_t slot8;
} y3_tx_slots_t;


typedef struct {
    uint32_t interval;
    uint32_t duration;
    uint32_t delay;

} y3_tx_timing_t;

typedef struct {
    uint8_t axis_direction;
} y3_tx_axis_direction_t;

typedef struct {
    uint32_t header_config;
} y3_tx_response_header_t;

typedef union {
    y3_tx_slots_t y3_tx_slots;
    y3_tx_timing_t y3_tx_timing;
    y3_tx_response_header_t y3_tx_response_header;
    y3_tx_axis_direction_t y3_tx_axis_direction;  
} y3_tx_buf_t;

typedef union {
    float f;
    uint32_t ul;
} y3_float_t;




class Y3Protocol
{
    public:
        Y3Protocol(uint32_t update_freq);
        ~Y3Protocol();

        bool openPort(std::string serialPort, int baudRate);
        bool closePort();

        typedef std::function<void(sensor_msgs::msg::Imu, sensor_msgs::msg::MagneticField, sensor_msgs::msg::Temperature)> SensorMessagesCallback;        

        void registerSensorMessagesCallback(SensorMessagesCallback callback);        

        bool configure();
        bool startStreaming();
        bool stopStreaming();

    private:
        int parseChar(const uint8_t b);
        void addByteToChecksum(const uint8_t b);
        void decodeInit();
        int payloadRxInit();
        int payloadRxAdd(const uint8_t b);        
        int payloadRxDone();                

        void calcChecksum(const uint8_t *buffer, const uint16_t length, uint8_t &checksum);
        bool validateChecksum();

        void serial_data_callback(const uint8_t data);

        bool sendMessage(const uint8_t msg, const uint8_t *payload, const uint16_t length);

        static float getFloat(const uint8_t* value)
        {
            y3_float_t flvalue;
            flvalue.ul = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];

            return flvalue.f;

        }        

        static uint32_t flipUint32(const uint8_t* value)
        {
            uint32_t newValue = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
            return newValue;
        }

        

        y3_decode_state_t  decode_state{};
        y3_rxmsg_state_t   rx_state{UBX_RXMSG_IGNORE};
        uint8_t rx_msg{0};
        uint16_t rx_payload_index{0};
        uint16_t rx_payload_length{0};                
        uint8_t ck{0};
        uint8_t rx_ck{0};


        y3_rx_buf_t  buf{};
        y3_tx_buf_t  tx_buf{};
        SerialPort* serialPort;

        uint32_t update_freq;

        SensorMessagesCallback sensorMessagesCallback;

        sensor_msgs::msg::Temperature tempMsg;      
        sensor_msgs::msg::Imu imuMsg;
        sensor_msgs::msg::MagneticField magMsg;


};


#endif
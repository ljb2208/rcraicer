#ifndef Y3PROTOCOL_H_
#define Y3PROTOCOL_H_


#include <stdint.h>
#include "serial_port.h"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"


#define Y3_SYNC                         0xF8

#define Y3_CMD                          0xF7
#define Y3_CMD_WITH_RESP_HEADER         0xF9
#define Y3_CMD_MSG_SIZE                 0x03

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

/* General message and payload buffer union */
typedef union {
	y3_payload_rx_temperature_t         payload_rx_temperature;	
    y3_payload_rx_quaternion_t          payload_rx_orientation_q;	
    y3_payload_rx_3dvector_t            payload_rx_gyro;

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

typedef union {
    y3_tx_slots_t y3_tx_slots;
    y3_tx_timing_t y3_tx_timing;
} y3_tx_buf_t;


class Y3Protocol
{
    public:
        Y3Protocol();
        ~Y3Protocol();

        bool openPort(std::string serialPort, int baudRate);
        bool closePort();

        typedef std::function<void(sensor_msgs::msg::Imu)> ImuMessageCallback;
        typedef std::function<void(sensor_msgs::msg::Imu)> ImuRawMessageCallback;
        typedef std::function<void(sensor_msgs::msg::MagneticField)> MagMessageCallback;
        typedef std::function<void(sensor_msgs::msg::Temperature)> TempMessageCallback;

        void registerImuMessageCallback(ImuMessageCallback callback);
        void registerImuRawMessageCallback(ImuRawMessageCallback callback);
        void registerMagMessageCallback(MagMessageCallback callback);
        void registerTempMessageCallback(TempMessageCallback callback);

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

        void calcChecksum(const uint8_t *buffer, const uint16_t length, uint8_t *checksum);

        void serial_data_callback(const uint8_t data);

        bool sendMessage(const uint8_t msg, const uint8_t *payload, const uint16_t length);

        y3_decode_state_t  decode_state{};
        y3_rxmsg_state_t   rx_state{UBX_RXMSG_IGNORE};
        uint16_t rx_msg{};
        uint16_t rx_payload_index{0};
        uint16_t rx_payload_length{0};                
        uint8_t rx_ck{0};

        y3_rx_buf_t  buf{};
        y3_tx_buf_t  tx_buf{};
        SerialPort* serialPort;

        ImuMessageCallback imuMessageCallback;
        ImuRawMessageCallback imuRawMessageCallback;
        MagMessageCallback magMessageCallback;
        TempMessageCallback tempMessageCallback;
};


#endif
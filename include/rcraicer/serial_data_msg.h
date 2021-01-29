//#include <cstdint>
#ifdef AVR // or whatever -- check the compiler docs, I don't know the standard way to check this offhand
# define NO_CSTDINT 1  // AVR arduino has no <cstdint>; but we're coding to portable C++. So substitute.
#endif

// unless we know otherwise, use the compiler's <cstdint>
#ifndef NO_CSTDINT
# include <cstdint>
#else
// no <cstdint> -- make sure std:: contains the things we need.
# include <stdint.h>
#endif

const uint8_t SERVO_MSG = 1;
const uint8_t COMMAND_MSG = 2;
const uint8_t ENCODER_MSG = 3;
const uint8_t ARDUINO_STATUS_MSG = 4;
const uint8_t IMU_MSG = 5;

const uint8_t MESSAGE_DELIM = 0x7E;
const uint8_t MAX_BUFFER = 100;

const uint8_t MSG_MSG_LEN = 24;

const uint8_t STATUS_OK = 0;
const uint8_t STATUS_DEGRADED = 1;
const uint8_t STATUS_ERROR = 2;

struct __attribute__ ((__packed__)) data_msg {
  uint8_t msg_type;
  uint8_t msg_len;
  uint8_t msg[MSG_MSG_LEN];  
  uint8_t crc1;
  uint8_t crc2;
};

const uint8_t MSG_SIZE = sizeof(data_msg);
const uint8_t MSG_SIZE_WITH_DELIM = MSG_SIZE + 2;

struct __attribute__ ((__packed__)) servo_msg {
  int32_t steer;
  int32_t throttle;
//   int32_t blank1;
//   int32_t blank2;
};

struct __attribute__ ((__packed__)) command_msg {
    uint8_t armed;
    // uint8_t blank1;
    // uint16_t blank2;    
    // int32_t blank3;    
    // int32_t blank4;
    // int32_t blank5;
};

struct __attribute__ ((__packed__)) encoder_msg {
    int32_t left_rear;
    int32_t left_front;
    int32_t right_front;
    int32_t right_rear;
};

struct __attribute__ ((__packed__)) arduino_status_msg {
    uint16_t servo_update_count;
    uint16_t encoder_msg_count;
    uint16_t main_loop_count;
    uint16_t main_loop_max;
    uint8_t armed;
    uint8_t status;
    uint16_t crc_error;    
    uint16_t unknown_msg;
    // uint16_t blank1;    
};

struct __attribute__ ((__packed__)) arduino_imu_msg {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};


void getCRCBit(uint16_t& crc)
{
    for (unsigned char bit = 0; bit < 8; bit++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } 
        else {
            crc = crc << 1;
        }
    }
}


uint16_t getCRC(data_msg msg)
{    
    uint16_t crc = 0;

    uint8_t byte = msg.msg_type;
    crc = crc ^ (byte << 8);

    getCRCBit(crc);    
    
    byte = msg.msg_len;
    crc = crc ^ (byte << 8);

    getCRCBit(crc);
    
    for (uint8_t i=0; i < sizeof(msg.msg); i++)
    {
        byte = msg.msg[i];
        crc = crc ^ (byte << 8);

        getCRCBit(crc);
    }    

    return crc;
}

bool validateCRC(data_msg msg)
{
    uint16_t sent_crc = (uint16_t) msg.crc1 << 8 | (uint16_t) msg.crc2;

    uint16_t crc = getCRC(msg);

    if (sent_crc == crc)
        return true;
        
    return false;    
}


void appendCRC(data_msg& msg)
{
    uint16_t crc = getCRC(msg);

    msg.crc1 = crc >> 8;
    msg.crc2 = crc;    
}


void fillMsg(data_msg& msg, uint8_t structSize)
{
    for (uint8_t i= structSize; i < MSG_MSG_LEN; i++)
    {
        msg.msg[i] = 0;
    }
}

void packServoMessage(servo_msg smsg, data_msg& msg)
{
    msg.msg_type = SERVO_MSG;
    msg.msg_len = MSG_SIZE;

    msg.msg[0] = smsg.steer >> 24;
    msg.msg[1] = smsg.steer >> 16;
    msg.msg[2] = smsg.steer >> 8;
    msg.msg[3] = smsg.steer;

    msg.msg[4] = smsg.throttle >> 24;
    msg.msg[5] = smsg.throttle >> 16;
    msg.msg[6] = smsg.throttle >> 8;
    msg.msg[7] = smsg.throttle;

    fillMsg(msg, sizeof(servo_msg));
    appendCRC(msg);
}


void packEncoderMessage(encoder_msg enc_msg, data_msg& msg)
{
    msg.msg_type = ENCODER_MSG;
    msg.msg_len = MSG_SIZE;
    msg.msg[0] = enc_msg.left_rear >> 24;
    msg.msg[1] = enc_msg.left_rear >> 16;
    msg.msg[2] = enc_msg.left_rear >> 8;
    msg.msg[3] = enc_msg.left_rear;

    msg.msg[4] = enc_msg.left_front >> 24;
    msg.msg[5] = enc_msg.left_front >> 16;
    msg.msg[6] = enc_msg.left_front >> 8;
    msg.msg[7] = enc_msg.left_front;

    msg.msg[8] = enc_msg.right_front >> 24;
    msg.msg[9] = enc_msg.right_front >> 16;
    msg.msg[10] = enc_msg.right_front >> 8;
    msg.msg[11] = enc_msg.right_front;

    msg.msg[12] = enc_msg.right_rear >> 24;
    msg.msg[13] = enc_msg.right_rear >> 16;
    msg.msg[14] = enc_msg.right_rear >> 8;
    msg.msg[15] = enc_msg.right_rear;    

    fillMsg(msg, sizeof(encoder_msg));
    appendCRC(msg);
}

void packArduinoStatusMessage(arduino_status_msg smsg, data_msg& msg)
{
    msg.msg_type = ARDUINO_STATUS_MSG;
    msg.msg_len = MSG_SIZE;
    msg.msg[0] = smsg.servo_update_count >> 8;
    msg.msg[1] = smsg.servo_update_count;
    msg.msg[2] = smsg.encoder_msg_count >> 8;
    msg.msg[3] = smsg.encoder_msg_count;
    msg.msg[4] = smsg.main_loop_count >> 8;
    msg.msg[5] = smsg.main_loop_count;
    msg.msg[6] = smsg.main_loop_max >> 8;
    msg.msg[7] = smsg.main_loop_max;
    msg.msg[8] = smsg.armed;
    msg.msg[9] = smsg.status;    
    msg.msg[10] = smsg.crc_error >> 8;
    msg.msg[11] = smsg.crc_error;
    msg.msg[12] = smsg.unknown_msg >> 8;
    msg.msg[13] = smsg.unknown_msg;    

    fillMsg(msg, sizeof(arduino_status_msg));

    appendCRC(msg);
}

void packCommandMessage(command_msg cmsg, data_msg& msg)
{
    msg.msg_type = COMMAND_MSG;
    msg.msg_len = MSG_SIZE;
    msg.msg[0] = cmsg.armed;    

    fillMsg(msg, sizeof(command_msg));
    appendCRC(msg);
}

void packIMUMessage(arduino_imu_msg imsg, data_msg& msg)
{
    msg.msg_type = IMU_MSG;
    msg.msg_len = MSG_SIZE;

    unsigned char* fval = (unsigned char*) &imsg.accel_x;
    msg.msg[0] = fval[0];
    msg.msg[1] = fval[1];
    msg.msg[2] = fval[2];
    msg.msg[3] = fval[3];

    fval = (unsigned char*) &imsg.accel_y;
    msg.msg[4] = fval[0];
    msg.msg[5] = fval[1];
    msg.msg[6] = fval[2];
    msg.msg[7] = fval[3];

    fval = (unsigned char*) &imsg.accel_z;
    msg.msg[8] = fval[0];
    msg.msg[9] = fval[1];
    msg.msg[10] = fval[2];
    msg.msg[11] = fval[3];

    fval = (unsigned char*) &imsg.gyro_x;
    msg.msg[12] = fval[0];
    msg.msg[13] = fval[1];
    msg.msg[14] = fval[2];
    msg.msg[15] = fval[3];

    fval = (unsigned char*) &imsg.gyro_y;
    msg.msg[16] = fval[0];
    msg.msg[17] = fval[1];
    msg.msg[18] = fval[2];
    msg.msg[19] = fval[3];

    fval = (unsigned char*) &imsg.gyro_z;
    msg.msg[20] = fval[0];
    msg.msg[21] = fval[1];
    msg.msg[22] = fval[2];
    msg.msg[23] = fval[3];

    fillMsg(msg, sizeof(arduino_imu_msg));
    appendCRC(msg);
}


void unpackEncoderMessage(data_msg msg, encoder_msg& enc_msg)
{    
    enc_msg.left_rear = (int32_t)msg.msg[0]<<24 | (int32_t)msg.msg[1]<<16 | (int32_t)msg.msg[2]<<8 | (int32_t)msg.msg[3];    
    enc_msg.left_front = (int32_t)msg.msg[4]<<24 | (int32_t)msg.msg[5]<<16 | (int32_t)msg.msg[6]<<8 | (int32_t)msg.msg[7];    
    enc_msg.right_front = (int32_t)msg.msg[8]<<24 | (int32_t)msg.msg[9]<<16 | (int32_t)msg.msg[10]<<8 | (int32_t)msg.msg[11];    
    enc_msg.right_rear = (int32_t)msg.msg[12]<<24 | (int32_t)msg.msg[13]<<16 | (int32_t)msg.msg[14]<<8 | (int32_t)msg.msg[15];        
}

void unpackServoMessage(data_msg msg, servo_msg& smsg)
{
    smsg.steer = (int32_t) msg.msg[0]<<24 | (int32_t)msg.msg[1] << 16 | (int32_t)msg.msg[2] << 8 | (int32_t)msg.msg[3];
    smsg.throttle = (int32_t) msg.msg[4]<<24 | (int32_t)msg.msg[5] << 16 | (int32_t)msg.msg[6] << 8 | (int32_t)msg.msg[7];    
}

void unpackArduinoStatusMessage(data_msg msg, arduino_status_msg& smsg)
{
    smsg.servo_update_count = (uint16_t) msg.msg[0] << 8 | (uint16_t) msg.msg[1];
    smsg.encoder_msg_count = (uint16_t) msg.msg[2] << 8 | (uint16_t) msg.msg[3];
    smsg.main_loop_count = (uint16_t) msg.msg[4] << 8 | (uint16_t) msg.msg[5];
    smsg.main_loop_max = (uint16_t) msg.msg[6] << 8 | (uint16_t) msg.msg[7];
    smsg.armed = msg.msg[8];
    smsg.status = msg.msg[9];
    smsg.crc_error = (uint16_t) msg.msg[10] << 8 | (uint16_t) msg.msg[11];
    smsg.unknown_msg = (uint16_t) msg.msg[12] << 8 | (uint16_t) msg.msg[13];
}

void unpackCommandMessage(data_msg msg, command_msg& cmsg)
{
    cmsg.armed = msg.msg[0];         
}

float getFloatFromByteArray(uint8_t* data, uint8_t startIndex)
{
    float fval = 0.0;
    memcpy(&fval, (void*) &data[startIndex], sizeof(float));
    return fval;
}

void unpackIMUMessage(data_msg msg, arduino_imu_msg& imsg)
{    

    imsg.accel_x = getFloatFromByteArray(msg.msg, 0);
    imsg.accel_y = getFloatFromByteArray(msg.msg, 4);
    imsg.accel_z = getFloatFromByteArray(msg.msg, 8);
    imsg.gyro_x = getFloatFromByteArray(msg.msg, 12);
    imsg.gyro_y = getFloatFromByteArray(msg.msg, 16);
    imsg.gyro_z = getFloatFromByteArray(msg.msg, 20);
}

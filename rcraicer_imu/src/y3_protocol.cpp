
#include "../include/rcraicer_imu/y3_protocol.h"
#include <iostream>
#include <cstring>

Y3Protocol::Y3Protocol() : serialPort(NULL)
{
    
}

Y3Protocol::~Y3Protocol()
{
    
}

bool Y3Protocol::openPort(std::string portPath, int baudRate)
{
    serialPort = new SerialPort(portPath, baudRate);
    serialPort->registerDataCallback(std::bind(&Y3Protocol::serial_data_callback, this, std::placeholders::_1));    

    if (serialPort->isConnected())
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
    // configure streaming slots
    memset(&tx_buf.y3_tx_slots, 0, sizeof(tx_buf.y3_tx_slots));
    tx_buf.y3_tx_slots.slot1 = 0xFF;
    tx_buf.y3_tx_slots.slot2 = 0xFF;
    tx_buf.y3_tx_slots.slot3 = 0xFF;
    tx_buf.y3_tx_slots.slot4 = 0xFF;
    tx_buf.y3_tx_slots.slot5 = 0xFF;
    tx_buf.y3_tx_slots.slot6 = 0xFF;
    tx_buf.y3_tx_slots.slot7 = 0xFF;
    tx_buf.y3_tx_slots.slot8 = 0xFF;

    bool ret = sendMessage(Y3_CMD_SET_STREAM_SLOTS, (uint8_t*)&tx_buf, Y3_CMD_MSG_SIZE + sizeof(tx_buf.y3_tx_slots));

    if (!ret)
        return ret;

    // configure timing
    memset(&tx_buf.y3_tx_timing, 0, sizeof(tx_buf.y3_tx_timing));
    tx_buf.y3_tx_timing.interval = 0;
    tx_buf.y3_tx_timing.duration = 0xFFFFFFFF; // continue idefinitely
    tx_buf.y3_tx_timing.delay = 0;

    ret = sendMessage(Y3_CMD_SET_STREAM_TIMING, (uint8_t*)&tx_buf, Y3_CMD_MSG_SIZE + sizeof(tx_buf.y3_tx_timing));

    return ret;

}

bool Y3Protocol::startStreaming()
{    
    bool ret = sendMessage(Y3_CMD_START_STREAM, NULL, Y3_CMD_MSG_SIZE);
    return ret;
}

bool Y3Protocol::stopStreaming()
{
    bool ret = sendMessage(Y3_CMD_STOP_STREAM, NULL, Y3_CMD_MSG_SIZE);
    return ret;
}

bool Y3Protocol::sendMessage(const uint8_t msg, const uint8_t *payload, const uint16_t length)
{
    y3_tx_header_t header = {Y3_CMD, msg};
    uint8_t chksum = 0;

    // Calculate checksum
	calcChecksum(((uint8_t *)&header) + 1, sizeof(header) - 1, &chksum); // skip 1 cmd bytes
    
    if (payload != nullptr) {
		calcChecksum(payload, length, &chksum);
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

    switch(decode_state)
    {
        case Y3_DECODE_SYNC:
            if (b == Y3_SYNC) {
                decode_state = Y3_DECODE_ID;
            }
            break;

        case Y3_DECODE_ID:
            addByteToChecksum(b);
            decode_state = Y3_DECODE_LENGTH;
            break;
        case Y3_DECODE_LENGTH:
            addByteToChecksum(b);
            decode_state = Y3_DECODE_PAYLOAD;
            rx_payload_length = b;

            if (payloadRxInit() != 0){
                decodeInit();
            } else {
                decode_state = (rx_payload_length > 0) ? Y3_DECODE_PAYLOAD : Y3_DECODE_CHKSUM;
            }

            break;
        case Y3_DECODE_PAYLOAD:
            addByteToChecksum(b);
            ret = payloadRxAdd(b);

            if (ret < 0) {
                decodeInit();
            }
            else if (ret > 0) {
                decode_state = Y3_DECODE_CHKSUM;
            }

            break;
        case Y3_DECODE_CHKSUM:
            if (rx_ck != b) {
                ret = payloadRxDone();
            }

            decodeInit();
            break;
    }

    return ret;
}


void Y3Protocol::addByteToChecksum(const uint8_t b)
{
    rx_ck = rx_ck + b;
}
    
void Y3Protocol::decodeInit()
{
    decode_state = Y3_DECODE_SYNC;
    rx_ck = 0;	
	rx_payload_length = 0;
	rx_payload_index = 0;
}

int Y3Protocol::payloadRxInit()  // -1 = abort, 0 = continue
{
    int ret = 0;

    rx_state = UBX_RXMSG_HANDLE;	// handle by default

    switch(rx_msg)
    {
        default:
            rx_state = UBX_RXMSG_IGNORE;	// ignore all other messages
            break;
    }

    switch (rx_state)
    {
        case UBX_RXMSG_HANDLE:	// handle message
        case UBX_RXMSG_IGNORE:	// ignore message but don't report error
            ret = 0;
            break;
        
        case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
            ret = -1;	// return error, abort handling this message
            break;

        default:	// invalid message state
            ret = -1;	// return error, abort handling this message
            break;
    }

}

int Y3Protocol::payloadRxAdd(const uint8_t b)
{
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

    // return if no message handled
	if (rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

    switch (rx_msg){
        case Y3_CMD_TARED_ORIENTATION_Q:
            break;

    }
}

void Y3Protocol::registerImuMessageCallback(ImuMessageCallback callback)
{
    imuMessageCallback = callback;
}

void Y3Protocol::registerImuRawMessageCallback(ImuRawMessageCallback callback)
{
    imuRawMessageCallback = callback;
}

void Y3Protocol::registerMagMessageCallback(MagMessageCallback callback)
{
    magMessageCallback = callback;
}

void Y3Protocol::registerTempMessageCallback(TempMessageCallback callback)
{
    tempMessageCallback = callback;
}

void Y3Protocol::calcChecksum(const uint8_t *buffer, const uint16_t length, uint8_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum = checksum + buffer[i];		
	}
}
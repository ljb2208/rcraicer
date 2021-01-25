#include <Servo.h>
#include "/home/lbarnett/ros2_ws/src/rcraicer/include/rcraicer/serial_data_msg.h"

// encoder variables
int LR_PIN = 4;
int LF_PIN = 5;
int RF_PIN = 6;
int RR_PIN = 7;

bool encoderSet[4] = {false, false, false, false};
int32_t encoderTicks[4] = {0, 0, 0, 0};

Servo steeringServo;
Servo throttleServo;

const int STEER_PIN = 9;
const int THROTTLE_PIN = 11;

int steerPos = 1500;
int throttlePos = 1500;
int priorThrottlePos = 1500;

const int STEER_POS_DEFAULT = 1500;
const int THROTTLE_POS_DEFAULT =1500;

const int8_t FORWARD = 1;
const int8_t REVERSE = -1;

int8_t direction = FORWARD;

char serialBuffer[MAX_BUFFER];
int serialBufferPtr = 0;

unsigned long SERVO_TIMER = 20; // 50 HZ Updates for servos
unsigned long servoTimerStart= 0;
bool servoTimerRunning = false;

unsigned long ENCODER_TIMER = 200; // 5 HZ Updates for encoder
unsigned long encoderTimerStart= 0;
bool encoderTimerRunning = false;

unsigned long STATUS_TIMER = 1000; // 1 HZ Updates for status
unsigned long statusTimerStart= 0;
bool statusTimerRunning = false;

unsigned long loopTimer = 0;
unsigned long loopDuration = 0;
unsigned long shortestLoopDuration = 10000;
unsigned long longestLoopDuration = 0;

uint16_t statusServoUpdates = 0;
uint16_t statusEncoderMsgs = 0;
uint16_t statusMainLoopMax = 0;
volatile uint32_t statusMainLoopCount = 0;
uint16_t statusInvalidCRC = 0;
uint16_t statusInvalidMSG = 0;

bool isArmed = false;
bool isOk = true;

unsigned long lastMove = 0;
bool lockThrottle = false;


void setup() {
  // set serial
  Serial.begin(115200);

  // set servos
  steeringServo.attach(STEER_PIN);
  throttleServo.attach(THROTTLE_PIN);

  // set digital pins for encoders
  pinMode(LR_PIN, INPUT);
  pinMode(LF_PIN, INPUT);
  pinMode(RF_PIN, INPUT);
  pinMode(RR_PIN, INPUT);

  // start servo timer
  servoTimerStart = millis();
  servoTimerRunning = true;

  // start encoder timer
  encoderTimerStart = millis();
  encoderTimerRunning = true;

  // start status timer
  statusTimerStart = millis();
  statusTimerRunning = true;    
}

void loop() {
    loopTimer = millis();   
    
    readSerial();
//
//    steeringServo.writeMicroseconds(steerPos);
//    steerPos += 100;
//
//    if (steerPos > 1900)
//      steerPos = 1200;
//
//     delay(1000);

//    sendServoControl();

    // if throttle locked (i.e. forward/reverse change requested, ensure that no movement for 
    if (lockThrottle)
    {
      if ((millis() - lastMove) > 500)
      {
        lockThrottle = false;
      }
    }

    if (servoTimerRunning && ((millis() - servoTimerStart) >= SERVO_TIMER))
    {
      //reset timer      
      servoTimerStart = millis();
      sendServoControl();
    }

    if (encoderTimerRunning && ((millis() - encoderTimerStart) >= ENCODER_TIMER))
    {
      encoderTimerStart = millis();
      sendEncoderMessage();
    }

    if (statusTimerRunning && ((millis() - statusTimerStart) >= STATUS_TIMER))
    {
      statusTimerStart = millis();
      sendStatusMessage();
    }

    loopDuration = millis() - loopTimer;

    if (loopDuration > longestLoopDuration)
    {
      statusMainLoopMax = loopDuration;
    }
    
    statusMainLoopCount++;
}

void readSerial()
{
  while (Serial.available() > 0)
  {
    uint8_t data = Serial.read();
    serialBuffer[serialBufferPtr] = data;
    serialBufferPtr++;  

    if (serialBufferPtr == MAX_BUFFER)
    {
      serialBufferPtr = 0;
    }

    if (data == MESSAGE_DELIM)
    {
      if (serialBufferPtr > 1 && serialBuffer[serialBufferPtr - 2] == MESSAGE_DELIM)
      {
        processSerialMsg();
        serialBufferPtr = 0;
      }        
    }    
  }
}

void sendServoControl()
{
  if (isArmed)
  {
    sendServoControl(STEER_PIN, steerPos);    
    sendServoControl(THROTTLE_PIN, throttlePos);
  }
  else
  {
    sendServoControl(STEER_PIN, STEER_POS_DEFAULT);
    sendServoControl(THROTTLE_PIN, THROTTLE_POS_DEFAULT);
  }

  statusServoUpdates++;
}

void readEncoderValues()
{
  bool isMoving = readEncoderValue(LR_PIN, 0);  
  isMoving &= readEncoderValue(LF_PIN, 1);
  isMoving &= readEncoderValue(RF_PIN, 2);
  isMoving &= readEncoderValue(RR_PIN, 3);

  if (isMoving)
    lastMove = millis();
}

bool readEncoderValue(int pin, int index)
{
  bool isMoving = false;
  int val = digitalRead(pin);

  if (val == 0 && !encoderSet[index])
  {
    encoderSet[index] = true;
    isMoving = true;
  }
  else if (val == 1 && encoderSet[index])
  {
    encoderSet[index] = false;
    encoderTicks[index]+= direction;
    isMoving = true;
  } 

  return isMoving;
}

void sendEncoderMessage()
{
  encoder_msg enc_msg;
  enc_msg.left_rear = steerPos;
  enc_msg.left_front = 0;
  enc_msg.right_front = statusMainLoopCount;
  enc_msg.right_rear = throttlePos;

  data_msg dmsg;
  packEncoderMessage(enc_msg, dmsg);    

  uint8_t serial_data[MSG_SIZE+2];
  memcpy(serial_data, &dmsg, MSG_SIZE);
  serial_data[MSG_SIZE] = MESSAGE_DELIM;
  serial_data[MSG_SIZE+1] = MESSAGE_DELIM;
  Serial.write(serial_data, MSG_SIZE_WITH_DELIM);

  statusEncoderMsgs++;
}

void sendStatusMessage()
{  
  arduino_status_msg status_msg;
  status_msg.servo_update_count = statusServoUpdates; 
  status_msg.encoder_msg_count = statusEncoderMsgs;
  status_msg.main_loop_count = statusMainLoopCount;
  status_msg.main_loop_max = statusMainLoopMax;  
  status_msg.armed = isArmed;
  status_msg.status = getStatus();
  status_msg.crc_error = statusInvalidCRC;
  status_msg.unknown_msg = statusInvalidMSG;

  data_msg dmsg;
  packArduinoStatusMessage(status_msg, dmsg);  

  uint8_t serial_data[MSG_SIZE+2];
  memcpy(serial_data, &dmsg, MSG_SIZE);
  serial_data[MSG_SIZE] = MESSAGE_DELIM;
  serial_data[MSG_SIZE+1] = MESSAGE_DELIM;
  Serial.write(serial_data, MSG_SIZE_WITH_DELIM);

  statusServoUpdates = 0;
  statusEncoderMsgs = 0; 
  statusMainLoopMax = 0;
  statusMainLoopCount = 0;
}

uint8_t getStatus()
{
  if (statusServoUpdates == 0 || statusMainLoopCount < 200)
  {
    return STATUS_ERROR;
  }

   if (statusServoUpdates < 40 || statusEncoderMsgs < 3 || statusMainLoopCount < 500)
   {
    return STATUS_DEGRADED;
   }

   return STATUS_OK;
}

void processServoMsg(servo_msg& msg)
{
  steerPos = msg.steer;
  throttlePos = msg.throttle;    

  if (msg.throttle < 1500 && priorThrottlePos >= 1500)
  {
    direction = REVERSE;
    lockThrottle = true;
    throttlePos = 1500;
  }
  else if (msg.throttle >= 1500 && priorThrottlePos < 1500)
  {
    direction = FORWARD;
    lockThrottle = true;
    throttlePos = 1500;
  }  

  priorThrottlePos = msg.throttle;
  
}

void processSerialMsg()
{
  data_msg msg;
  memcpy(&msg, serialBuffer, MSG_SIZE);

  if (!validateCRC(msg))
  {    
    if (statusInvalidCRC == 65535)
    {
      statusInvalidCRC = 0;
    }
      
    statusInvalidCRC++;
    return;
  }

  switch (msg.msg_type)
  {
    case SERVO_MSG:
    {
      servo_msg sm;
      unpackServoMessage(msg, sm);
      processServoMsg(sm);
      break;  
    }
    case COMMAND_MSG:
    {
      command_msg cm;    
      unpackCommandMessage(msg, cm);
      processCommandMsg(cm);
    }
    default:
    {
      if (statusInvalidMSG == 65536)
      {
        statusInvalidMSG = 0;      
      }
      
      statusInvalidMSG++;
      break;
    }
  }  
}


void processCommandMsg(command_msg& msg)
{
  if (msg.armed == 1)
  {
    if (isOk)
    {
      isArmed = true;
    }
  }
  else
  {
    isArmed = false;
  }  
}

void sendServoControl(int servo, int value)
{
  if (servo == STEER_PIN)
  {
    steeringServo.writeMicroseconds(value);
  }

  if (servo == THROTTLE_PIN)
  {
    throttleServo.writeMicroseconds(value);
  }
}

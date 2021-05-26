#include <Servo.h>
#include "/home/lbarnett/ros2_ws/src/rcraicer/include/rcraicer/serial_data_msg.h"
#include <Arduino_LSM6DS3.h>


#define MAX_MEASUREMENT_AGE 400  ///< Zero the reading at 400 ms

// encoder variables
int LR_PIN = 11;
int LF_PIN = 10;
int RF_PIN = 3;
int RR_PIN = 2;

int32_t lrEncoderTicks = 0;
int32_t lfEncoderTicks = 0;
int32_t rfEncoderTicks = 0;
int32_t rrEncoderTicks = 0;

unsigned long lastEncoderTick = 0;

Servo steeringServo;
Servo throttleServo;

const int STEER_PIN = 17;
const int THROTTLE_PIN = 19;

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

unsigned long WHEEL_SPEED_TIMER = 20; // 50 HZ Updates for encoder
unsigned long wheelSpeedTimerStart= 0;
bool wheelSpeedTimerRunning = false;

unsigned long ENCODER_TIMER = 20; // 50 HZ Updates for encoder
unsigned long encoderTimerStart= 0;
bool encoderTimerRunning = false;

unsigned long STATUS_TIMER = 1000; // 1 HZ Updates for status
unsigned long statusTimerStart= 0;
bool statusTimerRunning = false;

volatile unsigned long rightRearPeriod;      ///< Total period of wheel0
unsigned long rightRearPrevTimer;            ///< Previous timer value of wheel0
volatile unsigned long rightRearUpdateTime;  ///< Last update time for wheel0
volatile unsigned long leftRearPeriod;      ///< Total period of wheel0
unsigned long leftRearTimer;            ///< Previous timer value of wheel0
volatile unsigned long leftRearUpdateTime;  ///< Last update time for wheel0
volatile unsigned long rightFrontPeriod;      ///< Total period of wheel0
unsigned long rightFrontPrevTimer;            ///< Previous timer value of wheel0
volatile unsigned long rightFrontUpdateTime;  ///< Last update time for wheel0
volatile unsigned long leftFrontPeriod;      ///< Total period of wheel0
unsigned long leftFrontPrevtimer;            ///< Previous timer value of wheel0
volatile unsigned long leftFrontUpdatetime;  ///< Last update time for wheel0

int pulsesPerRevolution = 4;  ///< Number of magnets on each wheel

unsigned long loopTimer = 0;
unsigned long loopDuration = 0;
unsigned long shortestLoopDuration = 10000;
unsigned long longestLoopDuration = 0;

uint16_t statusServoUpdates = 0;
uint16_t statusEncoderMsgs = 0;
uint16_t statusWheelSpeedMsgs = 0;
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
  Serial.begin(230400);

  // set servos
  steeringServo.attach(STEER_PIN);
  throttleServo.attach(THROTTLE_PIN);

  // set digital pins for encoders
  pinMode(LR_PIN, INPUT);
  pinMode(LF_PIN, INPUT);
  pinMode(RF_PIN, INPUT);
  pinMode(RR_PIN, INPUT);

  rightRearPeriod = 0;      ///< Total period of wheel0
  rightRearPrevTimer = 0;            ///< Previous timer value of wheel0
  rightRearUpdateTime = 0;  ///< Last update time for wheel0
  leftRearPeriod = 0;      ///< Total period of wheel0
  leftRearTimer = 0;            ///< Previous timer value of wheel0
  leftRearUpdateTime = 0;  ///< Last update time for wheel0
  rightFrontPeriod = 0;      ///< Total period of wheel0
  rightFrontPrevTimer = 0;            ///< Previous timer value of wheel0
  rightFrontUpdateTime = 0;  ///< Last update time for wheel0
  leftFrontPeriod = 0;      ///< Total period of wheel0
  leftFrontPrevtimer = 0;            ///< Previous timer value of wheel0
  leftFrontUpdatetime = 0;  ///< Last update time for wheel0
  
  attachInterrupt(digitalPinToInterrupt(LR_PIN), lrInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LF_PIN), lfInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(RF_PIN), rfInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(RR_PIN), rrInterrupt, FALLING);

  // start servo timer
  servoTimerStart = millis();
  servoTimerRunning = true;

  // start encoder timer
  encoderTimerStart = millis();
  encoderTimerRunning = true;

  // start wheelspeed timer
  wheelSpeedTimerStart = millis();
  wheelSpeedTimerRunning = true;

  // start status timer
  statusTimerStart = millis();
  statusTimerRunning = true;    
}

void loop() {
    loopTimer = millis();   
    
    readSerial();

    // if throttle locked (i.e. forward/reverse change requested, ensure that no movement for 
    if (lockThrottle)
    {
      if ((millis() - lastEncoderTick) > 500)
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

    if (wheelSpeedTimerRunning && ((millis() - wheelSpeedTimerStart) >= WHEEL_SPEED_TIMER))
    {
      wheelSpeedTimerStart = millis();
      sendWheelSpeedMessage();
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

void readIMU()
{
  float x, y, z, delta;
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);
  }

  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(x, y, z);
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


void sendWheelSpeedMessage()
{
  wheel_speed_msg ws_msg;

  float leftFront, rightFront, leftRear, rightRear;
  
  getWheelSpeeds(rightRear, leftRear, rightFront, leftFront);
  ws_msg.left_front = leftFront;
  ws_msg.right_front = rightFront;
  ws_msg.left_rear = leftRear;
  ws_msg.right_rear = rightRear;

  data_msg dmsg;
  packWheelSpeedMessage(ws_msg, dmsg);    

  uint8_t serial_data[MSG_SIZE+2];
  memcpy(serial_data, &dmsg, MSG_SIZE);
  serial_data[MSG_SIZE] = MESSAGE_DELIM;
  serial_data[MSG_SIZE+1] = MESSAGE_DELIM;
  Serial.write(serial_data, MSG_SIZE_WITH_DELIM);

  statusWheelSpeedMsgs++;
}

void sendEncoderMessage()
{
  encoder_msg enc_msg;
  enc_msg.left_rear = lrEncoderTicks;
  enc_msg.left_front = lfEncoderTicks;
  enc_msg.right_front = rfEncoderTicks;
  enc_msg.right_rear = rrEncoderTicks;

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

void getWheelSpeeds(float &rightRear, float &leftRear, float &rightFront, float &leftFront)
{
  unsigned long rightRearTemp, leftRearTemp, rightFrontTemp, leftFrontTemp;
  unsigned long rightRearT, leftRearT, rightFrontT, leftFrontT;

  noInterrupts();
  rightRearT = rightRearUpdateTime;
  rightRearTemp = rightRearPeriod;
  leftRearT = leftRearUpdateTime;
  leftRearTemp = leftRearPeriod;
  rightFrontT = rightFrontUpdateTime;
  rightFrontTemp = rightFrontPeriod;
  leftFrontT = leftFrontUpdatetime;
  leftFrontTemp = leftFrontPeriod;
  interrupts();

  unsigned long t = millis();

  if ((rightRearT + MAX_MEASUREMENT_AGE) < t) {
    rightRear = 0;
  } else {
    rightRear = 1000000.0 / (pulsesPerRevolution * rightRearTemp);
  }

  if ((leftRearT + MAX_MEASUREMENT_AGE) < t) {
    leftRear = 0;
  } else {
    leftRear = 1000000.0 / (pulsesPerRevolution * leftRearTemp);
  }

  if ((rightFrontT + MAX_MEASUREMENT_AGE) < t) {
    rightFront = 0;
  } else {
    rightFront = 1000000.0 / (pulsesPerRevolution * rightFrontTemp);
  }

  if ((leftFrontT + MAX_MEASUREMENT_AGE) < t) {
    leftFront = 0;
  } else {
    leftFront = 1000000.0 / (pulsesPerRevolution * leftFrontTemp);
  }

}

void lrInterrupt()
{
  lrEncoderTicks += direction;  
  lastEncoderTick = millis();

  unsigned long t = micros();
  leftRearPeriod = t - leftRearTimer;
  leftRearTimer = t;
  leftRearUpdateTime = millis();
}

void lfInterrupt()
{
  lfEncoderTicks += direction;
  lastEncoderTick = millis();

  unsigned long t = micros();
  leftFrontPeriod = t - leftFrontPrevtimer;
  leftFrontPrevtimer = t;
  leftFrontUpdatetime = millis();
}

void rfInterrupt()
{
  rfEncoderTicks += direction;
  lastEncoderTick = millis();

  unsigned long t = micros();
  rightFrontPeriod = t - rightFrontPrevTimer;
  rightFrontPrevTimer = t;
  rightFrontUpdateTime = millis();
}

void rrInterrupt()
{
  rrEncoderTicks += direction;
  lastEncoderTick = millis();

  unsigned long t = micros();
  rightRearPeriod = t - rightRearPrevTimer;
  rightRearPrevTimer = t;
  rightRearUpdateTime = millis();
}

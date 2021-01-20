#include <Servo.h>
#include "/home/lbarnett/ros2_ws/src/rcraicer/include/rcraicer/serial_data_msg.h"

Servo steeringServo;
Servo throttleServo;

int STEER_PIN = 10;
int THROTTLE_PIN = 11;

int steerPos = 1500;
int newSteerPos = 1500;
int throttlePos = 0;
char serialBuffer[100];
int serialBufferPtr = 0;

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  steeringServo.attach(STEER_PIN);
//  steeringServo.attach(THROTTLE_PIN);
}

void loop() {
//  readSerial();

    encoder_msg enc_msg;
    enc_msg.left_rear = 652324;
    enc_msg.left_front = 20;
    enc_msg.right_front = 30;
    enc_msg.right_rear = 40;

    data_msg dmsg;
    packEncoderMessage(enc_msg, dmsg);    
    

//    for (int i=0; i < (sizeof(data_msg)); i++)
//    {
//      Serial.print(i);
//      Serial.write(": ");
//      Serial.print(serialData[i]);
//      Serial.println();
//    }
//    Serial.println();
    uint8_t serial_data[MSG_SIZE+2];
    memcpy(serial_data, &dmsg, MSG_SIZE);
    serial_data[MSG_SIZE] = MESSAGE_DELIM;
    serial_data[MSG_SIZE+1] = MESSAGE_DELIM;
    Serial.write(serial_data, MSG_SIZE+2);    
    
    delay(5000);
    
  
//  // put your main code here, to run repeatedly:
//  for (steerPos = 1300; steerPos <= 1700 ; steerPos += 25)
//  {
//    Serial.write("Steerpos: ");
//    Serial.print(steerPos);
//    Serial.println();
//    steeringServo.writeMicroseconds(steerPos);
//    delay(2000);
//  }
}

void readSerial()
{
  while (Serial.available() > 0)
  {
    char data = Serial.read();

    if (data !='\n')
    {
      serialBuffer[serialBufferPtr] = data;
      serialBufferPtr++;  
    }
    else
    {
      processSerialData();
      serialBufferPtr = 0;
    }
  }
}

void processSerialMsg()
{
  data_msg msg;

  switch (msg.msg_type)
  {
    case SERVO_MSG:
      servo_msg sm;
      processServoMsg(sm);
      break;  
    case COMMAND_MSG:
      command_msg cm;
      processCommandMsg(cm);
    default:
      Serial.println("unknown msg");
      break;
     
  }  
}

void processServoMsg(servo_msg& msg)
{
  
}

void processCommandMsg(command_msg& msg)
{
  
}

void processSerialData()
{  
  char data[serialBufferPtr];

  for (int i=0; i < serialBufferPtr; i++)
  {
    data[i] = serialBuffer[i];
  }
  
  int val = atoi(data);

  Serial.write("Input: ");
  Serial.print(val);
  Serial.println();  
  sendServoControl(0, val);
}

void sendServoControl(int servo, int value)
{
  steeringServo.writeMicroseconds(value);
}

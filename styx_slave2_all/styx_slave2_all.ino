#define BOARD_ID 2
#define DELAY 10 // main loop delay
int motor_speed = 20; // motor speed
int started = 0;

int SERVO_PORT[8] = {4, 5, 6, 7, 8, 9, 10, 11};
byte motor_target[8] = {200, 200, 200, 200, 200, 200, 200, 200};
byte current[8] = {90, 90, 90, 90, 90, 90, 90, 90};

#include <Wire.h>
#include <VarSpeedServo.h> // https://github.com/netlabtoolkit/VarSpeedServo
VarSpeedServo myservo[8];

void printStatus()
{
  Serial.print(motor_speed);
  Serial.print('\t');
  for (int i = 0; i < 8; i++) {
    Serial.print(motor_target[i]);
    Serial.print('\t');
  }
  Serial.println();
}

void driveMotor()
{
  for (int i = 0; i < 8; i++) {
    if (motor_target[i] >= 200) {
      if (myservo[i].attached()) {
        myservo[i].detach();
      }
    } else {
      if (myservo[i].read() == motor_target[i] && myservo[i].attached()) {
        //        myservo[i].detach();
      } else {
        if (!myservo[i].attached()) {
          myservo[i].attach(SERVO_PORT[i]);
        }
        myservo[i].write(motor_target[i], motor_speed, false);
      }
    }
  }
}

void receiveEvent(int howMany)
{
  byte temp;
  static int protocol_pos = 0;
  while (0 < Wire.available()) {
    temp = Wire.read();
    if (temp == 0xfe) {
      protocol_pos = 0; // set starting point
      started = 1;
    } else if (started == 1) {
      if (protocol_pos == 0) {
        motor_speed = temp; // target motor speed
      } else if (protocol_pos > 8) {
        started = 0;
      } else {
        motor_target[protocol_pos - 1] = temp; // target motor position
      }
      protocol_pos = protocol_pos + 1;
    }
  }
  digitalWrite(13, !digitalRead(13)); // toggle LED
}

void requestEvent()
{
}

void setup() {
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.begin(BOARD_ID);
  pinMode(13, OUTPUT); //LED
}

void loop() {
  driveMotor(); // 모터 제어
  printStatus(); // 상태 프린트
  delay(DELAY);
}




#define MODE 0
// 0:정상동작, 2:센서별세팅 3: 눈꺼풀 제어없이 동작 4: 센서만 테스트

int THRESHOLD = 100; // 초음파 센서 인식 거리
#define FIRED 50; // 센서인식되었을 때 유지되는 시간 (루프횟수)
byte motor_speed = 10; // 모터 동작 속도


#include <NewPing.h>
// http://playground.arduino.cc/Code/NewPing

#include <Wire.h>
#define NUM_SLAVES 5
#define DELAY 1 // main loop delay

#define SONAR_NUM     9 // 초음파 센서 개수
#define MAX_DISTANCE 500 // 최대 거리 (대기 시간)
int TEST_sonar = 1; // 1..9

unsigned int cm[SONAR_NUM] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE}; // 초음파 센서 결과 값 (거리)
unsigned long detected_time;
int fired[SONAR_NUM] = {0,};
int sleeped = 0;

// 보드1 모터들: 센서1..9 까지의 각 모터 위치
#define U_LID_L_UP1 160
#define U_LID_L_DOWN1 180
#define U_LID_R_UP1 120
#define U_LID_R_DOWN1 130
#define U_EYE_L1 110
#define U_EYE_R1 90

#define L_LID_L_UP1 150
#define L_LID_L_DOWN1 80
#define L_LID_R_UP1 60
#define L_LID_R_DOWN1 10
#define L_EYE_L1 100
#define L_EYE_R1 110

byte motor_target_1[SONAR_NUM][8] = { // L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
  L_EYE_L1 + 25, L_EYE_R1 + 25, 250, 250, U_EYE_L1 + 25, U_EYE_R1 + 25, U_LID_L_UP1, U_LID_R_UP1,
  L_EYE_L1 , L_EYE_R1 , 250, 250, U_EYE_L1, U_EYE_R1, U_LID_L_UP1, U_LID_R_UP1, //
  L_EYE_L1 - 10 , L_EYE_R1 - 10 , 250, 250, U_EYE_L1 - 10, U_EYE_R1 - 10, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 20 , L_EYE_R1 - 20 , 250, 250, U_EYE_L1 - 20, U_EYE_R1 - 20, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 25, L_EYE_R1 - 25, 250, 250, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 25, L_EYE_R1 - 25, 250, 250, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 25, L_EYE_R1 - 25, 250, 250, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 25, L_EYE_R1 - 25, 250, 250, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
  L_EYE_L1 - 25, L_EYE_R1 - 25, 250, 250, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1

//  L_EYE_L1 + 25, L_EYE_R1 + 25, L_LID_L_UP1, L_LID_R_UP1, U_EYE_L1 + 25, U_EYE_R1 + 25, U_LID_L_UP1, U_LID_R_UP1,
//  L_EYE_L1 , L_EYE_R1 , L_LID_L_UP1, L_LID_R_UP1, U_EYE_L1, U_EYE_R1, U_LID_L_UP1, U_LID_R_UP1, //
//  L_EYE_L1 - 10 , L_EYE_R1 - 10 , L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 10, U_EYE_R1 - 10, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 20 , L_EYE_R1 - 20 , L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 20, U_EYE_R1 - 20, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 25, L_EYE_R1 - 25, L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 25, L_EYE_R1 - 25, L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 25, L_EYE_R1 - 25, L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 25, L_EYE_R1 - 25, L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1,
//  L_EYE_L1 - 25, L_EYE_R1 - 25, L_LID_L_DOWN1, L_LID_R_DOWN1, U_EYE_L1 - 25, U_EYE_R1 - 25, U_LID_L_DOWN1, U_LID_R_DOWN1
};

// 보드2 모터들: 센서1..9 까지의 각 모터 위치
#define U_LID_L_UP2 110
#define U_LID_L_DOWN2 70
#define U_LID_R_UP2 70
#define U_LID_R_DOWN2 120
#define U_EYE_L2 100
#define U_EYE_R2 145

#define L_LID_L_UP2 90
#define L_LID_L_DOWN2 140
#define L_LID_R_UP2 30
#define L_LID_R_DOWN2 80
#define L_EYE_L2 80
#define L_EYE_R2 105

byte motor_target_2[SONAR_NUM][8] = { // L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
  L_EYE_L2 + 25, L_EYE_R2 + 25, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 + 25, U_EYE_R2 + 25, U_LID_L_DOWN2, U_LID_R_DOWN2,
  L_EYE_L2 + 25, L_EYE_R2 + 25, L_LID_L_UP2, L_LID_R_UP2, U_EYE_L2 + 25, U_EYE_R2 + 25, U_LID_L_UP2, U_LID_R_UP2,
  L_EYE_L2 , L_EYE_R2 +5 , L_LID_L_UP2, L_LID_R_UP2, U_EYE_L2, U_EYE_R2 + 10, U_LID_L_UP2, U_LID_R_UP2, //
  L_EYE_L2 - 10 , L_EYE_R2 - 10 , L_LID_L_UP2, L_LID_R_UP2, U_EYE_L2 - 10 , U_EYE_R2 , U_LID_L_UP2, U_LID_R_UP2,
  L_EYE_L2 - 20 , L_EYE_R2 - 20, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 - 20, U_EYE_R2 - 10, U_LID_L_DOWN2, U_LID_R_DOWN2,
  L_EYE_L2 - 20 , L_EYE_R2 - 20, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 - 20, U_EYE_R2 - 10, U_LID_L_DOWN2, U_LID_R_DOWN2,
  L_EYE_L2 - 20 , L_EYE_R2 - 20, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 - 20, U_EYE_R2 - 10, U_LID_L_DOWN2, U_LID_R_DOWN2,
  L_EYE_L2 - 20 , L_EYE_R2 - 20, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 - 20, U_EYE_R2 - 10, U_LID_L_DOWN2, U_LID_R_DOWN2,
  L_EYE_L2 - 20 , L_EYE_R2 - 20, L_LID_L_DOWN2, L_LID_R_DOWN2, U_EYE_L2 - 20, U_EYE_R2 - 10, U_LID_L_DOWN2, U_LID_R_DOWN2
};


// 보드3 모터들: 센서1..9 까지의 각 모터 위치
#define U_LID_L_UP3 110
#define U_LID_L_DOWN3 80
#define U_LID_R_UP3 25
#define U_LID_R_DOWN3 75
#define U_EYE_L3 55
#define U_EYE_R3 100

#define L_LID_L_UP3 80
#define L_LID_L_DOWN3 40
#define L_LID_R_UP3 70
#define L_LID_R_DOWN3 130
#define L_EYE_L3 85
#define L_EYE_R3 105

byte motor_target_3[SONAR_NUM][8] = { // L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
  L_EYE_L3 + 25, L_EYE_R3 + 25, L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 + 25, U_EYE_R3 + 25, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 + 25, L_EYE_R3 + 25, L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 + 25, U_EYE_R3 + 25, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 + 25, L_EYE_R3 + 25, L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 + 25, U_EYE_R3 + 25, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 + 15, L_EYE_R3 + 25, L_LID_L_UP3, L_LID_R_UP3, U_EYE_L3 + 15, U_EYE_R3 + 15, U_LID_L_UP3, U_LID_R_UP3,
  L_EYE_L3 , L_EYE_R3 , L_LID_L_UP3, L_LID_R_UP3, U_EYE_L3, U_EYE_R3, U_LID_L_UP3, U_LID_R_UP3,
  L_EYE_L3 - 20 , L_EYE_R3 - 20 , L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 - 10 , U_EYE_R3 - 10, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 - 20 , L_EYE_R3 - 20 , L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 - 20 , U_EYE_R3 - 20, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 - 20 , L_EYE_R3 - 20 , L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 - 20 , U_EYE_R3 - 20, U_LID_L_DOWN3, U_LID_R_DOWN3,
  L_EYE_L3 - 20 , L_EYE_R3 - 20 , L_LID_L_DOWN3, L_LID_R_DOWN3, U_EYE_L3 - 20 , U_EYE_R3 - 20, U_LID_L_DOWN3, U_LID_R_DOWN3
};

// 보드4 모터들: 센서1..9 까지의 각 모터 위치
#define U_LID_L_UP4 70
#define U_LID_L_DOWN4 100
#define U_LID_R_UP4 115
#define U_LID_R_DOWN4 140
#define U_EYE_L4 70
#define U_EYE_R4 150

#define L_LID_L_UP4 70
#define L_LID_L_DOWN4 110
#define L_LID_R_UP4 120
#define L_LID_R_DOWN4 80
#define L_EYE_L4 65
#define L_EYE_R4 125

byte motor_target_4[SONAR_NUM][8] = { // L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 + 15 , L_EYE_R4 + 25 , L_LID_L_UP4, L_LID_R_UP4, U_EYE_L4 + 15, U_EYE_R4 + 5, U_LID_L_UP4, U_LID_R_UP4,
  L_EYE_L4 , L_EYE_R4 , L_LID_L_UP4, L_LID_R_UP4, U_EYE_L4, U_EYE_R4, U_LID_L_UP4, U_LID_R_UP4,
  L_EYE_L4 - 25, L_EYE_R4 - 25, L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 - 10, U_EYE_R4 - 15, U_LID_L_DOWN4, U_LID_R_DOWN4,
  L_EYE_L4 - 25, L_EYE_R4 - 25 , L_LID_L_DOWN4, L_LID_R_DOWN4, U_EYE_L4 - 20, U_EYE_R4 - 25, U_LID_L_DOWN4, U_LID_R_DOWN4
};


// 보드5 모터들: 센서1..9 까지의 각 모터 위치
#define U_LID_L_UP5 80
#define U_LID_L_DOWN5 120
#define U_LID_R_UP5 100
#define U_LID_R_DOWN5 60
#define U_EYE_L5 40
#define U_EYE_R5 110

#define L_LID_L_UP5 80
#define L_LID_L_DOWN5 30
#define L_LID_R_UP5 40
#define L_LID_R_DOWN5 90
#define L_EYE_L5 80
#define L_EYE_R5 130

byte motor_target_5[SONAR_NUM][8] = { // L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5 , L_EYE_R5 , L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5 + 25, U_EYE_R5 + 25, U_LID_L_DOWN5, U_LID_R_DOWN5,
  L_EYE_L5, L_EYE_R5, L_LID_L_DOWN5, L_LID_R_DOWN5, U_EYE_L5, U_EYE_R5, U_LID_L_UP5, U_LID_R_UP5,
  L_EYE_L5 - 25, L_EYE_R5 - 25, L_LID_L_UP5, L_LID_R_UP5, U_EYE_L5 + 25, U_EYE_R5 + 25, U_LID_L_DOWN5, U_LID_R_DOWN5
};


// 초기상태:
// L_EYE_L, L_EYE_R, L_LID_L, L_LID_R, U_EYE_L, U_EYE_R, U_LID_L, U_LID_R
byte motor_target[NUM_SLAVES][8] = {

  L_EYE_L1, L_EYE_R1, L_LID_L_UP1, L_LID_R_UP1, U_EYE_L1, U_EYE_R1, U_LID_L_UP1, U_LID_R_UP1,
  L_EYE_L2, L_EYE_R2, L_LID_L_UP2, L_LID_R_UP2, U_EYE_L2, U_EYE_R2, U_LID_L_UP2, U_LID_R_UP2,
  L_EYE_L3, L_EYE_R3, L_LID_L_UP3, L_LID_R_UP3, U_EYE_L3, U_EYE_R3, U_LID_L_UP3, U_LID_R_UP3,
  L_EYE_L4, L_EYE_R4, L_LID_L_UP4, L_LID_R_UP4, U_EYE_L4, U_EYE_R4, U_LID_L_UP4, U_LID_R_UP4,
  L_EYE_L5, L_EYE_R5, L_LID_L_UP5, L_LID_R_UP5, U_EYE_L5, U_EYE_R5, U_LID_L_UP5, U_LID_R_UP5,

  /*
    L_EYE_L1, L_EYE_R1, (L_LID_L_DOWN1 + L_LID_L_UP1) / 2, (L_LID_R_DOWN1 + L_LID_R_UP1) / 2, U_EYE_L1, U_EYE_R1, (U_LID_L_DOWN1 + U_LID_L_UP1) / 2, (U_LID_R_DOWN1 + U_LID_R_UP1) / 2,
    L_EYE_L2, L_EYE_R2, (L_LID_L_DOWN2 + L_LID_L_UP2) / 2, (L_LID_R_DOWN2 + L_LID_R_UP2) / 2, U_EYE_L2, U_EYE_R2, (U_LID_L_DOWN2 + U_LID_L_UP2) / 2, (U_LID_R_DOWN2 + U_LID_R_UP2) / 2,
    L_EYE_L3, L_EYE_R3, (L_LID_L_DOWN3 + L_LID_L_UP3) / 2, (L_LID_R_DOWN3 + L_LID_R_UP3) / 2, U_EYE_L3, U_EYE_R3, (U_LID_L_DOWN3 + U_LID_L_UP3) / 2, (U_LID_R_DOWN3 + U_LID_R_UP3) / 2,
    L_EYE_L4, L_EYE_R4, (L_LID_L_DOWN4 + L_LID_L_UP4) / 2, (L_LID_R_DOWN4 + L_LID_R_UP4) / 2, U_EYE_L4, U_EYE_R4, (U_LID_L_DOWN4 + U_LID_L_UP4) / 2, (U_LID_R_DOWN4 + U_LID_R_UP4) / 2,
    L_EYE_L5, L_EYE_R5, (L_LID_L_DOWN5 + L_LID_L_UP5) / 2, (L_LID_R_DOWN5 + L_LID_R_UP5) / 2, U_EYE_L5, U_EYE_R5, (U_LID_L_DOWN5 + U_LID_L_UP5) / 2, (U_LID_R_DOWN5 + U_LID_R_UP5) / 2,
  */

  /*  250, 250, 250, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 250, 250, 250, */
};

byte motor_target_sleep[NUM_SLAVES][8] = {

  L_EYE_L1, L_EYE_R1, 250, 250, U_EYE_L1, U_EYE_R1, 250, 250,
  L_EYE_L2, L_EYE_R2, 250, 250, U_EYE_L2, U_EYE_R2, 250, 250,
  L_EYE_L3, L_EYE_R3, 250, 250, U_EYE_L3, U_EYE_R3, 250, 250,
  L_EYE_L4, L_EYE_R4, 250, 250, U_EYE_L4, U_EYE_R4, 250, 250,
  L_EYE_L5, L_EYE_R5, 250, 250, U_EYE_L5, U_EYE_R5, 250, 250,
};

int SENSOR_PORT[SONAR_NUM] = {11, 10, 9, 8, 7, 6, 5, 4, 3};

NewPing sonar[SONAR_NUM] = {
  NewPing(SENSOR_PORT[0], SENSOR_PORT[0], MAX_DISTANCE),
  NewPing(SENSOR_PORT[1], SENSOR_PORT[1], MAX_DISTANCE),
  NewPing(SENSOR_PORT[2], SENSOR_PORT[2], MAX_DISTANCE),
  NewPing(SENSOR_PORT[3], SENSOR_PORT[3], MAX_DISTANCE),
  NewPing(SENSOR_PORT[4], SENSOR_PORT[4], MAX_DISTANCE),
  NewPing(SENSOR_PORT[5], SENSOR_PORT[5], MAX_DISTANCE),
  NewPing(SENSOR_PORT[6], SENSOR_PORT[6], MAX_DISTANCE),
  NewPing(SENSOR_PORT[7], SENSOR_PORT[7], MAX_DISTANCE),
  NewPing(SENSOR_PORT[8], SENSOR_PORT[8], MAX_DISTANCE)
};

unsigned int sonar_temp;



void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT); //LED
  Wire.begin();
  detected_time = millis();

}

void printSensor()
{
  Serial.print("Sonars:");
  for (int i = 0; i < SONAR_NUM; i++) {
    Serial.print('\t');
    Serial.print(cm[i]);
  }
  Serial.println();
}

void printStatus()
{
  for (int i = 0; i < SONAR_NUM; i++) {
    Serial.print('\t');
    Serial.print(fired[i]);
  }
  Serial.println();
}

int priority_order1[SONAR_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
int priority_order2[SONAR_NUM] = {3, 2, 4, 1, 5, 0, 6, 7, 8};
int priority_order3[SONAR_NUM] = {4, 3, 5, 2, 6, 1, 7, 0, 8};
int priority_order4[SONAR_NUM] = {6, 7, 5, 4, 8, 3, 2, 1, 0};
int priority_order5[SONAR_NUM] = {8, 7, 6, 5, 4, 3, 2, 1, 0};


void loop()
{
  for (int i = 0; i < SONAR_NUM; i++) {

    if (MODE == 0 || MODE == 3 || MODE == 4) {

      // 초음파 센서 측정
      sonar_temp = sonar[i].ping_cm();
      if (sonar_temp == 0) sonar_temp = MAX_DISTANCE; // out of range 변환
      cm[i] = sonar_temp;

      // 센서 중 기준값 이하 찾기
      for (int fired_index = 0; fired_index < SONAR_NUM; fired_index++) {
        if (cm[i] < THRESHOLD) {
          fired[i] = FIRED;
          detected_time = millis();
        } else {
          if (fired[i] > 0)
            fired[i]--;
        }
      }

      // 센서 인식된 보드별로 목표 위치 설정하기
      for (int j = 0; j < SONAR_NUM; j++) {
        if (fired[priority_order1[j]] > 0) { //보드 1: 센서 우선 순위 1,0,2,3,4,5,6,7,8
          for (int m = 0; m < 8; m++)
            motor_target[0][m] = motor_target_1[priority_order1[j]][m];
          break;
        }
      }
      for (int j = 0; j < SONAR_NUM; j++) {
        if (fired[priority_order2[j]] > 0) { //보드 2: 센서 우선 순위 3,2,4,1,5,0,6,7,8
          for (int m = 0; m < 8; m++)
            motor_target[1][m] = motor_target_2[priority_order2[j]][m];
          break;
        }
      }
      for (int j = 0; j < SONAR_NUM; j++) {
        if (fired[priority_order3[j]] > 0) { //보드 3: 센서 우선 순위 4,3,5,2,6,1,7,0,8
          for (int m = 0; m < 8; m++)
            motor_target[2][m] = motor_target_3[priority_order3[j]][m];
          break;
        }
      }
      for (int j = 0; j < SONAR_NUM; j++) {
        if (fired[priority_order4[j]] > 0) { //보드 4: 센서 우선 순위 6,5,7,4,8,3,2,1,0
          for (int m = 0; m < 8; m++)
            motor_target[3][m] = motor_target_4[priority_order4[j]][m];
          break;
        }
      }
      for (int j = 0; j < SONAR_NUM; j++) {
        if (fired[priority_order5[j]] > 0) { //보드 5: 센서 우선 순위 7,8,6,5,4,3,2,1,0
          for (int m = 0; m < 8; m++)
            motor_target[4][m] = motor_target_5[priority_order5[j]][m];
          break;
        }
      }
      printStatus(); // 인식된 센서 표시
    }

    if (MODE == 0 || MODE == 3) {
      // 센서가 일정시간 인식되지 않으면 모터 끄기
      if (millis() - detected_time > 10000) {
        if (sleeped == 0){
          for (int k = 0; k < NUM_SLAVES; k++) {
            for (int m = 0; m < 8; m++) motor_target[k][m] = motor_target_sleep[k][m];

            for (int m = 0; m < NUM_SLAVES; m++) {
              Wire.beginTransmission(m + 1);
              Wire.write(0xfe); // start packet
              Wire.write(motor_speed);
              for (int j = 0; j < 8; j++) {
                Wire.write(motor_target[m][j]);
              }
              Wire.endTransmission();
            }
            digitalWrite(13, !digitalRead(13)); // toggle LED (깜박이지 않으면 통신이 안되고 있는 것으로..)
          }
          sleeped = 1;
        }
      }else{
        sleeped = 0;
      }
    }

    if (MODE == 1 || MODE == 2) {
      // 테스트 센서 위치 입력 받고 모터 세팅하기

      Serial.print("\n Test Sonar Number _0:stop _1..9:sensor pos ?");
      while (!(Serial.available()));
      TEST_sonar = Serial.parseInt();
      Serial.println(TEST_sonar);
      for (int k = 0; k < NUM_SLAVES; k++) {
        for (int m = 0; m < 8; m++) {
          if (TEST_sonar == 0) {
            motor_target[k][m] = 250;
          } else {
            switch (k) {
              case 0: motor_target[k][m] = motor_target_1[TEST_sonar - 1][m]; break;
              case 1: motor_target[k][m] = motor_target_2[TEST_sonar - 1][m]; break;
              case 2: motor_target[k][m] = motor_target_3[TEST_sonar - 1][m]; break;
              case 3: motor_target[k][m] = motor_target_4[TEST_sonar - 1][m]; break;
              case 4: motor_target[k][m] = motor_target_5[TEST_sonar - 1][m]; break;
            }
          }
          Serial.print('\t');
          Serial.print(motor_target[k][m]);
        }
        Serial.println();
      }
    }

    if (MODE == 3) {
      // 눈꺼풀 끄기
      for (int k = 0; k < NUM_SLAVES; k++) {
        motor_target[k][2] = 250;
        motor_target[k][3] = 250;
        motor_target[k][6] = 250;
        motor_target[k][7] = 250;
      }
    }
    
    if ((MODE == 0 || MODE == 1 || MODE == 2 || MODE == 3) && sleeped == 0) {
      // 각 보드로 전송하기
      for (int k = 0; k < NUM_SLAVES; k++) {
        Wire.beginTransmission(k + 1);
        Wire.write(0xfe); // start packet
        Wire.write(motor_speed);
        for (int j = 0; j < 8; j++) {
          Wire.write(motor_target[k][j]);
        }
        Wire.endTransmission();
      }
      digitalWrite(13, !digitalRead(13)); // toggle LED (깜박이지 않으면 통신이 안되고 있는 것으로..)
    }

    delay(DELAY);
  }
}




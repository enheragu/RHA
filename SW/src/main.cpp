/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 08_Sep_2017
 */



//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
//#include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);


void setup() {
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  // g15.begin(19200);
  //servo_test1.init(1, 2, 3, 8, 9200);
  ServoRHA servo_broadcast;
  servo_broadcast.init(0xFE,2,3,8,19200);         //Broadcast initialize
  servo_broadcast.ping(data);
  IDcurrent = data[0];
  Serial.print("ID now is: "); Serial.println(IDcurrent);
  ServoRHA servo1;
  servo1.init(1,2,3,8,19200);

  delay(25);
  servo1.setAlarmLED(0x7F);
  delay(25);
  Serial.println("Exiting wheel mode");
  word error = servo1.exitWheelMode();
  printServoStatusError(error);
  delay(25);
  Serial.println("Set wheel mode");
  error = servo1.setWheelMode();
  printServoStatusError(error);
  delay(25);
  Serial.println("Set speed");
  error = servo1.setWheelSpeed(1000, CW);
  printServoStatusError(error);
}

void loop(){

}

/*
void loop() {
  Serial.println("estoy en el loop");



}*/

//#endif

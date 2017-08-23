#ifndef UNIT_TEST // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
#include "joint_rha.h"

#define DEBUG_SERVO_RHA
//#define DEBUG_TEST_SERVO_RHA
//#define DEBUG_CYTRON_G15SHIELD
//#define DEBUG_TEST_CYTRON_G15SHIELD

Cytron_G15Shield g15(1, 2, 3, 8);

void setup(){
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  g15.begin(19200);
}
int flag = 0;

void loop() {
  Serial.println("estoy en el loop");
  g15.setWheelMode();
  g15.setWheelSpeed(500,CW,iWRITE_DATA);
  delay(1000);
  /*Serial.println("estoy en el loop");
  ServoRHA servo_test0(6, 2, 3, 8);
    Serial.println("servo construido");
  servo_test0.setWheelMode();
    Serial.println("modo rueda activado");
  if (flag=0) {
    //servo_test0.setGoalEncoder(8.56, CCW);
    flag=1;
  }
  //servo_test0.doNext();*/
  delay(20);
}

#endif

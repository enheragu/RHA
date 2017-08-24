//#ifndef UNIT_TEST // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
#include "joint_rha.h"

#define DEBUG_SERVO_RHA
//#define DEBUG_TEST_SERVO_RHA
//#define DEBUG_CYTRON_G15SHIELD
//#define DEBUG_TEST_CYTRON_G15SHIELD

//Cytron_G15Shield g15(1, 2, 3, 8);

int flag = 0;
ServoRHA servo_test1(1, 2, 3, 8);

void setup(){
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  //g15.begin(19200);
  servo_test1.initServo();
}

void loop() {
  Serial.println("estoy en el loop");
  //g15.setWheelMode();
  //g15.setWheelSpeed(500,CW,iWRITE_DATA);
  delay(1000);
  Serial.println("estoy en el loop");
  if (flag==0) {
    servo_test1.setWheelMode();
    Serial.println("modo rueda activado");
    servo_test1.setGoalEncoder(8.56, CCW);
    Serial.println("Goal enviado");
    flag=1;
  }
  servo_test1.doNext();
  delay(20);
}

//#endif

//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
#include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

int flag = 1;
ServoRHA servo_test1(1, 2, 3, 8);

void setup() {
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  // g15.begin(19200);
  //servo_test1.init(1, 2, 3, 8, 9200);
  servo_test1.init();
  servo_test1.exitWheelMode();
}


void loop() {
    Serial.println("estoy en el loop");
    byte angle = 0;
 while (1){

     servo_test1.setSpeed( 100, iWRITE_DATA); // Set G15 (ID = 1) speed to 500,
     servo_test1.rotateCW( angle, iWRITE_DATA); // Set G15 (ID = 1) position to 0 deg
     delay(100);
     Serial.print("Speed set to 500, real speed is : "); Serial.println(servo_test1.speedRead());
     delay(2000);
     angle -= 10;
 }
  // g15.setWheelMode();
  // g15.setWheelSpeed(500, CW, iWRITE_DATA);



}

//#endif

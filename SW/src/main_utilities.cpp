/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   23-Dec-2017
 * @Project: RHA
 * @Last modified by:   enheragu
 * @Last modified time: 23-Dec-2017
 */


// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include <Arduino.h>
#include "rha_types.h"
#include "utilities.h"
#include "servo_rha.h"
#include "joint_handler.h"
#include "robot_rha.h"
// #include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

JHUtilitiesJH joint_handler;

void setup() {
  delay(2000);
  Serial.begin(921600); //115200 230400 250000 460800 921600
  Serial.println("# Start setup");

  joint_handler.setTimer(50);
  Serial.println("# Init G15 serial port");
  joint_handler.initSerial(17,16,8,460800);  // baudrate 460800 means 57.6 bytes/milisecond
  Serial.println("# Init Joints");
  joint_handler.initJoints(0);
  //Serial.print("# Id for servo is: "); Serial.println(joint_handler.joint_[0].servo_.getID());
  joint_handler.updateJointInfo();
  delay(5000);
  Serial.println("# Setup done");


  Serial.println("# Init loop");
}

unsigned int i = 0, c = 0;
long timeNow = 0;
long timeBuffer[800];
float average = 0;
float standard_deviation = 0;

void loop() {
    //joint_handler.extractRegulatorData(0);
    //joint_handler.extractStepInputData(0);
    joint_handler.extractSlopeInputData(0);
    //joint_handler.checkSpeed(0);
    while (true) {
        delay(100);
    }
    //delay(500);

    /*for (i = 0; i < 800; i++) {
        timeNow = millis();
        robo_health_arm.handleWithChuck();
        timeBuffer[i] = millis() - timeNow;
    }

    MeasureUtilities::averageChauvenet(timeBuffer, 800, average, standard_deviation);
    Serial.print("Average time of 800 mesaures is : "); Serial.print(average);Serial.println("ms");

    for (i = 0; i < 800; i++) {
        timeNow = millis();
        robo_health_arm.joint_handler_.controlLoop();
        timeBuffer[i] = millis() - timeNow;
    }

    MeasureUtilities::averageChauvenet(timeBuffer, 800, average, standard_deviation);
    Serial.print("Average time of 800 mesaures is : "); Serial.print(average); Serial.println("ms");

    while (true)
    {
         delay(100);
    }*/
    //if (i % 1000 == 0){Serial.println(" "); Serial.print("############  "); Serial.print (c); Serial.print("  ############"); Serial.println(" "); c++;}
    //i++;
    //joint_handler.controlLoop();
    //joint_handler.checkComSucces(150);
    //joint_handler.extractSlopeInputData(0);
    //Serial.println(" ");
    //Serial.println("###############################################################");
    //Serial.println("######################### End of test #########################");
    //Serial.println("###############################################################");
    //while (true) {
    //    delay(150);
    //}
}

// #endif

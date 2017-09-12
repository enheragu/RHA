/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 12-Sep-2017
 */



// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
#include "utilities.h"
//#include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);


void setup() {
  Serial.begin(9600);
  Serial.println("Setup done");
}

void loop(){
    Serial.println("Begin of loop");
    //MeasureUtilities::checkSpeed();
    //ServoUtilities::fullFactoryResetBR();
    while(true){
        MeasureUtilities::checkTimeGetInfo(CHAUVENET_REPETITIONS);
        MeasureUtilities::checkTimeSpeedRead(CHAUVENET_REPETITIONS);
    }
    //MeasureUtilities::extractRegulatorData();
    /*while(true){
        DebugSerialSeparation(1);
        delay(1000);
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.begin(19200);         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error & ~SERROR_IDMISMATCH);  // ID mismatch is deleted
        DebugSerialSeparation(1);
    }*/
}

// #endif

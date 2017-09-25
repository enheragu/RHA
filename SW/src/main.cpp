/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 24-Sep-2017
 */



// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include <Arduino.h>
#include "servo_rha.h"
#include "joint_handler.h"
#include "rha_types.h"
#include "utilities.h"
// #include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

JHUtilitiesJH joint_handler;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup begin");

  joint_handler.setTimer(50);
  joint_handler.initSerial(2,3,8);
  joint_handler.initJoints(0);
  Serial.print("Id for servo is: "); Serial.println(joint_handler.joint_[0].servo_.getID());

  joint_handler.updateJointInfo();
  delay(5000);
  Serial.println("Setup done");
}

void loop() {
    //joint_handler.extractRegulatorData(0);
    //while (true) {

    //}
}
/**
void loop(){
    RHATypes::SpeedGoal speed_goal(1,50,0, CW);  // Id, speed, speed_slope
    joint_handler.setSpeedGoal(speed_goal);
    joint_handler.joint_[0].servo_.speed_regulator_.setKRegulator(kp_samples[sample],0,0);
    Serial.print("n_data"); Serial.print(sample); Serial.print(" = "); Serial.println(SAMPLE_REGULATOR);
    Serial.print("speed_target"); Serial.print(sample); Serial.print("  = "); Serial.println(joint_handler.joint_[0].getSpeedTarget());
    Serial.print("regulatorTest"); Serial.print(sample); Serial.print("  = [");
    Serial.print("['"); Serial.print(joint_handler.joint_[0].servo_.speed_regulator_.getKp()); Serial.print("']");

    int counter = 0;
    while(true){
        unsigned long time_init = millis();
        joint_handler.controlLoop();
        Serial.print(",['"); Serial.print(joint_handler.joint_[0].servo_.getSpeed()); Serial.print("','");
        Serial.print(time_init); Serial.println("']\\");
        counter ++;
        if (counter > SAMPLE_REGULATOR) break;

    }

    Serial.println("]");
    sample++;
    if (sample >= SAMPLE_KP)
        while(true) {}

}*/

// #endif

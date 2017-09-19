/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 20-Sep-2017
 */



// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include "joint_handler.h"
#include <Arduino.h>
// #include "utilities.h"
// #include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

JointHandler joint_handler;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup begin");

  joint_handler.initSerial(2,3,8);
  joint_handler.initJoints();
  joint_handler.setTimer(50);


  joint_handler.joint_[0].servo_.setRegulatorKp(100/60);

  delay(5000);
  Serial.println("Setup done");
}

#define SAMPLE_REGULATOR 150
#define SAMPLE_KP 6
int sample = 0;
float kp_samples[SAMPLE_KP] = {1.66, 5, 10, 20, 50, 100};

void loop(){
    SpeedGoal speed_goal(1,50,0);  // Id, speed, speed_slope
    joint_handler.setSpeedGoal(speed_goal);
    joint_handler.joint_[0].servo_.setRegulatorKp(kp_samples[sample]);
    Serial.print("n_data"); Serial.print(sample); Serial.print(" = "); Serial.println(SAMPLE_REGULATOR);
    Serial.print("speed_target"); Serial.print(sample); Serial.print("  = "); Serial.println(joint_handler.joint_[0].getSpeedTarget());
    Serial.print("regulatorTest"); Serial.print(sample); Serial.print("  = [");
    Serial.print("['"); Serial.print(joint_handler.joint_[0].servo_.getKp()); Serial.print("']");

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

}

// #endif

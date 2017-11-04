/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */



// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include <Arduino.h>
#include "servo_rha.h"
#include "joint_handler.h"
#include "rha_types.h"
#include "utilities.h"
#include "chuck_handler.h"
// #include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

//JHUtilitiesJH joint_handler;
JointHandler joint_handler;
ChuckHandler chuck_handler;

void setup() {
  delay(2000);
  Serial.begin(921600); //115200 230400 250000 460800 921600
  Serial.println("# Setup begin");

  joint_handler.setTimer(50);
  Serial.println("# Init G15 serial port");
  joint_handler.initSerial(19,18,8,460800);  // baudrate 460800 means 57.6 bytes/milisecond
  Serial.println("# Init Joints");
  joint_handler.initJoints();
  Serial.println("# Exit wheel mode");
  joint_handler.sendExitWheelModeAll();
  delay(100);
  Serial.println("# Set wheel mode");
  joint_handler.sendSetWheelModeAll();
  //Serial.print("# Id for servo is: "); Serial.println(joint_handler.joint_[0].servo_.getID());
  joint_handler.updateJointInfo();
  delay(5000);
  Serial.println("# Setup done");

  RHATypes::SpeedGoal speed_goal(1,150,0,CW);  // Id, speed, speed_slope
  joint_handler.setSpeedGoal(speed_goal);
  RHATypes::SpeedGoal speed_goal2(2,150,0,CW);  // Id, speed, speed_slope
  joint_handler.setSpeedGoal(speed_goal2);

  /*Serial.println("# Set wheel speed in single mode");
  joint_handler.sendSetWheelSpeedAll(500,CW);
  delay(5000);*/
  Serial.println("# Init loop");

  chuck_handler.begin();
}

unsigned int i = 0, c = 0;

void loop() {
    //joint_handler.extractRegulatorData(0);
    //joint_handler.extractStepInputData(0);
    chuck_handler.readAxis();
    delay(500);
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

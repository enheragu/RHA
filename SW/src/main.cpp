/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */

#ifndef UNIT_TEST  // disable program main loop while unit testing in progress

#include <Arduino.h>
// #include "rha_types.h"
// #include "utilities.h"
// #include "servo_rha.h"
// #include "joint_handler.h"
#include "robot_rha.h"
// #include "joint_rha.h"
#include "MemoryFree.h"

RobotRHA robo_health_arm;

void setup() {
    delay(2000);
    Serial.begin(921600);  // 115200 230400 250000 460800 921600
    Serial.println(F("# Start setup"));

    robo_health_arm.initJointHandler();
    Serial.println(F("#Joint Handler initialiced"));
    //robo_health_arm.initChuckHandler();

    /*joint_handler.setTimer(50);
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
    */
    /*Serial.println("# Set wheel speed in single mode");
    joint_handler.sendSetWheelSpeedAll(500,CW);
    delay(5000);*/
    Serial.println("F(# Init loop)");
}

uint32_t i = 0, c = 0;
bool flag = false;

void loop() {
    // delay(500);
    //robo_health_arm.handleWithChuck();

    if (flag) {
        robo_health_arm.handleWithSerialPort();
    }
    if (i == 999) {
        robo_health_arm.joint_handler_.updateJointInfo();
        //robo_health_arm.joint_handler_.controlLoopSpeed();
    }
    if (i == 500) {
        i = 0;

        Serial.print("freeMemory()= ");
        Serial.println(freeMemory());

        Serial.print("ID Servo:"); Serial.print("\t\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getID()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getID()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getID());

        Serial.print("Pot value:"); Serial.print("\t\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getAnalogReadPot()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getAnalogReadPot());


        Serial.print(F("Goal pos:")); Serial.print("\t\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getPosTarget()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getPosTarget());

        Serial.print(F("Articulation position:")); Serial.print("\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getPosition()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getPosition());

        Serial.print(F("Speed calculated:")); Serial.print("\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getGoalSpeed()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getGoalSpeed());

        Serial.print(F("Error calculated:")); Serial.print("\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getError()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getError());

        Serial.print(F("Goal Torque:")); Serial.print("\t\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getGoalTorque()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getGoalTorque()); Serial.print(",\t"); Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getGoalTorque());

        Serial.print(F("Speed in servos:")); Serial.print("\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getSpeed()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getSpeed()); Serial.print(",\t"); Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getSpeed());

        Serial.println();
        Serial.println();
        flag = true;
    }

    i++;
}

#endif

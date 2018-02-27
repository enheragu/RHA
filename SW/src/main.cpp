/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */

//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress

#include <Arduino.h>
// #include "rha_types.h"
// #include "utilities.h"
// #include "servo_rha.h"
// #include "joint_handler.h"
// #include "joint_rha.h"
#include "robot_rha.h"
#include "MemoryFree.h"

RobotRHA robo_health_arm;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
    Serial.begin(921600);  // 115200 230400 250000 460800 921600
    Serial.println(F("# Start setup"));
    Serial.print(F("# TX_BUFFER = "));
    Serial.println(SERIAL_TX_BUFFER_SIZE);
    Serial.print(F("# RX_BUFFER = "));
    Serial.println(SERIAL_RX_BUFFER_SIZE);

    robo_health_arm.initJointHandler();
    robo_health_arm.initPynterface();
    Serial.println(F("#Joint Handler initialiced"));
    // robo_health_arm.initChuckHandler();

    Serial.println(F("# Init loop"));

    /*RHATypes::Point3 cartesian_pos;
    cartesian_pos.x = 0.8;
    cartesian_pos.z = 0.1;
    robo_health_arm.goToCartesianPos(cartesian_pos);*/
}

// int i = 0;
 int k = 0;

void loop() {
    robo_health_arm.handleWithPynterface();
    /*robo_health_arm.joint_handler_.updateJointInfo();
    robo_health_arm.handleRobot();

    //Serial.println();
    if (robo_health_arm.isError()) {
        while (true) {
            Serial.println("[Error] Some error ocurred while running, arm in error mode. Needs reset to work again");
            digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(500);                       // wait for a second
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
            delay(500);
            break;
        }
    }*/
     /*if (k >= 100) {
        k = 0;
*/
        /*Serial.print("freeMemory()= ");
        Serial.println(freeMemory());

        Serial.println();
        Serial.print("Servo 0:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[0].servo_.printCheckVar();
        Serial.print("Servo 1:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[1].servo_.printCheckVar();
        Serial.print("Servo 2:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[2].servo_.printCheckVar();
        Serial.print("joint 0:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[0].printCheckVar();
        Serial.print("joint 1:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[1].printCheckVar();
        Serial.print("joint 2:"); Serial.print("\t");
        robo_health_arm.joint_handler_.joint_[2].printCheckVar();
        Serial.print("joint handler:"); Serial.print("\t");
        robo_health_arm.joint_handler_.printCheckVar();
        Serial.println();*/

        /*Serial.print("ID Servo:"); Serial.print("\t\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getID()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getID()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getID());

        Serial.print("Pot pin:"); Serial.print("\t\t");
        Serial.print("-"); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].getPotentiometerPin()); Serial.print(",\t");
        Serial.println(robo_health_arm.joint_handler_.joint_[2].getPotentiometerPin());

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

        Serial.print(F("Goal dir:")); Serial.print("\t\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getDirectionTarget()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getDirectionTarget()); Serial.print(",\t"); Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getDirectionTarget());

        Serial.print(F("Speed in servos:")); Serial.print("\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[0].servo_.getSpeed()); Serial.print(",\t");
        Serial.print(robo_health_arm.joint_handler_.joint_[1].servo_.getSpeed()); Serial.print(",\t"); Serial.println(robo_health_arm.joint_handler_.joint_[2].servo_.getSpeed());

        Serial.println();
        Serial.println();
     }

     k++;*/
}

//#endif

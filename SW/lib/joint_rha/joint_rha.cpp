/**
 * @file
 * @brief Implements JointRHA functions defined in joint_rha.h
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Sep-2017
 */

#include "joint_rha.h"

/** @brief Cunstructor of JointRHA class.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected
  */
JointRHA::JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    // servo_ = new ServoRHA (servo_id);
    servo_.init(servo_id);
    up_direction_ = up_direction;
    potentiometer_pin_ = potentiometer;
}

/** @brief ~JointRHA destructor of JointRHA class.
  */
JointRHA::~JointRHA() {
}

/** @brief Initialization for JointRHA default constructor.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected
  */
void JointRHA::init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    DebugSerialJRHALn("init: begin of function");
    DebugSerialJRHALn2("init: initialazing with id: ", servo_id);
    up_direction_ = up_direction;
    potentiometer_pin_ = potentiometer;

    servo_.init(servo_id);

    pinMode(potentiometer_pin_, INPUT);
}

/**
 * @brief Updates all the information of servo object and feedback information to use it in next control iteration (in control loop)
 */
void JointRHA::updateInfo() {
    position_pot_ = digitalRead(potentiometer_pin_);
}

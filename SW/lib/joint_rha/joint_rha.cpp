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
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected. If there is no realim for this joint value will be 255
  */
JointRHA::JointRHA(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer) {
    // servo_ = new ServoRHA (servo_id);
    servo_.init(_servo_id);
    up_direction_ = _up_direction;
    potentiometer_pin_ = _potentiometer;
}

/** @brief ~JointRHA destructor of JointRHA class.
  */
JointRHA::~JointRHA() {
}

/** @brief Initialization for JointRHA default constructor.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected. If there is no realim for this joint value will be 255
  */
void JointRHA::init(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer) {
    DebugSerialJRHALn("init: begin of function");
    DebugSerialJRHALn2("init: initialazing with id: ", _servo_id);
    up_direction_ = _up_direction;
    potentiometer_pin_ = _potentiometer;
    joint_pot_relation_ = 1;

    servo_.init(_servo_id);
    if (potentiometer_pin_ != 255) pinMode(potentiometer_pin_, INPUT);
}

/**
 * @brief Updates position reading from potentiometer if there is a pot to read (not 255). Updates joint angle position
 * @method JointRHA::updatePosition
 * @return returns position value in joint reference
 */
float JointRHA::updatePosition() {
    if (potentiometer_pin_ != 255)
        return map(analogRead(potentiometer_pin_), 0, 1023, 0, 255)*joint_pot_relation_;  // from 0 to 5V transform to 0-255 degrees (max angle of potentiometer)
    else
        return -1;
}

/**
 * Sets the relation between the potentiometer angle (in grads) and the joint angle
 * @method JointRHA::setPotRelation
 * @param  _relation            relation between measures. diameter of pot gear / diameter of bar gear
 */
void JointRHA::setPotRelation(float _relation) {
    joint_pot_relation_ = _relation;
}


/**
 * @brief Updates all the information of servo object information and position feedback of joint  to use it in next control iteration (in control loop)
 * @method JointRHA::updateInfo
 * @param {uint8_t *} data data with servo information to pass to it
 * @param {uint16_t *} error  error in communication with servo
 */
void JointRHA::updateInfo(uint8_t *_data, uint16_t _error) {
    position_pot_ = updatePosition();
    servo_.updateInfo(_data, _error);
}

/**
 * @file
 * @brief Implements JointRHA functions defined in joint_rha.h
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */

#include "joint_rha.h"

/** @brief Cunstructor of JointRHA class.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected
  */
JointRHA::JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    // servo_ = new ServoRHA (servo_id);
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
 * @brief Sets speed goal to achieve with speed slope
 * @param {uint16_t} speed_target speed to achieve
 * @param {uint16_t} speed_slope slope from actual speed to speed_target (acceleration)
 * @param {uint16_t} direction_target move CW or CCW
 */

uint8_t JointRHA::setSpeedGoal(RHATypes::SpeedGoal goal) {
    DebugSerialJRHALn("setSpeedGoal: seting speed goal");
    DebugSerialJRHALn2("setSpeedGoal: servo id now is: ", servo_.getID());
    DebugSerialJRHALn2("setSpeedGoal: goal intended for id: ", goal.servo_id);
    if (servo_.getID() == goal.servo_id) {
        speed_slope_ = goal.speed_slope;
        speed_target_ = goal.speed;
        direction_target_ = goal.direction;
        time_last_ = millis();
        DebugSerialJRHALn2("setSpeedGoal: Speed set to: ", speed_target_);
        DebugSerialJRHALn2("setSpeedGoal: Speed slope set to: ", speed_slope_);
        return true;
    } else return false;
}

/**
 * @brief Calculates error to send to servo regulator
 * @return {uint8_t} returns error between actual position and target position
 */
float JointRHA::speedError() {
    uint16_t speed = 0;
    if (speed_slope_ != 0) {
        speed = (float)servo_.getSpeed() + (float)(millis() - time_last_) * speed_slope_;
        if (speed > speed_target_) speed = (float)speed_target_;
        time_last_ = millis();
    } else speed = speed_target_;
    uint8_t sign = 1;
    if (direction_target_ != servo_.getSpeedDir()) sign = -1;
    return (sign*(float)speed - (float)servo_.getSpeed());
}

/**
 * @brief Updates all the information of servo object and feedback information to use it in next control iteration (in control loop)
 */
void JointRHA::updateInfo() {
    position_pot_ = digitalRead(potentiometer_pin_);
}

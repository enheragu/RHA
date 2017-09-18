/**
 * @file
 * @brief Implements JointRHA functions defined in joint_rha.h
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
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
void JointRHA::setGoal(uint16_t speed_target, uint16_t speed_slope, uint8_t direction_target) {
    speed_target_ = speed_target;
    speed_slope_ = speed_slope;
    direction_target_ = direction_target;
    time_last_ = millis();
}

uint8_t setSpeedGoal(SpeedGoal goal) {
    if (goal.servo_id == servo_.servo_id_) {
        speed_slope_ = goal.speed_slope;
        speed_target_ = goal.speed;
        return true;
    } else return false;
}

/**
 * @brief Calculates error to send to servo regulator
 * @return {uint8_t} returns error between actual position and target position
 */
uint8_t JointRHA::speedError() {
    uint16_t speed = servo_.getSpeed() + (millis() - time_last_) * speed_slope_;
    if (speed > speed_target_) speed = speed_target_;
    time_last_ = millis();
    return (speed - servo_.getSpeed());
}

/**
 * @brief Updates all the information of servo object and feedback information to use it in next control iteration (in control loop)
 */
void JointRHA::updateInfo() {
    position_pot_ = digitalRead(potentiometer_pin_);
    servo_->updateInfo();
}

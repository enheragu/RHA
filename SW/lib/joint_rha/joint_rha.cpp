/**
 * @file
 * @brief Implements JointRHA functions defined in joint_rha.h
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 09-Sep-2017
 */

#include "joint_rha.h"

/** @brief Cunstructor of JointRHA class.
  * @param {uint8_t} servo_id servo id controlled by this joint
  * @param {uint8_t} up_direction direction in which the servo has to move (CW or CCW) so the joint moves up.
  * @param {uint8_t} potentiometer pin in which the potentiometer for this joint is connected
  */
JointRHA::JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    servo_ = new ServoRHA (servo_id);
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
void JointRHA::init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer){
    up_direction_ = up_direction;
    potentiometer_pin_ = potentiometer;

    servo_.init(servo_id, 2, 3, 8, 19200);

    pinMode(potentiometer_pin_, INPUT);
}

/**
 * @brief Sets speed goal to achieve with speed slope
 * @param {uint16_t} speed_target speed to achieve
 * @param {uint16_t} speed_slope slope from actual speed to speed_target (acceleration)
 * @param {uint16_t} direction_target move CW or CCW
 */
void JointRHA::setGoal (uint16_t speed_target, uint16_t speed_slope, uint8_t direction_target){
    speed_target_ = speed_target;
    speed_slope_ = speed_slope;
    direction_target_ = direction_target;
    time_last_ = millis();
}

/**
 * @brief Calculates error to send to servo regulator
 * @return {uint8_t} returns error between actual position and target position
 */
uint8_t JointRHA::speedError(){
    uint16_t speed = servo_.getSpeed() + (millis() - time_last_) * speed_slope_;
    if (speed > speed_target_) speed = speed_target_;
    time_last_ = millis();
    return (speed - servo_.getSpeed());
}

/**
 * @brief Calls regulatorServo. Serves as interface with servo methods
 * @param  {uint16_t} error error input to the regulator
 * @return {uint16_t} regulator output (torque to send to servo)
 * @see ServoRHA::regulatorServo()
 */
uint16_t JointRHA::regulatorJoint(uint16_t error){
    return servo_.regulatorServo(error);
}


/** @brief Add goals from servos of this joint to packet. Implements interface to servo.addToPacket()
  * @param buffer is the buffer in which the information will be added (by reference)
  * @param position is the position from which it writes the new info (by reference)
  * @param instruction is the instruction that is being sended in this packet
  * @param goal the goal to send. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param goal_len length of the goal (uint8_ts)
  * @param num_servo how many servos had been added to this packet
  * @see ServoRHA::addToPacket();
  */
void ServoRHA::addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo) {
    servo_.addToPacket(buffer, position, goal, goal_len, num_servo);
}

/** @brief Implements an interface to call servo_ wrapPacket
  * @param buffer is the buffer in which the information will be added (by reference)
  * @param data is the data that have been completed by each servo (by reference)
  * @param data_len is the length of data
  * @param instruction is the instruction to send
  * @param num_servo how many servos had been added to this packet
  * @return Returns number of uint8_ts that contain usefull info (how many have been written)
  * @see ServoRHA::wrapPacket();
  */
uint8_t JointRHA::wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo) {
    return servo_.wrapPacket(buffer, data, data_len, instruction, num_servo);
}

uint16_t sendPacket(uint8_t instruction, uint8_t* data, uint8_t parameterLength){
  return servo_.sendPacket(servo_.getID(), instruction, data, parameterLength);
}

/**
 * @brief Updates all the information of servo object and feedback information to use it in next control iteration (in control loop)
 */
void JointRHA::updateInfo() {
    position_pot_ = digitalRead(potentiometer_pin_);
    servo_->updateInfo();
}

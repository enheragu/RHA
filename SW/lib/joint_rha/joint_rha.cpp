#include "joint_rha.h"

/** @brief JointRHA cunstructor of JointRHA class.
  * @param servo_id: servo id controlled by this joint
  * @param up_direction: direction in which the servo has to move (CW or CCW) so the joint moves up.
  */
JointRHA::JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    //  servo_ = new ServoRHA (servo_id);
    up_direction_ = up_direction;
    potentiometer_ = potentiometer;
}

/** @brief ~JointRHA destructor of JointRHA class.
  */
JointRHA::~JointRHA() {
    //  delete servo_;
}

void JointRHA::initJoint() {
    pinMode(potentiometer_, INPUT);
}

int16_t JointRHA::readFeedbackPosition() {
  return digitalRead(potentiometer_);
}

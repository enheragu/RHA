#include "joint_rha.h"

/** @brief JointRHA cunstructor of JointRHA class.
  * @param servo_id: servo id controlled by this joint
  * @param up_direction: direction in which the servo has to move (CW or CCW) so the joint moves up.
  */
JointRHA::JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer) {
    //  servo_ = new ServoRHA (servo_id);
    up_direction_ = up_direction;
    potentiometer_pin_ = potentiometer;
}

/** @brief ~JointRHA destructor of JointRHA class.
  */
JointRHA::~JointRHA() {
    //  delete servo_;
}

void JointRHA::init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer){
    up_direction_ = up_direction;
    potentiometer_pin_ = potentiometer;

    //servo_.init(servo_id, 2, 3, 8, 19200);

    pinMode(potentiometer_pin_, INPUT);
}


void JointRHA::updateInfo() {
  position_ = digitalRead(potentiometer_pin_);
  //servo_->updateInfo();
}

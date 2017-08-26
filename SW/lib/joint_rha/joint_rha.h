#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "servo_rha.h"
#include "Arduino.h"

class ServoRHA;

class JointRHA {
  // ServoRHA* servo_;
  uint8_t up_direction_, potentiometer_;

 public:
  JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  ~JointRHA();

  void initJoint();

  int16_t readFeedbackPosition();
 private:
};

#endif

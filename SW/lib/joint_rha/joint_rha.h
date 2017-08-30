#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "servo_rha.h"
#include "Arduino.h"

class ServoRHA;

class JointRHA {
  //ServoRHA servo_;
  uint8_t up_direction_, potentiometer_pin_;
  uint8_t position_;

 public:
  JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  JointRHA();
  ~JointRHA();

  void init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);

  void updateInfo();
 private:
};

#endif

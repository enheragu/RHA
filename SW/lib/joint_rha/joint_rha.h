#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "servo_rha.h"
#include "Arduino.h"

class ServoRHA;

class JointRHA {
  ServoRHA servo_;
  uint8_t up_direction_, potentiometer_pin_;
  uint8_t direction_target_;
  uint16_t speed_slope_, speed_target_;
  uint64_t time_last_;

 public:
  JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  JointRHA();
  ~JointRHA();

  void init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  void setGoal (uint16_t speed_target, uint16_t speed_slope, uint8_t direction_target);

  void addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo);
  uint8_t wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo);

  uint8_t speedError();
  uint16_t regulatorJoint(uint16_t error);
  void updateInfo();

  uint16_t sendPacket(uint8_t id, uint8_t instruction, uint8_t* data, uint8_t parameterLength);
 private:
};

#endif

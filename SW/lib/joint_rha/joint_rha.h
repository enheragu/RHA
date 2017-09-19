/**
 * @file
 * @brief Implements JointRHA class. This object combines potentiometer with ServoRHA object readings to enhance it's functionality
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.h
 * @Last modified by:   enheragu
 * @Last modified time: 19_Sep_2017
 */

#ifndef JOINT_RHA_H
#define JOINT_RHA_H

#include "servo_rha.h"
#include "Arduino.h"
#include <SoftwareSerial.h>

class ServoRHA;

class JointRHA {
  uint8_t up_direction_, potentiometer_pin_, position_pot_;
  uint8_t direction_target_;
  uint16_t speed_slope_, speed_target_;
  uint64_t time_last_;

 public:
  ServoRHA servo_;
  JointRHA(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  JointRHA() {}
  ~JointRHA();

  void init(uint8_t servo_id, uint8_t up_direction, uint8_t potentiometer);
  void setGoal(uint16_t speed_target, uint16_t speed_slope, uint8_t direction_target);
  uint8_t setSpeedGoal(SpeedGoal goal);

  uint8_t speedError();
  void updateInfo();

  uint16_t getSpeedTarget() { return speed_target_; }

 private:
};

#endif

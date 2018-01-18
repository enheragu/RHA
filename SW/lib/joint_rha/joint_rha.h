/**
 * @file
 * @brief Implements JointRHA class. This object combines potentiometer with ServoRHA object readings to enhance it's functionality
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_rha.h
 * @Last modified by:   quique
 * @Last modified time: 29-Sep-2017
 */

#ifndef JOINT_RHA_H
#define JOINT_RHA_H


#include "servo_rha.h"
#include "rha_types.h"
#include "debug.h"

#include "Arduino.h"
#include <SoftwareSerial.h>


class ServoRHA;

class JointRHA {
  uint8_t up_direction_, potentiometer_pin_, position_pot_;
  float joint_pot_relation_;

  uint32_t pot_max_value, pot_min_value;
  uint8_t angle_max_value, angle_min_value;

 public:
  ServoRHA servo_;
  JointRHA(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer = 255);
  JointRHA() {}
  ~JointRHA();

  void init(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer = 255);
  void setPotRelation(float _relation = 1);
  void initPotMeasurment(uint32_t _pot_min_value, uint32_t _pot_max_value, uint8_t _angle_min_value, uint8_t _angle_max_value);
  uint8_t setSpeedGoal(RHATypes::SpeedGoal _goal);

  float updatePosition();
  void updateInfo(uint8_t *_data, uint16_t _error);

 private:
};

#endif

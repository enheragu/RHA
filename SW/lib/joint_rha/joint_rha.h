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

namespace JointRHAConstants {

#define ANGLE_TOLERANCE 2

namespace SpeedRegulatorK {
    #define KP 0.75  // 1.66
    #define KD 0
    #define KI 0
}
}  // namespace JointRHAConstants

class ServoRHA;

class JointRHA {
  bool up_direction_;
  uint8_t potentiometer_pin_;
  float position_pot_;
  float joint_pot_relation_;

  uint32_t pot_max_value, pot_min_value;
  uint8_t angle_max_value, angle_min_value;

  int position_target_;
  float pos_error_, pos_last_error_, pos_derror_, pos_ierror_;
  uint64_t time_last_error_;

  RHATypes::SpeedGoal goal_speed_;

 public:
  RHATypes::Regulator speed_regulator_;

  ServoRHA servo_;
  JointRHA(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer = 245);
  JointRHA();
  ~JointRHA();

  void init(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer = 245);
  void setPotRelation(float _relation = 1);
  void initPotMeasurment(uint32_t _pot_min_value, uint32_t _pot_max_value, uint8_t _angle_min_value, uint8_t _angle_max_value);
  uint8_t setSpeedGoal(RHATypes::SpeedGoal _goal);

  void updatePosition();
  void updateInfo(uint8_t *_data, uint16_t _error);

  void setPositionGoal(int _position);
  void posError();
  void calculateSpeed(float _error = 0, float _derror = 0, float _ierror = 0);
  void updateServoSpeedGoal();

  bool reachedGoalPosition();
  float getPosition() { return position_pot_; }
  float getGoalSpeed() { return goal_speed_.speed; }
  int getPosTarget() { return position_target_; }
  float getError() { return pos_error_; }
  float getDError() { return pos_derror_; }
  float getIError() { return pos_ierror_; }
};

#endif

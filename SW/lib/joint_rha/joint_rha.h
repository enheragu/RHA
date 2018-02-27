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


#define ANGLE_TOLERANCE 0
#define NO_POTENTIOMETER 255
#define ERROR_MOVING_MARGIN 5  /**< It the servo moves but not the joint for more than ERROR_MOVING_MARGIN cicles it raiseses an error*/
#define SpeedRegulatorK_KP 5  // 1.66
#define SpeedRegulatorK_KD 0.2
#define SpeedRegulatorK_KI 0

#define FILTER_POTENTIOMETER 5

class ServoRHA;

class JointRHA {
    uint8_t empty_var;
    uint8_t empty_var_2;
    bool up_direction_;
    uint8_t potentiometer_pin_;
    int pot_analog_read_;
    float position_pot_, position_pot_last_;
    uint8_t error_moving_;  /**<  increments whenever the servo is moving but the joint is not >*/
    float zero_pos_compensation_;
    float joint_pot_relation_;

    int position_target_;
    float pos_error_, pos_last_error_, pos_derror_, pos_ierror_;
    uint64_t time_last_error_;
    uint8_t joint_ok_;

    /*float pos_filter_vector[FILTER_POTENTIOMETER];
    uint8_t iterator_filter;*/

    RHATypes::SpeedGoal goal_speed_;

 public:
    void printCheckVar() { Serial.print("Value is: "); Serial.print(empty_var); Serial.print("\t"); Serial.print("Value 2 is: "); Serial.println(empty_var_2);}
    RHATypes::Regulator speed_regulator_;

    ServoRHA servo_;
    JointRHA(uint8_t _servo_id, uint8_t _up_direction, uint8_t _potentiometer = NO_POTENTIOMETER);
    JointRHA();
    ~JointRHA();

    void init(uint8_t _servo_id, uint8_t _up_direction, float _zero_compensation = 0, uint8_t _potentiometer = NO_POTENTIOMETER);
    void setPotRelation(float _relation = 1);
    void initPotMeasurment(uint32_t _pot_min_value, uint32_t _pot_max_value, uint8_t _angle_min_value, uint8_t _angle_max_value);
    uint8_t setSpeedGoal(RHATypes::SpeedGoal _goal);

    void updatePosition();
    void updateInfo(uint8_t *_data, uint16_t _error);

    void setPositionGoal(int _position);
    void posError();
    void calculateSpeed(float _error = 0, float _derror = 0, float _ierror = 0);
    void updateServoSpeedGoal();
    bool checkSecurityJoint();
    bool checkSecurity() { return joint_ok_; }

    bool reachedGoalPosition();
    float getPosition() { return position_pot_; }
    float getGoalSpeed() { return goal_speed_.speed; }
    int getPosTarget() { return position_target_; }
    float getError() { return pos_error_; }
    float getDError() { return pos_derror_; }
    float getIError() { return pos_ierror_; }
    int getAnalogReadPot() { return pot_analog_read_; }
    int getPotentiometerPin() { return potentiometer_pin_; }
    int getPositionTarget() { return position_target_; }
};

#endif

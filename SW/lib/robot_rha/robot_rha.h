/**
 * @Author: Enrique Heredia Aguado <gmv>
 * @Date:   31_Oct_2017
 * @Project: RHA
 * @Filename: robot_rha.h
 * @Last modified by:   enheragu
 * @Last modified time: 31_Oct_2017
 */

#ifndef ROBOT_RHA_H
#define ROBOT_RHA_H

#include "chuck_handler.h"
#include "joint_handler.h"

#define TORQUE_CONTROL_PERIOD (3 * NUM_JOINT)  // 50
#define SPEED_CONTROL_PERIOD (TORQUE_CONTROL_PERIOD * 10)
#define CHUCK_UPDATE_PERIOD 100

#define G15_BAUDRATE 460800
#define G15_RX_PIN 17
#define G15_TX_PIN 16
#define G15_CONTRL_PIN 8

#define J1 1
#define J2 2
#define J3 3

class RobotRHA {
 private:
    bool first_time_serial_goal_;
 public:
    RobotRHA() : first_time_serial_goal_(true) {}
    void initJointHandler();
    void initChuckHandler();

    void handleRobot();
    void setCartesianSpeedGoal(float _speed_x, float _speed_y, float _speed_z);
    void setSpeedToServos(float _speed, uint8_t _servo_id);

    void handleWithChuck();
    void handleWithSerialPort();
    int getGoalFromSerialInput(int _joint_target);

    JointHandler joint_handler_;
    ChuckHandler chuck_handler_;
};
#endif

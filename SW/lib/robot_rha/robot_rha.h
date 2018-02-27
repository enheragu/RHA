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

#include <math.h>
#include <Arduino.h>

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

#define degreesToRad(angle) (float)((angle) * PI / 180)
#define radToDegrees(position) (float)((position) * 180 / PI)

namespace PynterfaceConstants {
    // Serial1 on pins 19 (RX) and 18 (TX)
    // Serial2 on pins 17 (RX) and 16 (TX)
    // Serial3 on pins 15 (RX) and 14 (TX)
    // Check what serial is used by joint_handler
    #define Serial_PYNTERFACE Serial
    #define PYNTERFACE_MSG_LENGTH 25
    #define PYNTERFACE_BAUDRATE 921600
    #define SEND_PYNTERFACE_DELAY 500  // PYNTERFACE_MSG_LENGTH/(PYNTERFACE_BAUDRATE*0.125*0.001)  // Baudrate * 0.125 = bytes/s; *0.001 -> bytes/milisecond
    enum package_content {UPDATE_INFO = 0, ERROR, ARTICULAR_GOAL};
}

namespace MechanicalConstantMesaures {
    #define L1 0.3070
    #define L2 0.4550
    #define L3 0.4550
    #define LA 0.0300
    #define LB 0.0420
}

class RobotRHA {
 private:
    bool first_time_serial_goal_;
    bool robot_error_;
    uint8_t buffer_[PYNTERFACE_MSG_LENGTH];

 public:
    RHATypes::Timer send_pynterface_data_;

    RobotRHA() : first_time_serial_goal_(true) {}
    void initJointHandler();
    void initChuckHandler();

    void handleRobot(uint8_t _calibration = false);
    void setCartesianSpeedGoal(float _speed_x, float _speed_y, float _speed_z);
    void setSpeedToServos(float _speed, uint8_t _servo_id);

    void initPynterface();
    void handleWithPynterface();
    bool sendPackage();
    void getPackage();


    void handleWithChuck();
    void handleWithSerialPort();
    int getGoalFromSerialInput(int _joint_target);

    void updateInfo();

    void goToCartesianPos(RHATypes::Point3 _cartesian_pos);
    void goToArticularPos(RHATypes::Point3 _articular_pos);

    void calibration();

    bool checkError();
    RHATypes::Point3 getCartesianPos() { return cartesian_position_; }

    RHATypes::Point3 forwardKinematics(RHATypes::Point3 _articular_pos);
    RHATypes::Point3 inverseKinematics(RHATypes::Point3 _cartesian_pos);

    JointHandler joint_handler_;
    //ChuckHandler chuck_handler_;
    RHATypes::Point3 articular_position_, cartesian_position_, pynterface_goal_;

    bool isError() { return joint_handler_.isError(); }

    void resetBuffer(uint8_t buffer[]);
};
#endif

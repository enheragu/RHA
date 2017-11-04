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

#define SPEED_CONTROL_PERIOD 50
#define CHUCK_UPDATE_PERIOD 100

#define G15_BAUDRATE 460800
#define G15_RX_PIN 19
#define G15_TX_PIN 18
#define G15_CONTRL_PIN 8

#define J1 1
#define J2 2
#define J3 3

class RobotRHA {
 protected:
    JointHandler joint_handler_;
    ChuckHandler chuck_handler_;
 public:
    void initJointHandler();
    void initChuckHandler();

    void handleRobot();
    void setCartesianSpeedGoal(float speed_x, float speed_y, float speed_z);
    void setSpeedToServos(float speed, uint8_t servo_id);

    void handleWithChuck();
};
#endif

#include "robot_rha.h"

void RobotRHA::initJointHandler() {
    joint_handler.setTimer(SPEED_CONTROL_PERIOD);
    joint_handler.initSerial(G15_RX_PIN,G15_TX_PIN,G15_CONTRL_PIN,G15_BAUDRATE);  // baudrate 460800 means 57.6 bytes/milisecond
    joint_handler.initJoints();
}

void RobotRHA::initChuckHandler() {
    chuck_handler_.setTimer(CHUCK_UPDATE_PERIOD);
}

void RobotRHA::handleRobot() {

}

void RobotRHA::setCartesianSpeedGoal(float speed_x, float speed_y, float speed_z) {
    setSpeedToServos(speed_x, J1);
    setSpeedToServos(speed_y, J2);
    setSpeedToServos(speed_z, J3);
}

void RobotRHA::setSpeedToServos(float speed, uint8_t servo_id) {
    bool direction = (speed > 0) ? CW : CCW;
    RHATypes::SpeedGoal speed_goal(servo_id, abs(speed), 0, direction);
    joint_handler_.setSpeedGoal(speed_goal);
}


void RobotRHA::handleWithChuck() {
    ChuckReadStruct speed_commands = chuck_handler_.readAxis();
    if (speed_commands.updated_) setCartesianSpeedGoal(speed_commands.X_, speed_commands.Y_, speed_commands.Z_);

    joint_handler_.controlLoop();

}

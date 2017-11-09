#include "robot_rha.h"

/**
 * @brief Inits Joint handler timer, serial and joints
 * @method RobotRHA::initJointHandler
 */
void RobotRHA::initJointHandler() {
    joint_handler_.setTimer(SPEED_CONTROL_PERIOD);
    joint_handler_.initSerial(G15_RX_PIN,G15_TX_PIN,G15_CONTRL_PIN,G15_BAUDRATE);  // baudrate 460800 means 57.6 bytes/milisecond
    joint_handler_.initJoints();
}

/**
 * @brief Inits chuck handler timer
 * @method RobotRHA::initChuckHandler
 */
void RobotRHA::initChuckHandler() {
    chuck_handler_.begin();
    chuck_handler_.setTimer(CHUCK_UPDATE_PERIOD);
}

void RobotRHA::handleRobot() {

}

/**
 * @brief Sets X, Y, X speed to Joints applyin transformation
 * @method RobotRHA::setCartesianSpeedGoal
 * @param  _speed_x       speed in X
 * @param  _speed_y       speed in Y
 * @param  _speed_z       speed in Z
 */
void RobotRHA::setCartesianSpeedGoal(float _speed_x, float _speed_y, float _speed_z) {
    DebugSerialRRHALn("setCartesianSpeedGoal: begin of function");
    DebugSerialRRHALn2("setCartesianSpeedGoal: speed_x is: ", _speed_x);
    DebugSerialRRHALn2("setCartesianSpeedGoal: speed_y is: ", _speed_y);
    DebugSerialRRHALn2("setCartesianSpeedGoal: speed_z is: ", _speed_z);
    setSpeedToServos(_speed_x, J1);
    setSpeedToServos(_speed_y, J2);
    setSpeedToServos(_speed_z, J3);
}

/**
 * @brief Sends speed goal
 * @method RobotRHA::setSpeedToServos
 * @param  _speed         speed to set
 * @param  _servo_id      target servo for this goal speed
 */
void RobotRHA::setSpeedToServos(float _speed, uint8_t _servo_id) {
    DebugSerialRRHALn("setSpeedToServos: begin of function");
    bool direction = (_speed > 0) ? CW : CCW;
    RHATypes::SpeedGoal speed_goal(_servo_id, abs(_speed), 0, direction);
    joint_handler_.setSpeedGoal(speed_goal);
}

/**
 * @brief Handles all GDL with nunchuck input. This method calls chuck reading ,sets speed goals and calls Joint handler controlLoop
 * @method RobotRHA::handleWithChuck
 */
void RobotRHA::handleWithChuck() {
    DebugSerialRRHALn("handleWithChuck: begin of function");
    DebugSerialRRHALn("handleWithChuck: getting speed commands");
    ChuckReadStruct speed_commands = chuck_handler_.readAxis();
    DebugSerialRRHALn("handleWithChuck: setting goal speed to servos");
    if (speed_commands.updated_) setCartesianSpeedGoal(speed_commands.X_, speed_commands.Y_, speed_commands.Z_);
    //else setCartesianSpeedGoal(speed_commands.X_, speed_commands.Y_, speed_commands.Z_);
    DebugSerialRRHALn("handleWithChuck: calling joint_handler control loop");
    joint_handler_.controlLoop();
}

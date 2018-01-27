#include "robot_rha.h"

/**
 * @brief Inits Joint handler timer, serial and joints
 * @method RobotRHA::initJointHandler
 */
void RobotRHA::initJointHandler() {
    DebugSerialRRHALn("initJointHandler: begin of function");
    joint_handler_.setTorqueControlTimer(TORQUE_CONTROL_PERIOD);
    joint_handler_.setSpeedControlTimer(SPEED_CONTROL_PERIOD);
    joint_handler_.initSerial(G15_RX_PIN, G15_TX_PIN, G15_CONTRL_PIN, G15_BAUDRATE);  // baudrate 460800 means 57.6 bytes/milisecond
    joint_handler_.initJoints();
    DebugSerialRRHALn("initJointHandler: end of function");
}

/**
 * @brief Inits chuck handler timer
 * @method RobotRHA::initChuckHandler
 */
void RobotRHA::initChuckHandler() {
    DebugSerialRRHALn("initChuckHandler: begin of function");
    chuck_handler_.begin();
    chuck_handler_.setTimer(CHUCK_UPDATE_PERIOD);
    DebugSerialRRHALn("initChuckHandler: end of function");
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
    // ChuckReadStruct speed_commands = ChuckReadStruct(20, 20, 20, true);
    ChuckReadStruct speed_commands = chuck_handler_.readAxis();
    DebugSerialRRHALn("handleWithChuck: setting goal speed to servos");
    if (speed_commands.updated_) {
        // Chuck returns % in speed.
        DebugSerialRRHALn("handleWithChuck: speed command was updated");
        int speed_x = (speed_commands.X_) * 100/ MAX_SPEED_VALUE;
        int speed_y = (speed_commands.Y_) * 100/ MAX_SPEED_VALUE;
        int speed_z = (speed_commands.Z_) * 100/ MAX_SPEED_VALUE;
        setCartesianSpeedGoal(speed_x, speed_y, speed_z);
    }
    // else setCartesianSpeedGoal(speed_commands.X_, speed_commands.Y_, speed_commands.Z_);
    DebugSerialRRHALn("handleWithChuck: calling joint_handler control loop for servo torque");
    joint_handler_.controlLoopTorque();
}

/**
 * @brief Handles all GDL with serial port input. Ask Goal position for all GDL and then goes to it
 * @method RobotRHA::handleWithSerialPort
 */
void RobotRHA::handleWithSerialPort() {
    DebugSerialRRHALn("handleWithSerialPort: begin of function");
    int goal_pos_joint_0, goal_pos_joint_1, goal_pos_joint_2;
    if (joint_handler_.joint_[0].reachedGoalPosition() && joint_handler_.joint_[2].reachedGoalPosition()) {
        goal_pos_joint_0 = getGoalFromSerialInput(0);
        // goal_pos_joint_1 = getGoalFromSerialInput(2);
        goal_pos_joint_2 = getGoalFromSerialInput(2);

        joint_handler_.joint_[0].setPositionGoal(goal_pos_joint_0);
        joint_handler_.joint_[2].setPositionGoal(goal_pos_joint_2);
    }

    DebugSerialRRHALn("handleWithSerialPort: calling joint_handler control loop for ServoRHA speed");
    joint_handler_.controlLoopSpeed();
    DebugSerialRRHALn("handleWithSerialPort: calling joint_handler control loop for servo torque");
    joint_handler_.controlLoopTorque();
}

/**
  * @brief Asks a position to go through the serial port and returns it
  * @method RobotRHA::getGoalFromSerialInput
  * @param  _joint_target         joint for which the goal is intended
 */
int RobotRHA::getGoalFromSerialInput(int _joint_target) {
    Serial.print("Join to command is now in pos = "); Serial.println(joint_handler_.joint_[_joint_target].getPosition());
    Serial.print("Introduce goal position for joint: "); Serial.println(_joint_target);
    while(Serial.available() <= 0) {
        delay(1);  // Wait for msg, if theres no msg do nothing
    }
    int incoming_value = Serial.read();
    Serial.print("Going to "); Serial.print(incoming_value); Serial.println(" position");
    return incoming_value;
}

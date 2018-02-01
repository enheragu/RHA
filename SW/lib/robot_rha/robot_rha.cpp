#include "robot_rha.h"

/**
 * @brief Inits Joint handler timer, serial and joints
 * @method RobotRHA::initJointHandler
 */
void RobotRHA::initJointHandler() {
    DebugSerialRRHALn("initJointHandler: begin of function");
    joint_handler_.setTorqueControlTimer(TORQUE_CONTROL_PERIOD);
    joint_handler_.setSpeedControlTimer(SPEED_CONTROL_PERIOD);
    joint_handler_.initSerial();  // baudrate 460800 means 57.6 bytes/milisecond
    joint_handler_.initJoints();
    DebugSerialRRHALn("initJointHandler: end of function");
}



void RobotRHA::handleRobot() {
}

void RobotRHA::goToCartesianPos(RHATypes::Point3 _cartesian_pos) {
    RHATypes::Point3 articular_pos = inverseKinematics(_cartesian_pos);
    goToArticularPos(articular_pos);
}

void RobotRHA::goToArticularPos(RHATypes::Point3 _articular_pos) {
    joint_handler_.joint_[0].setPositionGoal( _articular_pos.x);
    joint_handler_.joint_[1].setPositionGoal( _articular_pos.y);
    joint_handler_.joint_[2].setPositionGoal( _articular_pos.z);
}

RHATypes::Point3 RobotRHA::forwardKinematics (RHATypes::Point3 _articular_pos) {
    RHATypes::Point3 cartesian_pos;
    cartesian_pos.x = L1*cos(_articular_pos.y) + L2*cos(_articular_pos.z) + L3_X;
    cartesian_pos.z = L1*sin(_articular_pos.y) - L2*sin(_articular_pos.z) + L3_Y;
    return cartesian_pos;
}

RHATypes::Point3 RobotRHA::inverseKinematics (RHATypes::Point3 _cartesian_pos) {
    RHATypes::Point3 articular_pos;
    float xa = _cartesian_pos.x - L3_X;
    float ya = _cartesian_pos.z - L3_Y;
    // get q2:
    // float r = sqrt(pow(xa,2)+pow(ya,2));
    // float num = pow(r,2) + pow(L1,2) - pow(L2,2);
    // float dem = 2*r*L1;
    // float beta = acos(num/dem);
    articular_pos.y = atan2(ya,xa) + acos((pow((sqrt(pow(xa,2)+pow(ya,2))),2) + pow(L1,2) - pow(L2,2))/(2*(sqrt(pow(xa,2)+pow(ya,2)))*L1));
    //get q3:
    // float num2 = pow(L1,2) + pow(L2,2) - pow((sqrt(pow(xa,2)+pow(ya,2))),2);
    // float dem2 = 2*L1*L2;
    //float gamma = acos((pow(L1,2) + pow(L2,2) - pow((sqrt(pow(xa,2)+pow(ya,2))),2))/(2*L1*L2));
    articular_pos.z = PI - acos((pow(L1,2) + pow(L2,2) - pow((sqrt(pow(xa,2)+pow(ya,2))),2))/(2*L1*L2)) - articular_pos.y;

    return articular_pos;
}

void updateInfo() {
    //articular_position_, cartesian_position_;
}

/**
 * @brief Inits chuck handler timer
 * @method RobotRHA::initChuckHandler
 */
/*void RobotRHA::initChuckHandler() {
    DebugSerialRRHALn("initChuckHandler: begin of function");
    chuck_handler_.begin();
    chuck_handler_.setTimer(CHUCK_UPDATE_PERIOD);
    DebugSerialRRHALn("initChuckHandler: end of function");
}*/

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
    //bool direction = (_speed > 0) ? CW : CCW;
    RHATypes::SpeedGoal speed_goal(_servo_id, abs(_speed), (_speed>0)?CW:CCW);
    joint_handler_.setSpeedGoal(speed_goal);
}

/**
 * @brief Handles all GDL with nunchuck input. This method calls chuck reading ,sets speed goals and calls Joint handler controlLoop
 * @method RobotRHA::handleWithChuck
 */
/*void RobotRHA::handleWithChuck() {
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
}*/

/**
 * @brief Handles all GDL with serial port input. Ask Goal position for all GDL and then goes to it
 * @method RobotRHA::handleWithSerialPort
 */
void RobotRHA::handleWithSerialPort() {
    DebugSerialRRHALn("handleWithSerialPort: begin of function");
    if (joint_handler_.joint_[0].reachedGoalPosition() && joint_handler_.joint_[2].reachedGoalPosition() || first_time_serial_goal_) {
        // gjoint_handler_.joint_[0].setPositionGoal( getGoalFromSerialInput(0));
        // joint_handler_.joint_[1].setPositionGoal( getGoalFromSerialInput(2));
        // joint_handler_.joint_[2].setPositionGoal( getGoalFromSerialInput(2));

        /*while (true) {
            delay(50);
        }*/
        joint_handler_.joint_[1].setPositionGoal( 60 );// goal_pos_joint_0);
        RHATypes::SpeedGoal goal0;
        joint_handler_.joint_[0].servo_.setSpeedGoal(goal0);
        joint_handler_.joint_[2].servo_.setSpeedGoal(goal0);
        //joint_handler_.joint_[2].setPositionGoal(goal_pos_joint_2);
        first_time_serial_goal_ = false;
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
    Serial.print("Joint to command is now in pos = "); Serial.println(joint_handler_.joint_[_joint_target].getPosition());
    Serial.print("Introduce goal position for joint: "); Serial.println(_joint_target);
    while(Serial.available() <= 0) {
        delay(1);  // Wait for msg, if theres no msg do nothing
    }
    if (Serial.available() > 0) {
        int incoming_value = Serial.read();
        Serial.print("Going to "); Serial.print(incoming_value); Serial.println(" position");
        return incoming_value;
    }
}

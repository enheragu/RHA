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
    DebugSerialRRHALn("handleRobot: begin of function");
    updateInfo();
    robot_error_ = checkError();
    if (robot_error_){
        DebugSerialRRHALn("handleRobot: error detected. Not going through control loops");
        return;
    } else {
        joint_handler_.controlLoopTorque();
        joint_handler_.controlLoopSpeed();
    }
}


void RobotRHA::goToCartesianPos(RHATypes::Point3 _cartesian_pos) {
    DebugSerialRRHALn("goToCartesianPos: begin of function");
    DebugSerialRRHALn("goToCartesianPos: getting goal as articular space goal");
    DebugSerialRRHALn("goToCartesianPos: begin of function");
    DebugSerialRRHALn("goToCartesianPos: Going to pos:");
    DebugSerialRRHALn2("x: ", _cartesian_pos.x);
    DebugSerialRRHALn2("y: ", _cartesian_pos.y);
    DebugSerialRRHALn2("z: ", _cartesian_pos.z);
    DebugSerialRRHALn("goToCartesianPos: From pos:");
    DebugSerialRRHALn2("x: ", cartesian_position_.x);
    DebugSerialRRHALn2("y: ", cartesian_position_.y);
    DebugSerialRRHALn2("z: ", cartesian_position_.z);
    RHATypes::Point3 articular_pos = inverseKinematics(_cartesian_pos);
    goToArticularPos(articular_pos);
}

void RobotRHA::goToArticularPos(RHATypes::Point3 _articular_pos) {
    DebugSerialRRHALn("goToArticularPos: begin of function");
    DebugSerialRRHALn("goToArticularPos: Going to pos:");
    DebugSerialRRHALn2("q1: ", _articular_pos.x);
    DebugSerialRRHALn2("q2: ", _articular_pos.y);
    DebugSerialRRHALn2("q3: ", _articular_pos.z);
    DebugSerialRRHALn("goToArticularPos: From pos:");
    DebugSerialRRHALn2("q1: ", articular_position_.x);
    DebugSerialRRHALn2("q2: ", articular_position_.y);
    DebugSerialRRHALn2("q3: ", articular_position_.z);
    joint_handler_.joint_[0].setPositionGoal( _articular_pos.x);
    joint_handler_.joint_[1].setPositionGoal( _articular_pos.y);
    joint_handler_.joint_[2].setPositionGoal( _articular_pos.z);
}

RHATypes::Point3 RobotRHA::forwardKinematics (RHATypes::Point3 _articular_pos) {
    RHATypes::Point3 cartesian_pos;

    cartesian_pos.x  = LA + L2*cos(PI/2-degreesToRad(180-_articular_pos.y)) + L2*cos(degreesToRad(180-_articular_pos.z)-PI/2);
    cartesian_pos.z = LB + L2*sin(PI/2-degreesToRad(180-_articular_pos.y)) - L2*sin(degreesToRad(180-_articular_pos.z)-PI/2) + L1;

    return cartesian_pos;
}

RHATypes::Point3 RobotRHA::inverseKinematics (RHATypes::Point3 _cartesian_pos) {
    RHATypes::Point3 articular_pos;
    float xa = _cartesian_pos.x - LA;
    float za = _cartesian_pos.z - L1 - LB;

    articular_pos.y = 180-radToDegrees(PI/2 - (acos(sqrt(xa*xa + za*za)/(2*L2)) + atan2(za,xa)));
    articular_pos.z = 180-radToDegrees(PI - (acos(sqrt(xa*xa + za*za)/(2*L2)) + atan2(za,xa)) - acos((L2*L2+L2*L2-sqrt(xa*xa + za*za)*sqrt(xa*xa + za*za))/(2*L2*L2)) + PI/2);

    return articular_pos;
}

void RobotRHA::updateInfo() {
    // articular_position_, cartesian_position_;
    // articular_position_.x = <- Unknown for now
    articular_position_.y = joint_handler_.joint_[1].getPosition();
    articular_position_.z = joint_handler_.joint_[2].getPosition();
    cartesian_position_ = forwardKinematics(articular_position_);

}

bool RobotRHA::checkError() {
    return joint_handler_.isError();
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


void RobotRHA::initPynterface() {
    Serial_PYNTERFACE.begin(PYNTERFACE_BAUDRATE);
    send_pynterface_data_.setTimer(SEND_PYNTERFACE_DELAY);
    send_pynterface_data_.activateTimer();
}

void RobotRHA::handleWithPynterface(){
    handleRobot();
    sendPackage();
    getPackage();
}


bool RobotRHA::sendPackage() {
    uint8_t i;
    uint8_t buffer[PYNTERFACE_MSG_LENGTH];
    uint8_t checksum;
    if(send_pynterface_data_.checkContinue()) {
        if (isError()) {
            checksum = 0;
            buffer[0] = 0xFF;
            buffer[1] = 0xFF;
            buffer[2] = 4;  // length without header
            buffer[3] = PynterfaceConstants::ERROR;
            buffer[4] = 0;
            buffer[4] += joint_handler_.joint_[0].servo_.checkSecurity()?0x01:0;
            buffer[4] += joint_handler_.joint_[1].servo_.checkSecurity()?0x02:0;
            buffer[4] += joint_handler_.joint_[2].servo_.checkSecurity()?0x04:0;
            buffer[5] = 0;
            buffer[5] = joint_handler_.joint_[0].checkSecurity()?0x01:0;
            buffer[5] = joint_handler_.joint_[1].checkSecurity()?0x02:0;
            buffer[5] = joint_handler_.joint_[2].checkSecurity()?0x04:0;
            checksum += buffer[2];
            checksum += buffer[3];
            checksum += buffer[4];
            checksum += buffer[5];
            for (i = 0; i < buffer[2]+2; i++) {
                Serial_PYNTERFACE.write(buffer[i]);
            }
        }
        else {
            checksum = 0;
            buffer[0] = 0xFF;
            buffer[1] = 0xFF;                                                           // Header is not included in checksum
            buffer[2] = 18;                                                             checksum += buffer[2];  // Length without header
            buffer[3] = PynterfaceConstants::UPDATE_INFO;                               checksum += buffer[3];  // Package pourpose
            buffer[4] = uint8_t(articular_position_.x);                                 checksum += buffer[4];  // Package:
            buffer[5] = joint_handler_.joint_[0].servo_.getSpeedWithDir() & 0x00FF;     checksum += buffer[5];
            buffer[6] = joint_handler_.joint_[0].servo_.getSpeedWithDir() >> 8;         checksum += buffer[6];
            buffer[7] = joint_handler_.joint_[0].servo_.getTorqueWithDir() & 0x00FF;    checksum += buffer[7];
            buffer[8] = joint_handler_.joint_[0].servo_.getTorqueWithDir() >> 8;        checksum += buffer[8];
            buffer[9] = uint8_t(articular_position_.y);                                 checksum += buffer[9];
            buffer[10] = joint_handler_.joint_[1].servo_.getSpeedWithDir() & 0x00FF;    checksum += buffer[10];
            buffer[11] = joint_handler_.joint_[1].servo_.getSpeedWithDir() >> 8;        checksum += buffer[11];
            buffer[12] = joint_handler_.joint_[1].servo_.getTorqueWithDir() & 0x00FF;   checksum += buffer[12];
            buffer[13] = joint_handler_.joint_[1].servo_.getTorqueWithDir() >> 8;       checksum += buffer[13];
            buffer[14] = uint8_t(articular_position_.z);                                checksum += buffer[14];
            buffer[15] = joint_handler_.joint_[2].servo_.getSpeedWithDir() & 0x00FF;    checksum += buffer[15];
            buffer[16] = joint_handler_.joint_[2].servo_.getSpeedWithDir() >> 8;        checksum += buffer[16];
            buffer[17] = joint_handler_.joint_[2].servo_.getTorqueWithDir() & 0x00FF;   checksum += buffer[17];
            buffer[18] = joint_handler_.joint_[2].servo_.getTorqueWithDir() >> 8;       checksum += buffer[18];
            buffer[19] = ~(checksum);
            for (i = 0; i < buffer[2]+2; i++) {
                Serial_PYNTERFACE.write(buffer[i]);
            }
        }
        return true;
        send_pynterface_data_.activateTimer();
    }
    else return false;

}
void RobotRHA::getPackage() {
    uint8_t i;
    uint8_t buffer[PYNTERFACE_MSG_LENGTH+1];
    uint8_t length = 0;
    uint8_t checksum;


    // Reads command sent by pynterface
    uint8_t readCount = 0;
    // Serial read returns -1 if theres no data to read in buffer
    if (Serial_PYNTERFACE.read() == 0xFF && Serial_PYNTERFACE.read() == 0xFF) {
        // Now a msg can be read. Header was ok
        length = Serial_PYNTERFACE.read(); checksum += length;
        readCount = Serial_PYNTERFACE.readBytes(buffer, length-1);
        // Check if checksum is correct
        for (i = 0; i < length-1; i++) {
            checksum += buffer[i];
        }
        // If its correct it can handle the information
        // Checksum is the last byte in package
        if (checksum == buffer[length] && readCount == length-1) {
            if (buffer[0] == PynterfaceConstants::ARTICULAR_GOAL) {
                pynterface_goal_.x = buffer[1];
                pynterface_goal_.y = buffer[2];
                pynterface_goal_.z = buffer[3];
                goToArticularPos(pynterface_goal_);
            }
        }
    }
}

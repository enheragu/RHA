/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   31_Oct_2017
 * @Project: RHA
 * @Filename: robot_rha.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 23_Nov_2017
 */



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
    DebugSerialRRHALn("initJointHandler: begin arm calibration");
    //calibration();

    // Make sure the arm stais in position
    joint_handler_.updateJointInfo();
    updateInfo();
    RHATypes::Point3 init_pos;
    init_pos.x = joint_handler_.joint_[0].getPosition();
    init_pos.y = joint_handler_.joint_[1].getPosition();
    init_pos.z = joint_handler_.joint_[2].getPosition();
    goToArticularPos(init_pos);

    /*pynterface_goal_.x = 0;
    pynterface_goal_.y = 120;
    pynterface_goal_.z = 60;
    goToArticularPos(pynterface_goal_);*/

    DebugSerialRRHALn("initJointHandler: end of function");
}


void RobotRHA::handleRobot(uint8_t _calibration) {
    DebugSerialRRHALn("handleRobot: begin of function");
    updateInfo();
    robot_error_ = checkError();
    if (robot_error_ && !_calibration){
        DebugSerialRRHALn("handleRobot: error detected. Not going through control loops");
        return;
    } else {
        joint_handler_.controlLoopTorque(_calibration);
        joint_handler_.controlLoopSpeed(_calibration);
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

    cartesian_pos.x = cos(degreesToRad(_articular_pos.x))*(LA + L2*cos(degreesToRad(_articular_pos.y - PI/2)) + L2*cos(degreesToRad(PI/2 - _articular_pos.z)));

    cartesian_pos.y = sin(degreesToRad(_articular_pos.x))*(LA  + L2*cos(degreesToRad(_articular_pos.y - PI/2)) + L2*cos(degreesToRad(PI/2 - _articular_pos.z)));

    cartesian_pos.z = LB + L2*sin(degreesToRad(_articular_pos.y - PI/2)) - L2*sin(degreesToRad(PI/2 - _articular_pos.z)) + L1;

    return cartesian_pos;
}

RHATypes::Point3 RobotRHA::inverseKinematics (RHATypes::Point3 _cartesian_pos) {
    RHATypes::Point3 articular_pos;

    float x_prima = sqrt(_cartesian_pos.x*_cartesian_pos.x + _cartesian_pos.y*_cartesian_pos.y) - LA    ;
    float z_prima = _cartesian_pos.z - LB - L1;

    articular_pos.x = radToDegrees(atan2(_cartesian_pos.y,x_prima));
    articular_pos.y =  radToDegrees((acos(sqrt(x_prima*x_prima + z_prima*z_prima)/(2*L2)) + atan2(sqrt(x_prima*x_prima + z_prima*z_prima),x_prima)) + PI/2);
    articular_pos.z =  radToDegrees(PI/2 - PI - (acos(sqrt(x_prima*x_prima + z_prima*z_prima)/(2*L2)) + atan2(sqrt(x_prima*x_prima + z_prima*z_prima),x_prima)) - acos((L2*L2+L3*L3-pow(sqrt(x_prima*x_prima + z_prima*z_prima),2))/(2*L2*L3)));

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
 * @brief Calibrates all joints in case rope is not tense
 * @method RobotRHA::calibration
 */
void RobotRHA::calibration() {
    DebugSerialRRHALn("calibration: begin of function");
    uint8_t calibration_ok = 0;
    uint8_t j = 0;

    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        // stop all servos
        setSpeedToServos(20,joint_handler_.joint_[i].servo_.getID());
    }
    while (j <= ERROR_MOVING_MARGIN) {
        if(joint_handler_.controlLoopTorque(true))
            j++;
    }
    while (calibration_ok < NUM_JOINT) {
        calibration_ok = 0;
        for (uint8_t i = 0; i < NUM_JOINT; i++) {
            // check security returns true if theres no problem
            joint_handler_.joint_[i].checkSecurity()?setSpeedToServos(0,joint_handler_.joint_[i].servo_.getID()):setSpeedToServos(20,joint_handler_.joint_[i].servo_.getID());
            DebugSerialRRHALn4("calibration: calibrate joint ", i, " is:  ",joint_handler_.joint_[i].checkSecurity()?"completed":"on going");
            if(joint_handler_.joint_[i].checkSecurity())
                calibration_ok++;
        }
        //handleRobot(true);
    }
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        // stop all servos
        joint_handler_.sendSetWheelSpeedAll(1,CW);
    }
    DebugSerialRRHALn("calibration: end of function");
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
    RHATypes::SpeedGoal speed_goal(_servo_id, abs(_speed), (_speed>=0)?CW:CCW);
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
    DebugSerialRRHALn("handleWithPynterface: begin of function");
    handleRobot(true);
    sendPackage();
    getPackage();
}


bool RobotRHA::sendPackage() {
    DebugSerialRRHALn("sendPackage: begin of function");
    uint8_t i;
    uint8_t checksum;
    resetBuffer(buffer_);
    if(send_pynterface_data_.checkContinue()) {
        if (isError()) {
            DebugSerialRRHALn("sendPackage: build error msg");
            checksum = 0;
            buffer_[0] = 0xFF;
            buffer_[1] = 0xFF;
            buffer_[2] = 5;  // length without header
            buffer_[3] = PynterfaceConstants::ERROR;
            buffer_[4] = 0;
            buffer_[4] += joint_handler_.joint_[0].servo_.checkSecurity()?0x01:0;
            buffer_[4] += joint_handler_.joint_[1].servo_.checkSecurity()?0x02:0;
            buffer_[4] += joint_handler_.joint_[2].servo_.checkSecurity()?0x04:0;
            buffer_[5] = 0;
            buffer_[5] = joint_handler_.joint_[0].checkSecurity()?0x01:0;
            buffer_[5] = joint_handler_.joint_[1].checkSecurity()?0x02:0;
            buffer_[5] = joint_handler_.joint_[2].checkSecurity()?0x04:0;
            checksum += buffer_[2];
            checksum += buffer_[3];
            checksum += buffer_[4];
            checksum += buffer_[5];
            buffer_[6] = ~(checksum);
            for (i = 0; i < buffer_[2]+2; i++) {
                Serial_PYNTERFACE.write(buffer_[i]);
            }
        }
        else {
            DebugSerialRRHALn("sendPackage: build update msg");
            checksum = 0;
            buffer_[0] = 0xFF;
            buffer_[1] = 0xFF;                                                           // Header is not included in checksum
            buffer_[2] = 21;                                                             checksum += buffer_[2];  // Length without header
            buffer_[3] = PynterfaceConstants::UPDATE_INFO;                               checksum += buffer_[3];  // Package pourpose
            //DebugSerialRRHALn("sendPackage: load info for q1 joint");
            buffer_[4] = uint8_t(articular_position_.x);                                 checksum += buffer_[4];  // Package:
            buffer_[5] = joint_handler_.joint_[0].servo_.getSpeedWithDir() & 0x00FF;     checksum += buffer_[5];
            buffer_[6] = joint_handler_.joint_[0].servo_.getSpeedWithDir() >> 8;         checksum += buffer_[6];
            buffer_[7] = joint_handler_.joint_[0].servo_.getTorqueWithDir() & 0x00FF;    checksum += buffer_[7];
            buffer_[8] = joint_handler_.joint_[0].servo_.getTorqueWithDir() >> 8;        checksum += buffer_[8];
            //DebugSerialRRHALn("sendPackage: load info for q2 joint");
            buffer_[9] = uint8_t(articular_position_.y);                                 checksum += buffer_[9];
            //DebugSerialRRHALn("sendPackage: y set");
            buffer_[10] = joint_handler_.joint_[1].servo_.getSpeedWithDir() & 0x00FF;    checksum += buffer_[10];
            //DebugSerialRRHALn("sendPackage: speed set");
            buffer_[11] = joint_handler_.joint_[1].servo_.getSpeedWithDir() >> 8;        checksum += buffer_[11];
            //DebugSerialRRHALn("sendPackage: speed2 set");
            buffer_[12] = joint_handler_.joint_[1].servo_.getTorqueWithDir() & 0x00FF;   checksum += buffer_[12];
            //DebugSerialRRHALn("sendPackage: torque set");
            buffer_[13] = joint_handler_.joint_[1].servo_.getTorqueWithDir() >> 8;       checksum += buffer_[13];
            //DebugSerialRRHALn("sendPackage: torque2 set");
            //DebugSerialRRHALn("sendPackage: load info for q3 joint");
            buffer_[14] = uint8_t(articular_position_.z);                                checksum += buffer_[14];
            //DebugSerialRRHALn("sendPackage: z set");
            buffer_[15] = joint_handler_.joint_[2].servo_.getSpeedWithDir() & 0x00FF;    checksum += buffer_[15];
            //DebugSerialRRHALn("sendPackage: speed set");
            buffer_[16] = joint_handler_.joint_[2].servo_.getSpeedWithDir() >> 8;        checksum += buffer_[16];
            //DebugSerialRRHALn("sendPackage: speed2 set");
            buffer_[17] = joint_handler_.joint_[2].servo_.getTorqueWithDir() & 0x00FF;   checksum += buffer_[17];
            //DebugSerialRRHALn("sendPackage: torque set");
            buffer_[18] = joint_handler_.joint_[2].servo_.getTorqueWithDir() >> 8;       checksum += buffer_[18];
            //DebugSerialRRHALn("sendPackage: torque2 set");
            //DebugSerialRRHALn("sendPackage: inverse checksum");
            buffer_[19] = joint_handler_.joint_[0].getPositionTarget();       checksum += buffer_[19];
            buffer_[20] = joint_handler_.joint_[1].getPositionTarget();       checksum += buffer_[20];
            buffer_[21] = joint_handler_.joint_[2].getPositionTarget();       checksum += buffer_[21];
            buffer_[22] = ~(checksum);
            //DebugSerialRRHALn("sendPackage: prepared to send msg");
            for (i = 0; i < buffer_[2]+2; i++) {
                Serial_PYNTERFACE.write(buffer_[i]);
            }
        }
        DebugSerialRRHALn("sendPackage: end of function, package sent");
        send_pynterface_data_.activateTimer();
        return true;
    }
    else {
        DebugSerialRRHALn("sendPackage: end of function, package not sent");
        return false;
    }
}
void RobotRHA::getPackage() {
    DebugSerialRRHALn("getPackage: begin of function");
    uint8_t i;
    uint8_t length = 0;
    uint8_t checksum;
    resetBuffer(buffer_);

    // Reads command sent by pynterface
    uint8_t readCount = 0;
    // Serial read returns -1 if theres no data to read in buffer_
    if (Serial_PYNTERFACE.read() == 0xFF && Serial_PYNTERFACE.read() == 0xFF) {
        // Now a msg can be read. Header was ok
        length = Serial_PYNTERFACE.read(); checksum += length;
        readCount = Serial_PYNTERFACE.readBytes(buffer_, length-1);
        // Check if checksum is correct
        for (i = 0; i < length-1; i++) {
            checksum += buffer_[i];
        }
        checksum = ~checksum;
        // If its correct it can handle the information
        // Checksum is the last byte in package
        if (readCount == length-1) { //} && checksum == buffer_[length]) {
            if (buffer_[0] == PynterfaceConstants::ARTICULAR_GOAL) {
                pynterface_goal_.x = buffer_[1];
                pynterface_goal_.y = buffer_[2];
                pynterface_goal_.z = buffer_[3];
                goToArticularPos(pynterface_goal_);
            }
        }
    }
}

void RobotRHA::resetBuffer(uint8_t _buffer[]) {
    for (uint8_t i = 0; i < BUFFER_LEN; i++){
        _buffer[i] = 0;
    }
}

#include "servo_rha.h"
#include "Cytron_G15Shield.h"
#include "Arduino.h"


/** @brief ServoRHA cunstructor of ServoRHA class. Gets servo ID and calibrates the minimum torque. (See calibrateToruqe function)
  * @param servo_id: servo id controlled by this object
  */
ServoRHA::ServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):
    Cytron_G15Shield(servo_id, rxpin, txpin, ctrlpin) {
}

/** @brief initServo handles the inicialization of all ServoRHA internal parameters
  */
void ServoRHA::initServo() {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");

    Cytron_G15Shield::begin(19200);
    delay(DELAY1);
    calibrateTorque();

    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;
    acceleration_angle_ = ACCELERATION_ANGLE;
    flag_moving_ = false;
    current_pose_ = 0; goal_pose_encoder_ = 0; init_pose_ = 0; encoder_current_ = 0;
    acceleration_slope_ = (static_cast<float>100 - static_cast<float>0) / static_cast<float>acceleration_angle_;
    flag_accelerating_ = false;
    flag_decelerating_ = false;
    flag_first_time_accel_decel_ = true;

    returnPacketSet(RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions

    DebugSerialSRHALn("initServo: end of inicialitation function");
}


/************************************************************************
 *       Interface functions to get/set important data from servo       *
 ************************************************************************/

/** @brief angleRead function is used to read current position of the servo
  * @return Returns position in degrees (0 to 360)
  */
uint16_t ServoRHA::angleRead() {
    DebugSerialSRHALn("angleRead: begin of function.");
    uint8_t data[10];
    getPos(data);  // get the current position from servo1
    uint16_t pos = data[0];
    pos = pos | ((data[1]) << 8);
    DebugSerialSRHALn("angleRead: end of function.");
    return ConvertPosToAngle(pos);
}

/** @brief speedRead function is used to read current speed of the servo
  * @return Returns speed
  */
uint16_t ServoRHA::speedRead() {
    DebugSerialSRHALn("speedRead: begin of function.");
    uint8_t data[10];
    getSpeed(data);  // get the current position from servo1
    uint16_t speed = data[0];
    speed |=  word(data[1]) << 8;
    DebugSerialSRHALn("speedRead: end of function.");
    return ConvertPosToAngle(speed);
}

/** @brief returnPacketSet function sets the package return level of servo (error information for each command sent)
  * @param option: RETURN_PACKET_ALL -> servo returns packet for all commands sent; RETURN_PACKET_NONE -> servo never retunrs state packet; RETURN_PACKET_READ_INSTRUCTIONS -> servo answer packet state when a READ command is sent (to read position, temperature, etc)
  * @retunr Returns error code for this action
  */
uint16_t ServoRHA::returnPacketSet(uint8_t option) {
    DebugSerialSRHALn("returnPacketSet: begin of function.");
    uint8_t TxBuff[2];

    TxBuff[0] = STATUS_RETURN_LEVEL;         // Control Starting Address
    TxBuff[1] = option;             // ON = 1, OFF = 0

     // write the packet, return the error code
    DebugSerialSRHALn("returnPacketSet: end of function.");
    return(sendPacket(_servo_id, iREG_WRITE, TxBuff, 2));
}

/*****************************************
 *       Packet handling functions       *
 *****************************************/

/** @brief addToPacket adds this servo to a buffer with his own information (id, goal, etc). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param buffer: is the buffer in which the information will be added (by reference)
  * @param position: is the position from which it writes the new info (by reference)
  * @param instruction: is the instruction that is being sended in this packet
  * @param goal: the goal to send. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param goal_len: length of the goal (uint8_ts)
  * @param num_servo: how many servos had been added to this packet
  */
void ServoRHA::addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo) {
    DebugSerialSRHALn("addToPacket: begin of function");
    buffer[position] = _servo_id;
    position++;
    num_servo++;
    for (int i = 0; i < goal_len; i++) {
        buffer[position] = goal[i];
        position++;
    }
    DebugSerialSRHALn("addToPacket: end of function");
}

/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param buffer: is the buffer in which the information will be added (by reference)
  * @param data: is the data that have been completed by each servo (by reference)
  * @param data_len: is the length of data
  * @param instruction: is the instruction to send
  * @param num_servo: how many servos had been added to this packet
  * @return Returns number of uint8_ts that contain usefull info (how many have been written)
  */
uint8_t ServoRHA::wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo) {
    ebugSerialSRHALn("wrapPacket: begin of function");
    int i = 0;
    char checksum = 0;    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)

    buffer[0] = 0xFF;               // 0xFF not included in checksum
    buffer[1] = 0xFF;
    buffer[2] = ALL_SERVO;      checksum +=  buffer[2];
    buffer[3] = data_len+4;     checksum +=  buffer[3];
    buffer[4] = iSYNC_WRITE;    checksum +=  buffer[4];
    buffer[5] = instruction;    checksum +=  buffer[5];
     buffer[6] = num_servo;     checksum +=  buffer[6];
    for (i = 0; i < data_len; i++) {
        buffer[i+7] = data[i];
        checksum +=  buffer[i+7];
    }
    buffer[i+7] = ~checksum;                 // Checksum with Bit Inversion
    DebugSerialSRHALn("wrapPacket: end of function");
    return buffer[3] + 4;
}


/***************************************
 *       Calibratation functions       *
 ***************************************/

/** @brief calibrateTorque function gets the minimum torque in which the servo starts moving for CW and CCW direcion. (See calibrateTorqueDir function)
  */
void ServoRHA::calibrateTorque() {
    DebugSerialSRHALn("calibrateTorque: begin of function");
    calibrateTorqueDir(min_torque_cw_, CW);
    calibrateTorqueDir(min_torque_ccw_, CCW);
    DebugSerialSRHALn2("calibrateTorque: min_torque_cw_ is now: ", min_torque_cw_);
    DebugSerialSRHALn2("calibrateTorque: min_torque_ccw_ is now: ", min_torque_ccw_);
    DebugSerialSRHALn("calibrateTorque: enf of function");
}

/** @brief calibrateTorque function gets the minimum torque in which the servo starts moving in a set direction
  * @param min_torque: reference to the min torque value to change (it can be cor CW or CCW direction)
  * @param direction: CW or CCW, note that is has to be consistent with the min_torque variable
  */
void ServoRHA::calibrateTorqueDir(uint16_t &min_torque, uint16_t direction) {
    DebugSerialSRHALn2("calibrateTorqueDir: begin of function. Direction: CW = 1; CCW = 0", direction);

    setWheelMode();
    delay(DELAY1);

    for (min_torque = 0; min_torque < 1023; min_torque+= TORQUE_CALIBRATION_INTERVAL) {
        DebugSerialSRHALn2("calibrateTorqueDir: try with torque: ", min_torque);
        Cytron_G15Shield::setWheelSpeed(min_torque, direction, iWRITE_DATA);
        delay(DELAY1);
        if (isMoving()) break;
    }

    exitWheelMode();
    DebugSerialSRHALn2("calibrateTorqueDir: end of function. Direction: CW = 1; CCW = 0", direction);
}



/***************************************************************
 *       Overwritten functions from G15 original library       *
 ***************************************************************/

/** @brief SetWheelSpeed sets wheel speed according to margins saved in calibration.
  * @param speed: speed value (in %, 0 to 100) -> 0 means stop and from 1 to 100 means moving
  * @param cw_ccw: direction in which the servo will move
  * @return Error status in uint16_t. If return is non-zero, error occurred. (See returnPacketOnOFF function)
  */
uint16_t ServoRHA::setWheelSpeed(uint16_t speed, uint8_t cw_ccw) {
    DebugSerialSRHALn4("setWheelSpeed: begin of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    uint16_t g15_speed = -1;
    if (speed == 0) g15_speed = 1;
    else if (speed > 0 && cw_ccw == CW) g15_speed = (uint16_t)(map(speed, 1, 100, min_torque_cw_+TORQUE_CALIBRATION_INTERVAL, max_torque_cw_));
    else if (speed > 0 && cw_ccw == CCW) g15_speed = (uint16_t)(map (speed, 1, 100, min_torque_ccw_+TORQUE_CALIBRATION_INTERVAL, max_torque_ccw_));

    DebugSerialSRHALn2("setWheelSpeed: speed calculated to send to servo is: ", g15_speed)
    DebugSerialSRHALn4("setWheelSpeed: end of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    return Cytron_G15Shield::setWheelSpeed(g15_speed, cw_ccw, iWRITE_DATA);
}


/**********************************************************
 *       Action functions from G15 original library       *
 **********************************************************/


/** @brief setGoalEncoder sets goal for the servo.
  * @param goal_rotation: goal in rounds (1.35 means 1.35+360 degrees)
  * @param cw_ccw: direction in which the servo will move
  */
void ServoRHA::setGoalEncoder(float goal_rotation, uint8_t cw_ccw) {
    DebugSerialSRHALn("setGoalEncoder: begin of function");
    goal_pose_encoder_ = goal_rotation;
    goal_direction_ = cw_ccw;
    DebugSerialSRHALn("setGoalEncoder: end of function");
}

/** @brief doNext function checks and does pendin actions for this servo.
  */
void ServoRHA::doNext() {
    DebugSerialSRHALn("doNext: begin of function");
     // if theres goal or servo is moving:
    if (goal_pose_encoder_ !=  0 || flag_moving_ == true) {
        encoderModeRotation();
    }  // End of if (goal_pose_encoder_ !=  0)
    DebugSerialSRHALn("doNext: end of function");
}  // End of doNext function


/** @brief encoderModeRotation handles encoder rotation
  */
void ServoRHA::encoderModeRotation() {
    DebugSerialSRHALn("encoderModeRotation: begin of function");
    current_pose_ = angleRead();
    uint16_t speed = 1;

    // To start moving, changes flag to moving now and sets minimum speed
    if (flag_moving_ == false) {
        DebugSerialSRHALn("encoderModeRotation: init of movement");
        init_pose_ = angleRead();
        setWheelSpeed(speed, goal_direction_);
        flag_moving_ = true;
        encoder_current_ = 0;
        flag_accelerating_ = true;  // acceleration can get active
        flag_first_time_accel_decel_ = true;  // first time that accel is entered
    }

    int goal_whole_number = 360*static_cast<int>goal_pose_encoder_;  // degrees of full rounds
    int goal_decimal_part = ((goal_pose_encoder_ - goal_whole_number)*100)*100/360;  // degrees of decimal part of rounds
    uint8_t angle_travelled = current_pose_ - init_pose_ + 360*encoder_current_;
    uint8_t angle_left = abs(goal_whole_number + goal_decimal_part - (current_pose_ + encoder_current_*360));

    // Acceleration if its in acceleration interval
    if (angle_travelled < acceleration_angle_ && flag_accelerating_ == true) {
        DebugSerialSRHALn("encoderModeRotation: servo accelerating");
        accelerate(speed);
    }  // End of acceleration

    // Once acceleration interval is over and it still has to move more than 360 degrees it goes counting as an enconder
    else if (angle_travelled > acceleration_angle_ && angle_left > 360) {
        DebugSerialSRHALn("encoderModeRotation: servo at full speed");
        if (current_pose_ < (init_pose_ + ENCODER_MARGIN)  && current_pose_ > (init_pose_ - ENCODER_MARGIN) && encoder_flag_ == 0) {
            encoder_current_++;
            encoder_flag_ = 1;
        }
        if (current_pose_ > (init_pose_ + ENCODER_MARGIN)  || current_pose_ < (init_pose_ - ENCODER_MARGIN)) encoder_flag_ = 0;
        flag_decelerating_ = true;  // deceleration can get active
        flag_first_time_accel_decel_ = true;  // first time that deccel is entered
    }  // End of encoder counting (less than 360 degrees left)

    // Deceleration once it reach deceleration interval
    else if (angle_left < acceleration_angle_ && flag_decelerating_ == true) {
        DebugSerialSRHALn("encoderModeRotation: servo decelerating");
        decelerate(speed, angle_left);
    }  // End of deceleration
    DebugSerialSRHALn("encoderModeRotation: end of function");
}

/** @brief accelerate handles the acceleration process. Its separated from encoderModeRotation for testing purposes. (See encoderModeRotation() function)
  */
void ServoRHA::accelerate(uint16_t &speed) {
    // In case its not the first time whe enter deccelerate() and current angle = init angleangle (360º) acceleration is shut down for nex iterations
    if (speed == 100 || (flag_first_time_accel_decel_ != true && compareAngles(current_pose_, init_pose_, MARGIN_ANGLE_COMPARISON) == EQUAL && flag_accelerating_ == true)) flag_accelerating_ = false;
    // When te angles are different for first time flag_first_time_accel_decel_ is set to false
    if (compareAngles(current_pose_, init_pose_, MARGIN_ANGLE_COMPARISON) !=  EQUAL) flag_first_time_accel_decel_ = false;
    else {
        DebugSerialSRHALn("accelerate: begin of function");
        speed = abs(current_pose_ - init_pose_)*acceleration_slope_;
        setWheelSpeed(speed, goal_direction_);
        DebugSerialSRHALn("accelerate: end of function");
    }
}

/** @brief decelerate handles the deceleration process. Its separated from encoderModeRotation for testing purposes. (See encoderModeRotation() function)
  */
void ServoRHA::decelerate(uint16_t &speed, uint16_t angle_left) {
    // In case its not the first time whe enter deccelerate() and angle_left is cero deceleration is shut down for nex iterations
    if (speed == 0 || (flag_first_time_accel_decel_ != true && compareAngles(angle_left, 0, MARGIN_ANGLE_COMPARISON) == EQUAL && flag_decelerating_ == true)) {
        flag_moving_ = false;
        goal_pose_encoder_ = 0;
        init_pose_ = 0;
        encoder_current_ = 0;
        flag_decelerating_ = false;
    }  // End of goal reached
    // When te angles are different for first time flag_first_time_accel_decel_ is set to false
    if (compareAngles(current_pose_, init_pose_, MARGIN_ANGLE_COMPARISON) !=  EQUAL) flag_first_time_accel_decel_ = false;

    else {
        DebugSerialSRHALn("decelerate: begin of function");
        speed = angle_left*(-acceleration_slope_);  // (current_pose_ - init_pose_)*(-acceleration_slope_);
        setWheelSpeed(speed, goal_direction_);
        DebugSerialSRHALn("decelerate: end of function");
    }
}

/** @brief isMoving to know whether the servo is moving or not based on servo real speed
  * @retunr Returns bool (true if its moving, or false if not)
  */
bool ServoRHA::isMoving() {
    DebugSerialSRHALn("isMoving: begin of function");
    uint16_t speed = speedRead();
    DebugSerialSRHALn("isMoving: end of function");
    if (speed == 0) return false;
    else return true;
}

/***************************************
 *       Complementary functions       *
 ***************************************/

/** @brief compareAngles function compares two angles with a margin set.
  * @param angle1: angle to compare
  * @param angle2: angle used in the comparison
  * @param angle_margin: margin in which the angle1 will be considered to be equal to angle2 [angle2-angle_margin, angle2+angle_margin]
  * @return Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareAngles: begin of function. Angle 1: ", angle1, ". Angle 2: ", angle2);
    if (angle1 < angle2-angle_margin) return LESS_THAN;
    else if (angle1 > angle2+angle_margin) return GREATER_THAN;
    else return EQUAL;
}

/** @brief compareSpeed function compares two speeds with a margin set.
  * @param speed1: speed to compare
  * @param speed2: speed used in the comparison
  * @param speed_margin: margin in which the speed will be considered to be equal to speed2 [speed2-speed_margin, speed2+speed_margin]
  * @return Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareSpeed: begin of function. Speed 1: ", speed1, ". Speed 2: ", speed2);
    if (speed1 < speed2-speed_margin) return LESS_THAN;
    else if (speed1 > speed2+speed_margin) return GREATER_THAN;
    else return EQUAL;
}

/**
 * @file
 * @brief Implements ServoRHA functions defined in servo_rha.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
 */

#include "servo_rha.h"
// #include "cytron_g15_servo.h"
#include "Arduino.h"

// using namespace ServoRHAConstants;

/** @brief Constructor of ServoRHA class.
  * @param {uint8_t} servo_id servo id controlled by this object
  */
ServoRHA::ServoRHA(uint8_t servo_id) {
    servo_id_ = servo_id;
    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;
    min_torque_cw_ = MIN_TORQUE_CW;
    min_torque_ccw_ = MIN_TORQUE_CCW;
}


/** @brief Handles the inicialization of all ServoRHA internal parameters when default constructor is used
  * @param {uint8_t} servo_id servo id controlled by this object
  */
void ServoRHA::init(uint8_t servo_id) {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");
    // calibrateTorque();

    servo_id_ = servo_id;

    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;

    // returnPacketSet(ServoRHAConstants::RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions

    DebugSerialSRHALn("initServo: end of inicialitation function");
}

/************************************************************************
 *       Interface functions to get/set important data from servo       *
 ************************************************************************/

/** @brief Asks the servo for all the information to be updated by class servo.
  *
  * Reads from register PRESENT_POSITION_L (0x24) to MOVING (0x2E).
  * Position are bits 10 to 0 from register 0x24 and 0x25
  * Speed are bits 9 to 0 from register 0x26 and 0x27, 10th bit is direction
  * Load are bits 9 to 0 from register 0x28 and 0x29, 10th bit is direction
  * Voltage is in register 0x2a
  * Temperature is in register 0x2B
  * Action registered (pending from activation) flag is in register 0x2C
  * Moving flag is in register 0x2E
  */
void ServoRHA::updateInfo(uint8_t *data) {
    position_ = *data; data++;  // acces data[0]
    position_ |= (*data << 8); data++;  // acces data[1]

    speed_ = *data; data++;   // acces data[2]
    speed_ |= (*data << 8); data++;  // acces data[3]
    speed_dir_  = ((speed_ >> 9) & 0x10);  // 10th byte is direction
    // bytes from 9 to 0 are speed value:
    speed_ = speed_ & ~0x0400;

    load_ = *data; data++;  // acces data[4]
    load_ |= (*data << 8); data++;  // acces data[5]
    load_dir_  = ((load_ >> 9) & 0x10);  // 10th byte is direction
    // bytes from 9 to 0 are load value:
    load_ = load_ & ~0x0400;

    voltage_ = *data; data++;  // acces data[6]  // NOTE: ¿should be divided by 10?

    temperature_ = *data; data++;  // acces data[7]

    // registered_ = data[8];

    // is_moving_ = data[10];
}


/**********************************************************
 *       Action functions from G15 original library       *
 **********************************************************/

/**
 * [regulatorServo description]
 * @method regulatorServo
 * @param  {float} error target speed - actual speed
 * @return  {uint16_t} Returns torque value to send to the servo
 */
uint16_t ServoRHA::regulatorServo(float error) {
    return KP*error;
}


/*****************************************
 *       Packet handling functions       *
 *****************************************/

void addUpadteInfoToPacket(uint8_t * &buffer) {
     uint8_t data[2];
     data[0] = JointHandlerConstants::PRESENT_POSITION_L;
     data[1] = 0x08;  // Wants to read 11 bytes from PRESENT_POSITION_L
     addToSinglePacket(buffer, data, 2);
}

 /** @brief returnPacketSet function sets the package return level of servo (error information for each command sent)
   * @param {uint8_t buffer*}
   * @param {uint8_t} option RETURN_PACKET_ALL -> servo returns packet for all commands sent; RETURN_PACKET_NONE -> servo never retunrs state packet; RETURN_PACKET_READ_INSTRUCTIONS -> servo answer packet state when a READ command is sent (to read position, temperature, etc)
   * @see addToPacket()
   */
void addReturnOptionToPacket(uint8_t * &buffer, uint8_t option) {
     DebugSerialSRHALn("returnPacketSet: begin of function.");

     uint8_t option[2];
     option[0] = JointHandlerConstants::STATUS_RETURN_LEVEL;         // Control Starting Address
     option[1] = option;             // ON = 1, OFF = 0

     addToSinglePacket(buffer, option, 2);

     DebugSerialSRHALn("returnPacketSet: end of function.");
     return;
}

/** @brief Adds this servo torque command to a buffer with his own information. This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @see addToPacket()
  */
void ServoRHA::addTorqueToPacket(uint8_t * &buffer, uint8_t &bytes_write) {
    DebugSerialSRHALn("addToPacket: begin of function");
    addToPacket(buffer, goal_torque_, 2);
    DebugSerialSRHALn("addToPacket: end of function");
}


/** @brief addToPacket adds this servo to a buffer with his own information (id, goal, etc). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} packet small packet to add. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param {uint8_t} packet_len length of the small packet to add (uint8_ts)
  */
void ServoRHA::addToSyncPacket(uint8_t * &buffer, uint8_t *packet, uint8_t packet_len, uint8_t &bytes_write) {
    DebugSerialSRHALn("addToPacket: begin of function");
    *buffer = servo_id_; buffer++;
    for (int i = 0; i < packet_len; i++) {
        *buffer = packet[i];  buffer++;
    }
    bytes_write += packet_len + 1;  // Packet len + servo ID
    DebugSerialSRHALn("addToPacket: end of function");
}

/** @brief addToPacket adds this servo to a buffer with his own information (id, goal, etc). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} packet small packet to add. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param {uint8_t} packet_len length of the small packet to add (uint8_ts)
  */
void ServoRHA::addToSinglePacket(uint8_t * &buffer, uint8_t *packet, uint8_t packet_len) {
    DebugSerialSRHALn("addToPacket: begin of function");
    *buffer = servo_id_; buffer++;
    *buffer = packet_len; buffer++;
    *buffer = *packet; buffer++; packet++;  // instruction
    for (int i = 0; i < packet_len; i++) {
        *buffer = packet; buffer++; packet++;
    }

    DebugSerialSRHALn("addToPacket: end of function");
}


/***************************************
 *       Calibratation functions       *
 ***************************************/

/** @brief calibrateTorque function gets the minimum torque in which the servo starts moving for CW and CCW direcion.
  * @see calibrateTorqueDir(param1, param2)
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
  * @param min_torque reference to the min torque value to change (it can be cor CW or CCW direction)
  * @param direction CW or CCW, note that is has to be consistent with the min_torque variable
  */
void ServoRHA::calibrateTorqueDir(uint16_t &min_torque, uint16_t direction) {
    DebugSerialSRHALn2("calibrateTorqueDir: begin of function. Direction: CW = 1; CCW = 0", direction);

    setWheelMode();
    delay(DELAY1);

    for (min_torque = 0; min_torque < 1023; min_torque+= TORQUE_CALIBRATION_INTERVAL) {
        DebugSerialSRHALn2("calibrateTorqueDir: try with torque: ", min_torque);
        Cytron_G15_Servo::setWheelSpeed(min_torque, direction, iWRITE_DATA);
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
  * @param {uint16_t} speed speed value (in %, 0 to 100) -> 0 means stop and from 1 to 100 means moving
  * @param {uint8_t} cw_ccw direction in which the servo will move
  * @return {uint16_t} Error status. If return is non-zero, error occurred. (depends on retunrPacket option)
  * @see returnPacketOnOFF()
  */
uint16_t ServoRHA::setWheelSpeedPercent(uint16_t speed, uint8_t cw_ccw) {
    DebugSerialSRHALn4("setWheelSpeed: begin of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    uint16_t g15_speed = -1;
    if (speed == 0) g15_speed = 1;
    else if (speed > 0 && cw_ccw == CW) g15_speed = (uint16_t)(map(speed, 1, 100, min_torque_cw_+TORQUE_CALIBRATION_INTERVAL, max_torque_cw_));
    else if (speed > 0 && cw_ccw == CCW) g15_speed = (uint16_t)(map (speed, 1, 100, min_torque_ccw_+TORQUE_CALIBRATION_INTERVAL, max_torque_ccw_));

    DebugSerialSRHALn2("setWheelSpeed: speed calculated to send to servo is: ", g15_speed)
    DebugSerialSRHALn4("setWheelSpeed: end of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    return Cytron_G15_Servo::setWheelSpeed(g15_speed, cw_ccw, iWRITE_DATA);
    // return Cytron_G15_Servo::setWheelSpeed(speed, cw_ccw, iWRITE_DATA);
}


/***************************************
 *       Complementary functions       *
 ***************************************/

/** @brief compareAngles function compares two angles with a margin set.
  * @param {uint16_t} angle1 angle to compare
  * @param {uint16_t} angle2 angle used in the comparison
  * @param {uint8_t} angle_margin margin in which the angle1 will be considered to be equal to angle2 [angle2-angle_margin, angle2+angle_margin]
  * @return {uint8_t} Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareAngles: begin of function. Angle 1: ", angle1, ". Angle 2: ", angle2);
    if (angle1 < angle2-angle_margin) return LESS_THAN;
    else if (angle1 > angle2+angle_margin) return GREATER_THAN;
    else return EQUAL;
}

/** @brief compareSpeed function compares two speeds with a margin set.
  * @param {uint16_t} speed1 speed to compare
  * @param {uint16_t} speed2 speed used in the comparison
  * @param {uint8_t} speed_margin margin in which the speed will be considered to be equal to speed2 [speed2-speed_margin, speed2+speed_margin]
  * @return {uint8_t} Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareSpeed: begin of function. Speed 1: ", speed1, ". Speed 2: ", speed2);
    if (speed1 < speed2-speed_margin) return LESS_THAN;
    else if (speed1 > speed2+speed_margin) return GREATER_THAN;
    else return EQUAL;
}

/**
 * @file
 * @brief Implements ServoRHA functions defined in servo_rha.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 20-Sep-2017
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
}


/** @brief Handles the inicialization of all ServoRHA internal parameters when default constructor is used
  * @param {uint8_t} servo_id servo id controlled by this object
  */
void ServoRHA::init(uint8_t servo_id) {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");
    // calibrateTorque();

    servo_id_ = servo_id;
    DebugSerialSRHALn2("initServo: id is now: ", servo_id_);

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
void ServoRHA::updateInfo(uint8_t *data, uint16_t error) {
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

    error_comunication_ = error;
}


/**********************************************************
 *       Action functions from G15 original library       *
 **********************************************************/

void ServoRHA::calculateTorque(float error) {
    // float error = (float)target_speed - (float)speed_;
    float torque = regulatorServo(error);
    uint8_t direction = 0;
    if (torque > 0) direction = CW;
    else if (torque < 0) direction = CCW;
    torque = abs(torque);
    if (torque > 1023) torque = 1023;  // compensate saturation of servos

    goal_torque_ = torque;

    if (direction == CW) {
        goal_torque_ = goal_torque_ | 0x0400;
    }
}

/**
 * [regulatorServo description]
 * @method regulatorServo
 * @param  {float} error target speed - actual speed
 * @return  {uint16_t} Returns torque value to send to the servo
 */
uint16_t ServoRHA::regulatorServo(float error) {
    return kp_*error;
}


/*****************************************
 *       Packet handling functions       *
 *****************************************/

void ServoRHA::addUpadteInfoToPacket(uint8_t *buffer) {
     uint8_t data[2];
     data[0] = ServoRHAConstants::PRESENT_POSITION_L;
     data[1] = 0x08;  // Wants to read 11 bytes from PRESENT_POSITION_L
     addToPacket(buffer, data, 2);
}

 /** @brief returnPacketSet function sets the package return level of servo (error information for each command sent)
   * @param {uint8_t buffer*}
   * @param {uint8_t} option RETURN_PACKET_ALL -> servo returns packet for all commands sent; RETURN_PACKET_NONE -> servo never retunrs state packet; RETURN_PACKET_READ_INSTRUCTIONS -> servo answer packet state when a READ command is sent (to read position, temperature, etc)
   * @see addToPacket()
   */
void ServoRHA::addReturnOptionToPacket(uint8_t *buffer, uint8_t option) {
     DebugSerialSRHALn("returnPacketSet: begin of function.");

     uint8_t optionPacket[2];
     optionPacket[0] = ServoRHAConstants::STATUS_RETURN_LEVEL;         // Control Starting Address
     optionPacket[1] = option;             // ON = 1, OFF = 0

     addToPacket(buffer, optionPacket, 2);

     DebugSerialSRHALn("returnPacketSet: end of function.");
     return;
}

/** @brief Adds this servo torque command to a buffer with his own information. This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @see addToPacket()
  */
bool ServoRHA::addTorqueToPacket(uint8_t *buffer) {
    DebugSerialSRHALn("addToPacket: begin of function");
    uint8_t txBuffer[3];
    txBuffer[0] = ServoRHAConstants::MOVING_SPEED_L;
    txBuffer[1] = goal_torque_ & 0x00FF;  // Speed bottom 8 bits
    txBuffer[2] = goal_torque_ >> 8;  // Speed top 8 bits
    addToPacket(buffer, txBuffer, 3);
    DebugSerialSRHALn("addToPacket: end of function");
    return true;
}

void ServoRHA::setTorqueOnOfToPacket(uint8_t *buffer, uint8_t onOff) {
    uint8_t txBuffer[2];
    txBuffer[0] = ServoRHAConstants::TORQUE_ENABLE;
    txBuffer[1] = onOff;  // ON = 1, OFF = 0

    addToPacket(buffer, txBuffer, 2);
}

void ServoRHA::setWheelModeToPacket(uint8_t *buffer) {
    wheelModeToPacket(buffer, 0, 0);  // Enable wheel mode

}

void ServoRHA::exitWheelModeToPacket(uint8_t *buffer) {
    wheelModeToPacket(buffer, 0, 1087);  // Reset to default angle limit
}

void ServoRHA::wheelModeToPacket(uint8_t *buffer, uint16_t CW_angle, uint16_t CCW_angle) {
    uint8_t txBuffer[5];

    txBuffer[0] = ServoRHAConstants::CW_ANGLE_LIMIT_L;
    txBuffer[1] = CW_angle & 0x00FF;   // CW limit bottom 8 bits
    txBuffer[2] = CW_angle >> 8;       // CW limit top 8 bits
    txBuffer[3] = CCW_angle & 0x00FF;  // CCW limit bottom 8 bits
    txBuffer[4] = CCW_angle >> 8;      // CCW limit top 8 bits

    addToPacket(buffer, txBuffer, 5);
}


/** @brief addToPacket adds this servo to a buffer with his own information (id, goal, etc). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} packet small packet to add. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param {uint8_t} packet_len length of the small packet to add (uint8_ts)
  */
void ServoRHA::addToPacket(uint8_t *buffer, uint8_t *packet, uint8_t packet_len) {
    DebugSerialSRHALn("addToPacket: begin of function");
    buffer[0] = servo_id_;
    buffer[1] = packet_len;
    buffer[2] = packet[0];
    for (int i = 0; i < packet_len; i++) {
        buffer[3 + i] = packet[1 + i];
    }

    DebugSerialSRHALn("addToPacket: end of function");
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
    if (angle1 < angle2-angle_margin) return ServoRHAConstants::LESS_THAN;
    else if (angle1 > angle2+angle_margin) return ServoRHAConstants::GREATER_THAN;
    else return ServoRHAConstants::EQUAL;
}

/** @brief compareSpeed function compares two speeds with a margin set.
  * @param {uint16_t} speed1 speed to compare
  * @param {uint16_t} speed2 speed used in the comparison
  * @param {uint8_t} speed_margin margin in which the speed will be considered to be equal to speed2 [speed2-speed_margin, speed2+speed_margin]
  * @return {uint8_t} Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareSpeed: begin of function. Speed 1: ", speed1, ". Speed 2: ", speed2);
    if (speed1 < speed2-speed_margin) return ServoRHAConstants::LESS_THAN;
    else if (speed1 > speed2+speed_margin) return ServoRHAConstants::GREATER_THAN;
    else return ServoRHAConstants::EQUAL;
}

/**
 * @file
 * @brief Implements ServoRHA functions defined in servo_rha.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.cpp
 * @Last modified by:   quique
 * @Last modified time: 30-Sep-2017
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
void ServoRHA::init(uint8_t _servo_id) {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");
    // calibrateTorque();

    servo_id_ = _servo_id;
    DebugSerialSRHALn2("initServo: id is now: ", servo_id_);

    speed_regulator_.setKRegulator(KP,KI,KD);

    DebugSerialSRHALn("initServo: end of inicialitation function");
}

/************************************************************************
 *       Interface functions to get/set important data from servo       *
 ************************************************************************/

/** @brief Asks the servo for all the information to be updated by class servo.
  * @method ServoRHA::updateInfo
  * @param {uint8_t *} data  Array containing all the data
  * @param {uint16_t} error Error in comunication
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
void ServoRHA::updateInfo(uint8_t *_data, uint16_t _error) {
    DebugSerialSRHALn("updateInfo: begin of function");

    position_ = _data[0];  // acces data[0]
    position_ |= (_data[1] << 8);  // acces data[1]

    uint16_t speed;
    speed = _data[2];    // acces data[2]
    speed |= (_data[3] << 8);   // acces data[3]
    speed_dir_  = ((speed & 0x0400) >> 10);  // 10th byte is direction -> 0000010000000000 binary is 400 in hex
    // bytes from 9 to 0 are speed value:
    speed = speed & ~0x0400;  // Speed is in rad/s
    speed_ = (float(speed) * 112.83) / 1023;  // Speed in register is from 0 to 1023, this operation is to transform it into RPM (is taken from servo docs)

/*  Load is load performed by the servo, for now this information is no being used so its not asked to the servo to save time
    load_ = _data[4];  // acces data[4]
    load_ |= (_data[5] << 8);   // acces data[5]
    load_dir_  = ((load_ & 0x0400) >> 10);  // 10th byte is direction
    // bytes from 9 to 0 are load value:
    load_ = load_ & ~0x0400;

    voltage_ = _data[6];   // acces data[6]  // NOTE: ¿should be divided by 10?

    temperature_ = _data[7] ; // acces data[7]
*/
    error_comunication_ = _error;

    DebugSerialSRHALn("updateInfo: Information updated");
    DebuSerialRHALnPrintServoStatus(position_, speed_, speed_dir_, load_, load_dir_, voltage_, temperature_, error_comunication_);
}

/**
 * @brief Sets speed goal to achieve with speed slope
 * @param {uint16_t} speed_target speed to achieve
 * @param {uint16_t} speed_slope slope from actual speed to speed_target (acceleration)
 * @param {uint16_t} direction_target move CW or CCW
 */

uint8_t ServoRHA::setSpeedGoal(RHATypes::SpeedGoal _goal) {
    DebugSerialJRHALn("setSpeedGoal: seting speed goal");
    DebugSerialSRHALn2("setSpeedGoal: servo id now is: ", this->getID());
    DebugSerialSRHALn2("setSpeedGoal: goal intended for id: ", _goal.servo_id);
    if (servo_id_ == _goal.servo_id) {
        speed_slope_ = _goal.speed_slope;
        speed_target_ = _goal.speed;
        direction_target_ = _goal.direction;
        time_last_ = millis();
        DebugSerialSRHALn2("setSpeedGoal: Speed set to: ", speed_target_);
        DebugSerialSRHALn2("setSpeedGoal: Speed slope set to: ", speed_slope_);
        return true;
    } else return false;
}

/**
 * @brief Calculates error to send to servo regulator
 */
void ServoRHA::speedError() {
    DebugSerialSRHALn("speedError: begin of function");
    uint16_t speed = 0;
    // TODO: if (speed_slope_ != 0) {
    //     speed = (float)speed_ + (float)(millis() - time_last_) * speed_slope_;
    //     if (speed > speed_target_) speed = (float)speed_target_;
    //     time_last_ = millis();
    // } else
    speed = speed_target_;
    int8_t sign = 1;
    // TODO: if (direction_target_ != speed_dir_) sign = -1;
    error_ = (sign*((float)speed - (float)speed_));
    derror_ = ( error_ - last_error_ ) / (millis() - time_last_error_);
    ierror_ = error_ * (millis() - time_last_error_);
    last_error_ = error_;
    time_last_error_ = millis();
    DebugSerialSRHALn2("speedError: target speed is: ", speed_target_);
    DebugSerialSRHALn2("speedError: error is: ", error_);
}

/**
 * @brief calculates torque from speed error using regulator. Params are by default 0, it is only used with params for testing pourposes
 * @method ServoRHA::calculateTorque
 * @param  error      speed error
 * @param  derror     derivative of speed error
 * @param  ierror     integral of speed error
 */
void ServoRHA::calculateTorque(float _error, float _derror, float _ierror) {
    DebugSerialSRHALn("calculateTorque: begin of function");
    // float error = (float)target_speed - (float)speed_;
    float torque;
    if (_error != 0)
        error_ = _error; derror_ = _derror; ierror_ = _ierror;

    torque = speed_regulator_.regulator(error_, derror_, ierror_);
    // uint8_t direction = 0;
    // error < 0 causes torque < 0 which causes change in direction of movement
    // TODO: if (torque < 0 ) {
    //     if (speed_dir_ == CCW) direction = CW;
    //     else if (speed_dir_ == CW) direction = CCW;
    // }
    if(error_ < 0) torque = 0;
    // else {
    //     torque = abs(torque);
    // }
    else if (torque > 0) torque = torque + TORQUE_OFFSET + TORQUE_PREALIMENTATION*float(speed_target_);
    if (torque > MAX_TORQUE_VALUE) torque = MAX_TORQUE_VALUE;  // compensate saturation of servos

    DebugSerialSRHALn2("calculateTorque: torque calculated is: ", goal_torque_);
    goal_torque_ = uint16_t(torque);
}


/*****************************************
 *       Packet handling functions       *
 *****************************************/

/**
 * @brief adds to buffer packet with the uptade info command
 * @method ServoRHA::addUpadteInfoToPacket
 * @param {uint8_t*} buffer array in which add the information
 */
void ServoRHA::addUpadteInfoToPacket(uint8_t *_buffer) {
     uint8_t data[2];
     data[0] = ServoRHAConstants::PRESENT_POSITION_L;
     data[1] = 0x04;  // Wants to read 4 bytes from PRESENT_POSITION_L
     addToPacket(_buffer, data, 2);
}

 /** @brief Saves in buffer the package return level of servo (error information for each command sent)
   * @method ServoRHA::addReturnOptionToPacket
   * @param {uint8_t*} buffer array in which add the information
   * @param {uint8_t} option RETURN_PACKET_ALL -> servo returns packet for all commands sent; RETURN_PACKET_NONE -> servo never retunrs state packet; RETURN_PACKET_READ_INSTRUCTIONS -> servo answer packet state when a READ command is sent (to read position, temperature, etc)
   * @see addToPacket()
   */
void ServoRHA::addReturnOptionToPacket(uint8_t *_buffer, uint8_t _option) {
     DebugSerialSRHALn("returnPacketSet: begin of function.");

     uint8_t optionPacket[2];
     optionPacket[0] = ServoRHAConstants::STATUS_RETURN_LEVEL;         // Control Starting Address
     optionPacket[1] = _option;             // ON = 1, OFF = 0

     addToPacket(_buffer, optionPacket, 2);

     DebugSerialSRHALn("returnPacketSet: end of function.");
     return;
}

/** @brief Adds this servo torque command to a buffer with his own information. This function is used to send just one packet for all servos instead of each sending their respective information
  * @method ServoRHA::addTorqueToPacket
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @see addToPacket()
  */
bool ServoRHA::addTorqueToPacket(uint8_t *_buffer) {
    DebugSerialSRHALn("addToPacket: begin of function");
    uint8_t txBuffer[3];

    goal_torque_ = goal_torque_ & 0x03FF;  // Eliminate bits which are non speed
    if (direction_target_ == CW) {
        goal_torque_ = goal_torque_ | 0x0400;
    }

    txBuffer[0] = ServoRHAConstants::MOVING_SPEED_L;
    txBuffer[1] = goal_torque_ & 0x00FF;  // Speed bottom 8 bits
    txBuffer[2] = goal_torque_ >> 8;  // Speed top 8 bits
    addToPacket(_buffer, txBuffer, 3);
    DebugSerialSRHALn("addToPacket: end of function");
    return true;
}

/**
 * @brief Adds to buffer information about the torque option (on or off)
 * @method ServoRHA::setTorqueOnOfToPacket
 * @param buffer array in which add the information
 * @param onOff  ON = 1; OFF = 0;
 */
void ServoRHA::setTorqueOnOfToPacket(uint8_t *_buffer, uint8_t _onOff) {
    DebugSerialSRHALn("setTorqueOnOfToPacket: begin of function");
    uint8_t txBuffer[2];
    txBuffer[0] = ServoRHAConstants::TORQUE_ENABLE;
    txBuffer[1] = _onOff;  // ON = 1, OFF = 0

    addToPacket(_buffer, txBuffer, 2);
}

/**
 * @brief Adds to buffer information to set wheel mode for servo
 * @method ServoRHA::setWheelModeToPacket
 * @param buffer array in which add the information
 * @see exitWheelModeToPacket()
 * @see wheelModeToPacket()
 */
void ServoRHA::setWheelModeToPacket(uint8_t *_buffer) {
    DebugSerialSRHALn("setWheelModeToPacket: begin of function");
    wheelModeToPacket(_buffer, 0, 0);  // Enable wheel mode

}

/**
 * @brief Adds to buffer information to exit wheel mode for servo
 * @method ServoRHA::exitWheelModeToPacket
 * @param buffer array in which add the information
 * @see setWheelModeToPacket()
 * @see wheelModeToPacket()
 */
void ServoRHA::exitWheelModeToPacket(uint8_t *_buffer) {
    DebugSerialSRHALn("exitWheelModeToPacket: begin of function");
    wheelModeToPacket(_buffer, 0, 1087);  // Reset to default angle limit
}

/**
 * @brief Adds to buffer information to set/exit wheel mode for servo. Common function for exit and set functions.
 * @param buffer array in which add the information
 * @param CW_angle cw angle limit
 * @param CCW_angle ccw angle limit
 * @see setWheelModeToPacket()
 * @see exitWheelModeToPacket()
 */
void ServoRHA::wheelModeToPacket(uint8_t *_buffer, uint16_t _CW_angle, uint16_t _CCW_angle) {
    uint8_t txBuffer[5];

    txBuffer[0] = ServoRHAConstants::CW_ANGLE_LIMIT_L;
    txBuffer[1] = _CW_angle & 0x00FF;   // CW limit bottom 8 bits
    txBuffer[2] = _CW_angle >> 8;       // CW limit top 8 bits
    txBuffer[3] = _CCW_angle & 0x00FF;  // CCW limit bottom 8 bits
    txBuffer[4] = _CCW_angle >> 8;      // CCW limit top 8 bits

    addToPacket(_buffer, txBuffer, 5);
}

/**
 * @brief Arranges data array to ping action
 * @method ServoRHA::pingToPacket
 * @param  buffer  Array in which to store the data
 */
void ServoRHA::pingToPacket(uint8_t *_buffer) {
    uint8_t txBuffer[1] = {0};
    addToPacket(_buffer, txBuffer, 0);
}

/**
 * @brief Arranges data packet with torque limit
 * @method ServoRHA::setTorqueLimitToPacket
 * @param  buffer  Array in which to store the data
 * @param  torque_limit  Torque limit to set
 */
void ServoRHA::setTorqueLimitToPacket(uint8_t *_buffer, uint16_t _torque_limit) {
    DebugSerialSRHALn("setTorqueLimitToPacket: begin of function");
    uint8_t txBuffer[3];

    txBuffer[0] = ServoRHAConstants::TORQUE_LIMIT_L;
    txBuffer[1] = _torque_limit & 0x00FF;  // Torque limit bottom 8 bits
    txBuffer[2] = _torque_limit >> 8;  // Torque limit top 8 bits
    addToPacket(_buffer, txBuffer, 3);

}

/**
 * @brief Makes packet with speed goal with set direction
 * @method ServoRHA::setWheelSpeedToPacket
 * @param  buffer  Array in which to store the data
 * @param  speed   Speed to set
 * @param  direction   Direction in which servo will move
 */
void ServoRHA::setWheelSpeedToPacket(uint8_t *_buffer, uint16_t _speed, uint8_t _direction) {
    DebugSerialSRHALn("setWheelSpeedToPacket: begin of function");
    DebugSerialSRHALn2("setWheelSpeedToPacket: speed to set is: ", _speed);
    uint8_t txBuffer[3];

    _speed = _speed & 0x03FF;  // Eliminate bits which are non speed
    if (_direction == CW) {
        _speed = _speed | 0x0400;
    }

    txBuffer[0] = ServoRHAConstants::MOVING_SPEED_L;
    txBuffer[1] = _speed & 0x00FF;  // Torque limit bottom 8 bits
    txBuffer[2] = _speed >> 8;  // Torque limit top 8 bits
    addToPacket(_buffer, txBuffer, 3);
}


/** @brief addToPacket adds this servo to a buffer with his own information (id, goal, etc). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} packet small packet to add. Note that it can be speed, torque, position... It can be a combination (go to an X position with an Y speed) (by reference)
  * @param {uint8_t} packet_len length of the small packet to add (uint8_ts)
  */
void ServoRHA::addToPacket(uint8_t *_buffer, uint8_t *_packet, uint8_t _packet_len) {
    DebugSerialSRHALn("addToPacket: begin of function");
    _buffer[0] = servo_id_;
    _buffer[1] = _packet_len;
    for (int i = 0; i < _packet_len; i++) {
      _buffer[2 + i] = _packet[i];


    DebugSerialSRHALn("addToPacket: end of function");
    }
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
uint8_t compareAngles(uint16_t _angle1, uint16_t _angle2, uint8_t _angle_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareAngles: begin of function. Angle 1: ", _angle1, ". Angle 2: ", _angle2);
    if (_angle1 < _angle2-_angle_margin) return ServoRHAConstants::LESS_THAN;
    else if (_angle1 > _angle2+_angle_margin) return ServoRHAConstants::GREATER_THAN;
    else return ServoRHAConstants::EQUAL;
}

/** @brief compareSpeed function compares two speeds with a margin set.
  * @param {uint16_t} speed1 speed to compare
  * @param {uint16_t} speed2 speed used in the comparison
  * @param {uint8_t} speed_margin margin in which the speed will be considered to be equal to speed2 [speed2-speed_margin, speed2+speed_margin]
  * @return {uint8_t} Returns enumeration defined in servo_rha.h -> LESS_THAN, GREATER_THAN or EQUAL
  */
uint8_t compareSpeed(uint16_t _speed1, uint16_t _speed2, uint8_t _speed_margin) {
    DebugSerialSRHALn4("ServoRHA.cpp::compareSpeed: begin of function. Speed 1: ", _speed1, ". Speed 2: ", _speed2);
    if (_speed1 < _speed2-_speed_margin) return ServoRHAConstants::LESS_THAN;
    else if (_speed1 > _speed2+_speed_margin) return ServoRHAConstants::GREATER_THAN;
    else return ServoRHAConstants::EQUAL;
}

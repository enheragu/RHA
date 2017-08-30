#include "servo_rha.h"
#include "cytron_g15_servo.h"
#include "Arduino.h"


/** @brief ServoRHA cunstructor of ServoRHA class. Gets servo ID and calibrates the minimum torque.
  * @param servo_id: servo id controlled by this object
  * @see calibrateToruqe()
  */
ServoRHA::ServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):
    Cytron_G15_Servo(servo_id, rxpin, txpin, ctrlpin) {
}


/** @brief initServo handles the inicialization of all ServoRHA internal parameters
  */
void ServoRHA::init(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate) {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");
    Cytron_G15_Servo::init(servo_id, rxpin, txpin, ctrlpin, baudrate);
    delay(DELAY1);
    //calibrateTorque();

    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;

    returnPacketSet(RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions

    DebugSerialSRHALn("initServo: end of inicialitation function");
}

void ServoRHA::init() {
    DebugSerialSRHALn("initServo: begin of inicialitationfunction");

    Cytron_G15_Servo::begin(19200);
    delay(DELAY1);
    //calibrateTorque();

    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;

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
    pos = pos & 0000000011111111; // only the last 8 bits contain pos info
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
    //speed = speed & 0000000111111111; // only the last 9 bits contain speed info
    return speed;
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


void ServoRHA::updateInfo(){
    speed_ = speedRead();
    position_ = angleRead();
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
    return(sendPacket(servo_id_, iREG_WRITE, TxBuff, 2));
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
    buffer[position] = servo_id_;
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
    DebugSerialSRHALn("wrapPacket: begin of function");
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

/** @brief calibrateTorque function gets the minimum torque in which the servo starts moving for CW and CCW direcion.
  * @see calibrateToruqe()
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
  * @param speed: speed value (in %, 0 to 100) -> 0 means stop and from 1 to 100 means moving
  * @param cw_ccw: direction in which the servo will move
  * @return Error status in uint16_t. If return is non-zero, error occurred.
  * @see returnPacketOnOFF()
  */
uint16_t ServoRHA::setWheelSpeed(uint16_t speed, uint8_t cw_ccw) {
    DebugSerialSRHALn4("setWheelSpeed: begin of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    uint16_t g15_speed = -1;
    if (speed == 0) g15_speed = 1;
    else if (speed > 0 && cw_ccw == CW) g15_speed = (uint16_t)(map(speed, 1, 100, min_torque_cw_+TORQUE_CALIBRATION_INTERVAL, max_torque_cw_));
    else if (speed > 0 && cw_ccw == CCW) g15_speed = (uint16_t)(map (speed, 1, 100, min_torque_ccw_+TORQUE_CALIBRATION_INTERVAL, max_torque_ccw_));

    DebugSerialSRHALn2("setWheelSpeed: speed calculated to send to servo is: ", g15_speed)
    DebugSerialSRHALn4("setWheelSpeed: end of function. Speed set to ", speed, ". Direction: CW = 1; CCW = 0n", cw_ccw);
    return Cytron_G15_Servo::setWheelSpeed(g15_speed, cw_ccw, iWRITE_DATA);
}


/**********************************************************
 *       Action functions from G15 original library       *
 **********************************************************/

uint16_t regulator(uint16_t error) {
    return KP*error;
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

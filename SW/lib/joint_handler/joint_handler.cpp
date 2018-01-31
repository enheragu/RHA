/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 31_Oct_2017
 */

// #include "HardwareSerial.h"
#include "joint_handler.h"

boolean hardwareSerial_ = false;
SoftwareSerial* G15Serial_;

/**
  * @brief Constructor with timer info
  * @method JointHandler::JointHandler
  */
JointHandler::JointHandler(uint64_t _timer) {
    torque_control_loop_timer_.setTimer(_timer);
}

/**
 * @brief Initialices timer from control_loop for speed in ServoRHA
 * @method JointHandler::init
 * @param  timer              time in ms for control loop
 */
void JointHandler::setTorqueControlTimer(uint64_t _timer) {
    DebugSerialJHLn("setTimer: begin of function");
    speed_control_loop_timer_.setTimer(_timer);
    speed_control_loop_timer_.activateTimer();
}

/**
 * @brief Initialices timer from control_loop for position in JointRHA
 * @method JointHandler::init
 * @param  timer              time in ms for control loop
 */
void JointHandler::setSpeedControlTimer(uint64_t _timer) {
    DebugSerialJHLn("setTimer: begin of function");
    torque_control_loop_timer_.setTimer(_timer);
    torque_control_loop_timer_.activateTimer();
}

/**
 * @brief Initialices joints with ID, up direction and potentiometer pin. Sets wheel mode to all servo
 * @method JointHandler::initJoints
 */
void JointHandler::initJoints() {
    DebugSerialJHLn("initJoints: begin of function");
    joint_[0].init(1, CW, 0);
    joint_[1].init(2, CW);  // does not have potentiometer, not realimented
    joint_[2].init(3, CW);  // , 1);

    joint_[0].setPotRelation(0.69);  // (float)(29/42));
    joint_[2].setPotRelation(1);

    RHATypes::Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_WRITE_DELAY);
    eeprom_timer.activateTimer();

    DebugSerialJHLn("initJoints: set wheel mode for all servos");
    sendSetWheelModeAll();

    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();

    DebugSerialJHLn("initJoints: return packet all for all servos");
    setReturnPacketOption(RETURN_PACKET_ALL);  // RETURN_PACKET_ALL  RETURN_PACKET_READ_INSTRUCTIONS

    eeprom_timer.checkWait();
}

/**
  * @brief controlLoopTorque() function handles control loop for servo speed (output of regulator is torque for servo)
  */
void JointHandler::controlLoopTorque() {
    DebugSerialJHLn("controlLoop: begin of function");
    if (torque_control_loop_timer_.checkContinue()) {
        // Firt all info have to be updated
        DebugSerialJHLn("controlLoop: Updating joint info");
        updateJointInfo();

        // Ask each joint torque to update speed
        DebugSerialJHLn("controlLoop: Updating joint error torque");
        updateJointErrorTorque();

        DebugSerialJHLn("controlLoop: send new torque to servos");
        // TODO(eeha): sendJointTorques();  // <- uses sync packet which aparently does not work well
        sendSetWheelSpeedAll();  // <- uses async/single packet to each servo
        // Send buffer

        torque_control_loop_timer_.activateTimer();
    }
}

/**
 * @brief controlLoopSpeed() function handles control loop for joint position (output of regulator is speed for ServoRHA)
 * @method JointHandler::updateJointInfo
 */
 void JointHandler::controlLoopSpeed() {
     DebugSerialJHLn("controlLoopSpeed: begin of function");
     if (speed_control_loop_timer_.checkContinue()) {
         // Info is supposed to be updated faster in the other control loop. It is not updated here to avoid losing time comunicating
         //DebugSerialJHLn("controlLoopSpeed: Updating joint info");
         //updateJointInfo();

         DebugSerialJHLn("controlLoopSpeed: Updating joint error pos");
         updateJointErrorSpeed();

         DebugSerialJHLn("controlLoopSpeed: send new speed to ServoRHA");
         sendSpeedGoalAll();

         speed_control_loop_timer_.activateTimer();
     }
 }

/**
 * @brief Updates internal information from all joint.
 * @method JointHandler::updateJointInfo
 * @see JointRHA::updateJointInfo
 */
void JointHandler::updateJointInfo() {
    DebugSerialJHLn("updateJointInfo: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.addUpadteInfoToPacket(buffer);
        uint8_t txBuffer[BUFFER_LEN];
        warpSinglePacket(iREAD_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
        joint_[i].updateInfo(txBuffer, error);
    }
}

/**
 * @brief Updates all joints error to update torque goal
 * @method JointHandler::updateJointErrorTorque
 */
void JointHandler::updateJointErrorTorque() {
    DebugSerialJHLn("updateJointErrorTorque: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.speedError();
        joint_[i].servo_.calculateTorque();
    }
}

/**
 * @brief handles the packet construction and sent for all joint torques (torque goal saved in each servo)
 * @method JointHandler::sendJointTorques
 */
void JointHandler::sendJointTorques() {
    DebugSerialJHLn("sendJointTorques: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        if (joint_[i].servo_.addTorqueToPacket(buffer)) {
            num_servo++;
            num_bytes = addToSyncPacket(buffer_to_send, buffer, num_bytes);
        }
    }
    uint8_t txBuffer[BUFFER_LEN];
    warpSyncPacket(buffer_to_send, buffer[2], txBuffer, num_bytes, num_servo);
    sendPacket(txBuffer);
}

/**
 * @brief Updates all joint position error to update speed goal sor ServoRHA
 * @method JointHandler::updateJointErrorSpeed
 */
void JointHandler::updateJointErrorSpeed() {
    DebugSerialJHLn("updateJointErrorSpeed: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].posError();
        joint_[i].calculateSpeed();
    }
}

/**
 * @brief Sends speed goal calculated to ServoRHA
 * @method JointHandler::sendSpeedGoalAll
 */
void JointHandler::sendSpeedGoalAll() {
    DebugSerialJHLn("sendSpeedGoalAll: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].updateServoSpeedGoal();
    }
}


/**
 * @brief Sets speed goal to a given joint (based on servo ID in goal)
 * @method JointHandler::setSpeedGoal
 * @param  goal Goal containing speed, speed_slope and ID
 */
void JointHandler::setSpeedGoal(RHATypes::SpeedGoal _goal) {
    DebugSerialJHLn("setSpeedGoal: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        if (joint_[i].servo_.setSpeedGoal(_goal)) return;
    }
}

/**
 * @brief Sets retunr packet option for all joints
 * @method JointHandler::setReturnPacketOption
 * @param  _option                             [description]
 */
void JointHandler::setReturnPacketOption(uint8_t _option) {
    DebugSerialJHLn("setReturnPacketOption: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.addReturnOptionToPacket(buffer, _option);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

/**
 * @brief Sets wheel mode for all servo
 * @method JointHandler::sendSetWheelModeAll
 */
void JointHandler::sendSetWheelModeAll() {
    DebugSerialJHLn("sendSetWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];

    RHATypes::Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_WRITE_DELAY);
    eeprom_timer.activateTimer();

    DebugSerialJHLn("sendSetWheelModeAll: set wheel mode");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        // First set angle limit
        joint_[i].servo_.setWheelModeToPacket(buffer);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();
    DebugSerialJHLn("sendSetWheelModeAll: set torque ON");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        // Then set torque ON
        joint_[i].servo_.setTorqueOnOfToPacket(buffer, ON);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
    eeprom_timer.checkWait();
}

/**
 * @brief Exit wheel mode for all servo
 * @method JointHandler::sendExitWheelModeAll
 */
void JointHandler::sendExitWheelModeAll() {
    DebugSerialJHLn("sendExitWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];

    RHATypes::Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_WRITE_DELAY);
    eeprom_timer.activateTimer();

    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.exitWheelModeToPacket(buffer);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();
    DebugSerialJHLn("sendExitWheelModeAll: set torque OFF");
    /*for (uint8_t i = 0; i < NUM_JOINT; i++) {
        // Then set torque ON
        joint_[i].servo_.setTorqueOnOfToPacket(buffer, OFF);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }*/
    eeprom_timer.checkWait();
}

// NOTE: testing purposes
/**
 * @brief Sets torque limit to all servo
 * @method JointHandler::sendSetTorqueLimitAll
 * @param  torque_limit    torque limit to set
 */
void JointHandler::sendSetTorqueLimitAll(uint16_t _torque_limit) {
    DebugSerialJHLn("sendSetTorqueLimitAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.setTorqueLimitToPacket(buffer, _torque_limit);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
/**
 * @brief Interface to send speed (in wheel mode this means torque) to servos. Params are by default 0, if this is the case servos work in a closed control loop, if not they use the speed set.
 * @method JointHandler::sendSetWheelSpeedAll
 * @param  speed        speed/torque to set
 * @param  direction    direction CW (clocwise) or CCW (counterclockwise)
 */
void JointHandler::sendSetWheelSpeedAll(uint16_t _speed, uint8_t _direction) {
    DebugSerialJHLn("sendSetWheelSpeedAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        if (_speed != 0) {
            DebugSerialJHLn2("sendSetWheelSpeedAll: speed to set is: ", _speed);
            DebugSerialJHLn2("sendSetWheelSpeedAll: in direction: (CW = 1; CCW = 0)", _direction);
            joint_[i].servo_.setWheelSpeedToPacket(buffer, _speed, _direction);
        } else {
            joint_[i].servo_.setWheelSpeedToPacket(buffer, joint_[i].servo_.getGoalTorque(), joint_[i].servo_.getDirectionTarget());  // getGoalTorque if used with realimentation, getSpeedTarget if not
        }
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
/**
 * @brief checks if can conect with all servo
 * @method JointHandler::checkConectionAll
 * @return retunrs true if the conection with all servo was succesfull. Returns false if failed with any of them
 */
bool JointHandler::checkConectionAll() {
    DebugSerialJHLn("checkConectionAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.pingToPacket(buffer);
        warpSinglePacket(iPING, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
        if (error != 0) return false;
    }
    return true;
}


/****************************************************
 *   Comunication and packet construction methods   *
 ****************************************************/

/**
 * @brief Adds data to common buffer. Intended to put commands from all servo into one packet
 * @method JointHandler::addToSyncPacket
 * @param  {uint8_t *} buffer array to write all the info
 * @param  {uint8_t *} data   contains data to copy
 * @param  {uint8_t} num_bytes   number of bytes tha have been written
 * @return        returns length copied in bytes
 */
uint8_t JointHandler::addToSyncPacket(uint8_t *_buffer, uint8_t *_data, uint8_t _num_bytes) {
    DebugSerialJHLn("addToSyncPacket: begin of function");
    _buffer[ _num_bytes] = _data[0];  _num_bytes++;
    for (int i = 0; i < _data[1]-1; i++) {  // skips lenght and register addres
        _buffer[ _num_bytes] = _data[i+3];   _num_bytes++;  // component 2 and 3 are packet_len and instruction
    }
    return  _num_bytes;  // Packet len + servo ID
}



/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @method JointHandler::warpSyncPacket
  * @param {uint8_t *} buffer is the data that have been completed by each servo (by reference)
  * @param {uint8_t} adress Direction of servo register in which to write/read...
  * @param {uint8_t *} txBuffer data warped and ready to send
  * @param {uint8_t} num_bytes is the length of data
  * @param {uint8_t} num_servo how many servos had been added to this packet
  * @see JointHandler::sendPacket()
  */
void JointHandler::warpSyncPacket(uint8_t *_buffer, uint8_t _adress, uint8_t *_txBuffer, uint8_t _num_bytes, uint8_t _num_servo) {
    DebugSerialJHLn("warpSyncPacket: begin of function");

    uint8_t i;
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)

    _txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    _txBuffer[1] = 0xFF;
    _txBuffer[2] = ALL_SERVO;      checksum +=  _txBuffer[2];
    _txBuffer[3] = _num_bytes+4;     checksum +=  _txBuffer[3];
    _txBuffer[4] = iSYNC_WRITE;    checksum +=  _txBuffer[4];
    _txBuffer[5] = _adress;    checksum +=  _txBuffer[5];
    _txBuffer[6] = (_num_bytes/_num_servo) - 1;     checksum +=  _txBuffer[6];
    // _num_bytes is bytes in packet + servo ID in all servo. Divided by num servo is just for one servo. Servo ID is sustracted ( - 1 )
    for (i = 0; i < _num_bytes; i++) {
        _txBuffer[i + 7] = _buffer[i];
        checksum +=  _txBuffer[i + 7];
    }
    _txBuffer[i + 7] = ~checksum;                 // Checksum with Bit Inversion
}

/**
 * @brief Warps packet info with the information needed for the comunication
 * @method JointHandler::warpSinglePacket
 * @param  instruction Instruction of how to access servo register (iREAD_DATA, iREG_WRITE, iWRITE_DATA...)
 * @param {uint8_t *} buffer                         data to warp
 * @param {uint8_t *} txBuffer                       data warped and ready to send
 * @see JointHandler::sendPacket()
 */
void JointHandler::warpSinglePacket(uint8_t _instruction, uint8_t *_buffer, uint8_t *_txBuffer) {
    DebugSerialJHLn("warpSinglePacket: begin of function");
    uint8_t i;
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)


    _txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    _txBuffer[1] = 0xFF;
    _txBuffer[2] = _buffer[0];      checksum +=  _txBuffer[2];  // _buffer[0] is ID from servo
    _txBuffer[3] = _buffer[1]+2;     checksum +=  _txBuffer[3];  // _buffer[1] is data length
    _txBuffer[4] = _instruction;    checksum +=  _txBuffer[4];
    // _txBuffer[5] = _buffer[2];    checksum +=  _txBuffer[5];  // _buffer[2] is register pos in which to write/read data
    for (i = 0; i < _buffer[1]; i++) {
        _txBuffer[i + 5] = _buffer[i + 2];
        checksum +=  _txBuffer[i + 5];
    }
    _txBuffer[i + 5] = ~checksum;                 // Checksum with Bit Inversion
}

/**
 * @brief Function to send to bus information contained in buffer param. Contains logic to read data in case it is needed
 * @method JointHandler::sendPacket
 * @param {uint8_t *} buffer      array with all the information to send, if info is read it will be copied here
 * @return             error in comunication
 */
uint16_t JointHandler::sendPacket(uint8_t *_txBuffer) {
    DebugSerialJHLn("sendPacket: begin of function");
    uint8_t readCount = 0;
    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t parameterLength = 0;
    uint8_t status[BUFFER_LEN];
    uint8_t txBuffer_print[BUFFER_LEN];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
    uint16_t error = 0;

    setTxMode();

    packetLength = _txBuffer[3] + 4;  // Number of bytes for the whole packet

    if (hardwareSerial_ == true) {
        for (i = 0; i < packetLength; i++) {
          Serial_G15_lib.write(_txBuffer[i]);
        }
        Serial_G15_lib.flush();
    } else {
        G15Serial_->listen();
        for (i = 0; i < packetLength; i++) {
            G15Serial_->write(_txBuffer[i]);
            txBuffer_print[i] = _txBuffer[i];
        }
        G15Serial_->flush();
    }

    // G15 only response if it was not broadcast command
    if ((_txBuffer[2] != 0xFE) || (_txBuffer[4] == iPING)) {
        if (_txBuffer[4] == iREAD_DATA) {
            parameterLength = _txBuffer[6];
            packetLength = _txBuffer[6] + 6;
        } else {
            packetLength = 6;
        }

        setRxMode();

        if (hardwareSerial_ == true) {
            readCount = Serial_G15_lib.readBytes(status, packetLength);
        } else {
            readCount = G15Serial_->readBytes(status, packetLength);
        }

        setTxMode();

        error = 0;
        if (readCount != packetLength) {
            error |= 0x0100;
        }
        if ((status[0] != 0xFF) || (status[1] != 0xFF)) {
            error |= 0x0200;
        }
        if (status[2] != _txBuffer[2]) {
            error |= 0x0400;
        }
        if (status[4] != 0) {
            error |= status[4];
        }

        // Calculate checksum
        checksum = 0;
        for (i = 2; i < packetLength; i++) {
            checksum += status[i];
        }
        if (checksum != 0xFF) {
            error |= 0x0800;
        }
        if (status[4] == 0x00 && (error & 0x0100) == 0x00) {    // Copy data only if there is no packet error
            if (_txBuffer[4] == iPING) {
                  _txBuffer[0] = status[2];
            } else if (_txBuffer[4] == iREAD_DATA) {
                  for (i = 0; i < parameterLength; i++) {
                      _txBuffer[i] = status[i + 5];
                  }
            }
        }
    }

    /*Serial.println("- Packet Sent: ");
    Serial.print("  [");
    for (i = 0; i < txBuffer_print[3]+4; i++) {
        Serial.print(", "); Serial.print("0x"); Serial.print(txBuffer_print[i], HEX);
    }
    Serial.println("]");
    Serial.println("");

    Serial.println("- Packet received: ");
    Serial.print("  [");
    for (i = 0; i < packetLength; i++) {
        Serial.print(", "); Serial.print("0x"); Serial.print(status[i], HEX);
    }
    Serial.println("]");
    Serial.println("");*/
    return(error);
}


/***********************************************
 *        Inherited from Cytron Library        *
 ***********************************************/

/**
 * @brief Constructor with custom software serial.
 * @method JointHandler::JointHandler
 * @param  rxpin                    RX pin for serial comunication
 * @param  txpin                    TX pin for serial comunication
 * @param  ctrlpin                  control pin for serial comunication
 */
JointHandler::JointHandler(uint8_t _rxpin, uint8_t _txpin, uint8_t _ctrlpin) {
     rxpin_shield_ = _rxpin;
     txpin_shield_ = _txpin;
     ctrlpin_shield_ = _ctrlpin;
}

/**
 * @brief Constructor with default hardwareSerial (RX in pin 0, TX in pin 1) and with set control pin
 * @method JointHandler::JointHandler
 * @param  ctrlpin                    control pin for serial comunication
 */
JointHandler::JointHandler(uint8_t _ctrlpin) {
     rxpin_shield_ = 0;
     txpin_shield_ = 1;
     ctrlpin_shield_ = _ctrlpin;
}

/**
 * @brief Method to set serial data (rx, tx, ctrlpin and baudrate) to init communication
 * @method JointHandler::initSerial
 * @param  rxpin                    RX pin for serial comunication
 * @param  txpin                    TX pin for serial comunication
 * @param  ctrlpin                  control pin for serial comunication
 * @param  baudrate                 Baudrate in which to communicate
 * @see JointHandler::begin()
 */
void JointHandler::initSerial(uint8_t _rxpin, uint8_t _txpin, uint8_t _ctrlpin, uint32_t _baudrate) {
     DebugSerialJHLn("initSerial: begin of function");
     rxpin_shield_ = _rxpin;
     txpin_shield_ = _txpin;
     ctrlpin_shield_ = _ctrlpin;
     begin(_baudrate);
}

/**
 * @brief Cpnfigures comunication at a set baudrate. Sets the serial port
 * @method JointHandler::begin
 * @param  baudrate            Baudrate in which to communicate
 */
void JointHandler::begin(uint32_t _baudrate) {
    DebugSerialJHLn2("begin: begin at baudrate: ", _baudrate);
    if ((rxpin_shield_ == 0 && txpin_shield_ == 1) || (CHECK_MEGA_HARDWARESERIAL(rxpin_shield_, txpin_shield_))) {
        hardwareSerial_ = true;
        Serial_G15_lib.begin(_baudrate);
        while (!Serial) {}
        Serial_G15_lib.setTimeout(SerialTimeOut);
    } else {
        hardwareSerial_ = false;
        pinMode(rxpin_shield_, INPUT);
        pinMode(txpin_shield_, OUTPUT);
        G15Serial_ = new SoftwareSerial(rxpin_shield_, txpin_shield_);
        G15Serial_->begin(_baudrate);
        G15Serial_->setTimeout(SerialTimeOut);
    }
    pinMode(ctrlpin_shield_, OUTPUT);
    setTxMode();
}

void JointHandler::end(void) {
    if ((rxpin_shield_ == 0 && txpin_shield_ == 1)  || CHECK_MEGA_HARDWARESERIAL(rxpin_shield_, txpin_shield_)) {
        Serial_G15_lib.end();
    } else {
        pinMode(rxpin_shield_, INPUT);
        pinMode(txpin_shield_, INPUT);
        G15Serial_->end();
    }
    pinMode(ctrlpin_shield_, INPUT);
}

void JointHandler::setTxMode(void) {
     digitalWrite(ctrlpin_shield_, TxMode);
}

void JointHandler::setRxMode(void) {
     digitalWrite(ctrlpin_shield_, RxMode);
}

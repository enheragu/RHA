/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 21_Sep_2017
 */


#include "joint_handler.h"

/**
  * @brief
  */
JointHandler::JointHandler(uint64_t timer) {
    timer_ = timer;
}

/**
 * @brief Initialices timer from control_loop
 * @method JointHandler::init
 * @param  timer              [description]
 */
void JointHandler::setTimer(uint64_t timer) {
    DebugSerialJHLn("setTimer: begin of function");
    control_loop_timer_.setTimer(timer);
    control_loop_timer_.activateTimer();
}

/**
 * @brief Initialices joints
 * @method JointHandler::initJoints
 */
void JointHandler::initJoints() {
    DebugSerialJHLn("initJoints: begin of function");
    uint8_t buffer[BUFFER_LEN];
    joint_[0].init(1, CW, A0);

    Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_DELAY);
    eeprom_timer.activateTimer();

    for (uint8_t i = 0; i < NUM_JOINT; i++) {
          joint_[i].servo_.setTorqueOnOfToPacket(buffer, ON);
          uint16_t error = sendSinglePacket(iWRITE_DATA, buffer);
          DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }

    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();

    for (uint8_t i = 0; i < NUM_JOINT; i++) {
          sendSetWheelModeAll();
    }
    eeprom_timer.checkWait();
}

/**
  * @brief controlLoop() function handles control loop for servo speed
  */
void JointHandler::controlLoop() {
    DebugSerialJHLn("controlLoop: begin of function");
    if(control_loop_timer_.checkContinue()) {
        // Firt all info have to be updated
        DebugSerialJHLn("controlLoop: Updating joint info");
        updateJointInfo();

        // Ask each joint torque to update speed
        DebugSerialJHLn("controlLoop: Updating joint error torque");
        updateJointErrorTorque();

        DebugSerialJHLn("controlLoop: send new torque to servos");
        sendJointTorques();
        //Send buffer

        control_loop_timer_.activateTimer();

    }
}

/**
 * @brief Updates internal information from all joint.
 * @method JointHandler::updateJointInfo
 */
void JointHandler::updateJointInfo() {
    DebugSerialJHLn("updateJointInfo: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.addUpadteInfoToPacket(buffer);
        uint16_t error = sendSinglePacket(iREAD_DATA, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
        joint_[i].servo_.updateInfo(&buffer[0], error);
    }

}

/**
 * @brief Updates all joints error to update torque goal
 * @method JointHandler::updateJointErrorTorque
 */
void JointHandler::updateJointErrorTorque() {
    DebugSerialJHLn("updateJointErrorTorque: begin of function");
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.calculateTorque(joint_[i].speedError());
    }
}

/**
 * @brief handles the packet construction and sent for all joint torques
 * @method JointHandler::sendJointTorques
 */
void JointHandler::sendJointTorques() {
    DebugSerialJHLn("sendJointTorques: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        if (joint_[i].servo_.addTorqueToPacket(buffer)) {
            num_servo++;
            num_bytes += addToSyncPacket(&buffer_to_send[num_bytes], buffer);
        }
    }
    sendSyncPacket(iWRITE_DATA, buffer_to_send, num_bytes, num_servo);
}

/**
 * @brief Sets speed goal to a given joint (based on servo ID in goal)
 * @method JointHandler::setSpeedGoal
 * @param  goal Goal containing speed, speed_slope and ID
 */
void JointHandler::setSpeedGoal(SpeedGoal goal) {
    DebugSerialJHLn("setSpeedGoal: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++){
        if (joint_[i].setSpeedGoal(goal)) return;
    }

}

void JointHandler::sendSetWheelModeAll() {
    DebugSerialJHLn("sendSetWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.setWheelModeToPacket(buffer);
        uint16_t error = sendSinglePacket(iWRITE_DATA, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

void JointHandler::sendExitWheelModeAll() {
    DebugSerialJHLn("sendExitWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.exitWheelModeToPacket(buffer);
        uint16_t error = sendSinglePacket(iWRITE_DATA, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
void sendSetTorqueLimitAll(uint16_t torque_limit) {
    DebugSerialJHLn("sendSetTorqueLimitAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.setTorqueLimitToPacket(buffer, torque_limit);
        uint16_t error = sendSinglePacket(iWRITE_DATA, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
void sendSetWheelSpeedAll(uint16_t speed, uint8_t direction) {
    DebugSerialJHLn("sendSetWheelSpeedAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.exitWheelModeToPacket(buffer, speed, direction);
        uint16_t error = sendSinglePacket(iWRITE_DATA, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
bool JointHandler::checkConectionAll() {
    DebugSerialJHLn("checkConectionAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++){
        joint_[i].servo_.pingToPacket(buffer);
        uint16_t error = sendSinglePacket(iPING, buffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
        if (error != 0) return false;
    }
    return true;
}

/**
 * @brief Adds data to common buffer. Intended to put commands from all servo into one packet
 * @method JointHandler::addToSyncPacket
 * @param  buffer array to write all the info
 * @param  data   contains data to copy
 * @return        returns length copied in bytes
 */
uint8_t JointHandler::addToSyncPacket(uint8_t *buffer, uint8_t *data) {
    DebugSerialJHLn("addToSyncPacket: begin of function");
    *buffer = data[0]; buffer++;
    for (int i = 0; i < data[2]; i++) {
        *buffer = data[i+3];  buffer++;  // component 2 and 3 are packet_len and instruction
    }
    return (uint8_t)(data[2] + 1);  // Packet len + servo ID
}



/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @method JointHandler::sendSyncPacket
  * @param {uint8_t} instruction is the instruction to send
  * @param {uint8_t *} buffer is the data that have been completed by each servo (by reference)
  * @param {uint8_t} num_bytes is the length of data
  * @param {uint8_t} num_servo how many servos had been added to this packet
  */
void JointHandler::sendSyncPacket(uint8_t instruction, uint8_t *buffer, uint8_t num_bytes, uint8_t num_servo) {
    DebugSerialJHLn("sendSyncPacket: begin of function");

    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t txBuffer[BUFFER_LEN];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)

    setTxMode();


    txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    txBuffer[1] = 0xFF;
    txBuffer[2] = ALL_SERVO;      checksum +=  txBuffer[2];
    txBuffer[3] = num_bytes+4;     checksum +=  txBuffer[3];
    txBuffer[4] = iSYNC_WRITE;    checksum +=  txBuffer[4];
    txBuffer[5] = instruction;    checksum +=  txBuffer[5];
    txBuffer[6] = num_servo;     checksum +=  txBuffer[6];
    for (i = 0; i < num_bytes; i++) {
        txBuffer[i+7] = buffer[i];
        checksum +=  txBuffer[i+7];
    }
    txBuffer[i+7] = ~checksum;                 // Checksum with Bit Inversion


    packetLength = txBuffer[3] + 4;  // Number of bytes for the whole packet

    if (hardwareSerial_ == true) {
        for (i = 0; i < packetLength; i++) {
            Serial.write(txBuffer[i]);
        }
        Serial.flush();
    } else {
        G15Serial_->listen();
        for (i = 0; i < packetLength; i++) {
            G15Serial_->write(txBuffer[i]);
        }
        G15Serial_->flush();
    }
}

/**
 * @brief Function to send to bus information contained in buffer param (just for one servo). Contains logic to read data in case it is needed
 * @method JointHandler::sendSinglePacket
 * @param  instruction instruction for servo register (iREAD_DATA, iREG_WRITE, iWRITE_DATA...)
 * @param  buffer      array with all the information to send, if info is read it will be copied here
 * @return             error in comunication
 */
uint16_t JointHandler::sendSinglePacket(uint8_t instruction, uint8_t *buffer) {
    DebugSerialJHLn("sendSyncPacket: begin of function");
    uint8_t readCount = 0;
    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t parameterLength = 0;
    uint8_t txBuffer[BUFFER_LEN];
    uint8_t status[16];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
    uint16_t error = 0;

    setTxMode();

    txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    txBuffer[1] = 0xFF;
    txBuffer[2] = buffer[0];      checksum +=  txBuffer[2];  // buffer[0] is ID from servo
    txBuffer[3] = buffer[1]+2;     checksum +=  txBuffer[3];  // buffer[1] is data length
    txBuffer[4] = instruction;    checksum +=  txBuffer[4];
    // txBuffer[5] = buffer[2];    checksum +=  txBuffer[5];  // buffer[2] is register pos in which to write/read data
    for (i = 0; i < buffer[1]; i++) {
        txBuffer[i+5] = buffer[i+2];
        checksum +=  txBuffer[i+5];
    }
    txBuffer[i+7] = ~checksum;                 // Checksum with Bit Inversion

    packetLength = txBuffer[3] + 4;  // Number of bytes for the whole packet

    if (hardwareSerial_ == true) {
        for (i = 0; i < packetLength; i++) {
          Serial.write(txBuffer[i]);
        }
        Serial.flush();
    } else {
        G15Serial_->listen();
        for (i = 0; i < packetLength; i++) {
            G15Serial_->write(txBuffer[i]);
        }
        G15Serial_->flush();
    }

    // G15 only response if it was not broadcast command
    if ((txBuffer[2] != 0xFE) || (instruction == iPING)) {
        if (instruction == iREAD_DATA) {
            parameterLength = buffer[1];
            packetLength = buffer[1] + 6;
        } else {
            packetLength = 6;
        }

        setRxMode();

        if (hardwareSerial_ == true) {
            readCount = Serial.readBytes(status, packetLength);
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
        if (status[2] != buffer[0]) {
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
            if (instruction == iPING) {
                  buffer[0] = status[2];
            } else if (instruction == iREAD_DATA) {
                  for (i = 0; i < parameterLength; i++) {
                      buffer[i] = status[i + 5];
                  }
            }
        }
    }
    return(error);
}


/***********************************************
 *        Inherited from Cytron Library        *
 ***********************************************/

JointHandler::JointHandler(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin) {
     rxpin_shield_ = rxpin;
     txpin_shield_ = txpin;
     ctrlpin_shield_ = ctrlpin;
}

JointHandler::JointHandler(uint8_t ctrlpin) {
     rxpin_shield_ = 0;
     txpin_shield_ = 1;
     ctrlpin_shield_ = ctrlpin;
}

void JointHandler::initSerial(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate){
     DebugSerialJHLn("initSerial: begin of function");
     rxpin_shield_ = rxpin;
     txpin_shield_ = txpin;
     ctrlpin_shield_ = ctrlpin;
     begin(baudrate);
}

void JointHandler::begin(uint32_t baudrate) {
     DebugSerialJHLn2("begin: begin at baudrate: ", baudrate);
   if (rxpin_shield_ == 0 &&txpin_shield_ == 1) {
         hardwareSerial_ = true;
         Serial.begin(baudrate);
         while (!Serial) {}
         Serial.setTimeout(SerialTimeOut);
     } else {
         hardwareSerial_ = false;
         pinMode(rxpin_shield_, INPUT);
         pinMode(txpin_shield_, OUTPUT);
         G15Serial_ = new SoftwareSerial(rxpin_shield_, txpin_shield_);
         G15Serial_->begin(baudrate);
         G15Serial_->setTimeout(SerialTimeOut);
     }
     pinMode(ctrlpin_shield_, OUTPUT);
     setTxMode();
}

void JointHandler::end(void) {
     if (rxpin_shield_ == 0 &&txpin_shield_ == 1) {
         Serial.end();
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

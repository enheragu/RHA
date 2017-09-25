/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   quique
 * @Last modified time: 25-Sep-2017
 */

// #include "HardwareSerial.h"
#include "joint_handler.h"

boolean hardwareSerial_ = false;
SoftwareSerial* G15Serial_;

/**
  * @brief
  */
JointHandler::JointHandler(uint64_t timer) {
    control_loop_timer_.setTimer(timer);
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
    joint_[0].init(1, CW, A0);

    sendSetWheelModeAll();
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
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.addUpadteInfoToPacket(buffer);
        uint8_t txBuffer[BUFFER_LEN];
        warpSinglePacket(iREAD_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
        joint_[i].servo_.updateInfo(txBuffer, error);
    }

}

/**
 * @brief Updates all joints error to update torque goal
 * @method JointHandler::updateJointErrorTorque
 */
void JointHandler::updateJointErrorTorque() {
    DebugSerialJHLn("updateJointErrorTorque: begin of function");
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
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
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        if (joint_[i].servo_.addTorqueToPacket(buffer)) {
            num_servo++;
            num_bytes = addToSyncPacket(buffer_to_send, buffer, num_bytes);
        }
    }
    uint8_t txBuffer[BUFFER_LEN];
    warpSyncPacket(buffer_to_send, ServoRHAConstants::MOVING_SPEED_L, txBuffer, num_bytes, num_servo);
    sendPacket(txBuffer);
}

/**
 * @brief Sets speed goal to a given joint (based on servo ID in goal)
 * @method JointHandler::setSpeedGoal
 * @param  goal Goal containing speed, speed_slope and ID
 */
void JointHandler::setSpeedGoal(RHATypes::SpeedGoal goal) {
    DebugSerialJHLn("setSpeedGoal: begin of function");
    for (uint8_t i = 0; i < NUM_JOINT; i++) {
        if (joint_[i].setSpeedGoal(goal)) return;
    }

}

void JointHandler::sendSetWheelModeAll() {
    DebugSerialJHLn("sendSetWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];

    RHATypes::Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_DELAY);
    eeprom_timer.activateTimer();

    DebugSerialJHLn("sendSetWheelModeAll: set wheel mode");
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        // First set angle limit
        joint_[i].servo_.setWheelModeToPacket(buffer);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();
    DebugSerialJHLn("sendSetWheelModeAll: set torque ON");
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        // Then set torque ON
        joint_[i].servo_.setTorqueOnOfToPacket(buffer, ON);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
    eeprom_timer.checkWait();
}

void JointHandler::sendExitWheelModeAll() {
    DebugSerialJHLn("sendExitWheelModeAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.exitWheelModeToPacket(buffer);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
void JointHandler::sendSetTorqueLimitAll(uint16_t torque_limit) {
    DebugSerialJHLn("sendSetTorqueLimitAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.setTorqueLimitToPacket(buffer, torque_limit);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
void JointHandler::sendSetWheelSpeedAll(uint16_t speed, uint8_t direction) {
    DebugSerialJHLn("sendSetWheelSpeedAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.setWheelModeToPacket(buffer);
        warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
        DebugSerialJHLn4Error(error, joint_[i].servo_.getID());
    }
}

// NOTE: testing purposes
bool JointHandler::checkConectionAll() {
    DebugSerialJHLn("checkConectionAll: begin of function");
    uint8_t buffer[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    for(uint8_t i = 0; i < NUM_JOINT; i++) {
        joint_[i].servo_.pingToPacket(buffer);
        warpSinglePacket(iPING, buffer, txBuffer);
        uint16_t error = sendPacket(txBuffer);
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
uint8_t JointHandler::addToSyncPacket(uint8_t *buffer, uint8_t *data, uint8_t num_bytes) {
    DebugSerialJHLn("addToSyncPacket: begin of function");
    buffer[num_bytes] = data[0]; num_bytes++;
    for (int i = 0; i < data[1]-1; i++) {  //skips lenght and register addres
        buffer[num_bytes] = data[i+3];  num_bytes++;  // component 2 and 3 are packet_len and instruction
    }
    return num_bytes;  // Packet len + servo ID
}



/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @method JointHandler::warpSyncPacket
  * @param {uint8_t} instruction is the instruction to send
  * @param {uint8_t *} buffer is the data that have been completed by each servo (by reference)
  * @param {uint8_t} num_bytes is the length of data
  * @param {uint8_t} num_servo how many servos had been added to this packet
  */
void JointHandler::warpSyncPacket(uint8_t *buffer, uint8_t address, uint8_t *txBuffer, uint8_t num_bytes, uint8_t num_servo) {
    DebugSerialJHLn("warpSyncPacket: begin of function");

    uint8_t i;
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)

    txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    txBuffer[1] = 0xFF;
    txBuffer[2] = ALL_SERVO;      checksum +=  txBuffer[2];
    txBuffer[3] = num_bytes+4;     checksum +=  txBuffer[3];
    txBuffer[4] = iSYNC_WRITE;    checksum +=  txBuffer[4];
    txBuffer[5] = address;    checksum +=  txBuffer[5];
    txBuffer[6] = (num_bytes/num_servo) - 1;     checksum +=  txBuffer[6];
    // num_bytes is bytes in packet + servo ID in all servo. Divided by num servo is just for one servo. Servo ID is sustracted ( - 1 )
    for (i = 0; i < num_bytes; i++) {
        txBuffer[i + 7] = buffer[i];
        checksum +=  txBuffer[i + 7];
    }
    txBuffer[i + 7] = ~checksum;                 // Checksum with Bit Inversion
}

void JointHandler::warpSinglePacket(uint8_t instruction, uint8_t *buffer, uint8_t *txBuffer) {
    DebugSerialJHLn("warpSinglePacket: begin of function");
    uint8_t i;
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)


    txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    txBuffer[1] = 0xFF;
    txBuffer[2] = buffer[0];      checksum +=  txBuffer[2];  // buffer[0] is ID from servo
    txBuffer[3] = buffer[1]+2;     checksum +=  txBuffer[3];  // buffer[1] is data length
    txBuffer[4] = instruction;    checksum +=  txBuffer[4];
    // txBuffer[5] = buffer[2];    checksum +=  txBuffer[5];  // buffer[2] is register pos in which to write/read data
    for (i = 0; i < buffer[1]; i++) {
        txBuffer[i + 5] = buffer[i + 2];
        checksum +=  txBuffer[i + 5];
    }
    txBuffer[i + 5] = ~checksum;                 // Checksum with Bit Inversion
}

/**
 * @brief Function to send to bus information contained in buffer param (just for one servo). Contains logic to read data in case it is needed
 * @method JointHandler::sendPacket
 * @param  instruction instruction for servo register (iREAD_DATA, iREG_WRITE, iWRITE_DATA...)
 * @param  buffer      array with all the information to send, if info is read it will be copied here
 * @return             error in comunication
 */
uint16_t JointHandler::sendPacket(uint8_t *txBuffer) {
    DebugSerialJHLn("sendPacket: begin of function");
    uint8_t readCount = 0;
    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t parameterLength = 0;
    uint8_t status[BUFFER_LEN];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
    uint16_t error = 0;

    setTxMode();

    packetLength = txBuffer[3] + 4;  // Number of bytes for the whole packet

    if (hardwareSerial_ == true) {
        for (i = 0; i < packetLength; i++) {
          Serial.write(txBuffer[i]);
        }
        Serial.flush();
    } else {
        G15Serial_->listen();
        Serial.println("- Packet Sent: ");
        Serial.print("  [");
        for (i = 0; i < packetLength; i++) {
            G15Serial_->write(txBuffer[i]);
            Serial.print(", "); Serial.print("0x"); Serial.print(txBuffer[i], HEX);
        }
        Serial.println("]");
        Serial.println("");
        G15Serial_->flush();
    }

    // G15 only response if it was not broadcast command
    if ((txBuffer[2] != 0xFE) || (txBuffer[4] == iPING)) {
        if (txBuffer[4] == iREAD_DATA) {
            parameterLength = txBuffer[6];
            packetLength = txBuffer[6] + 6;
        } else {
            packetLength = 6;
        }

        setRxMode();

        if (hardwareSerial_ == true) {
            readCount = Serial.readBytes(status, packetLength);
        } else {
            readCount = G15Serial_->readBytes(status, packetLength);
        }

        Serial.println("- Packet received: ");
        Serial.print("  [");
        for (i = 0; i < packetLength; i++) {
            Serial.print(", "); Serial.print("0x"); Serial.print(status[i], HEX);
        }
        Serial.println("]");
        Serial.println("");
        setTxMode();

        error = 0;
        if (readCount != packetLength) {
            error |= 0x0100;
        }
        if ((status[0] != 0xFF) || (status[1] != 0xFF)) {
            error |= 0x0200;
        }
        if (status[2] != txBuffer[2]) {
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
            if (txBuffer[4] == iPING) {
                  txBuffer[0] = status[2];
            } else if (txBuffer[4] == iREAD_DATA) {
                  for (i = 0; i < parameterLength; i++) {
                      txBuffer[i] = status[i + 5];
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

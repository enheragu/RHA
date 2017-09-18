/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
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
    timer_ = timer;
}

/**
 * @brief Initialices joints
 * @method JointHandler::initJoints
 */
void JointHandler::initJoints() {
    DebugSerialJHLn("initJoints: begin of function");
    joint_[1].init(1, CW, A0);
}

/**
  * @brief controlLoop() function handles control loop for servo speed
  */
void JointHandler::controlLoop() {
    DebugSerialJHLn("controlLoop: begin of function");
    servos_packet_ = 0;
    if(millis() - time_last_ >= timer_) {
        // Firt all info have to be updated
        updateJointInfo();

        // Ask each joint torque to update speed
        updateJointErrorTorque();

        addJointToPacket();
        wrapSyncPacket(JointHandlerConstants::MOVING_SPEED_L);
        //Send buffer

        sendSyncPacket();

        time_last_ = millis();
    }
}

/**
 * @brief Updates internal information from all joint.
 * @method JointHandler::updateJointInfo
 */
void JointHandler::updateJointInfo() {
    cleanBuffer();
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        joint_[i].servo_.updateInfoToPacket(buffer_);
        wrapSinglePacket(buffer_);
        joint_[i].servo_.updateInfo(buffer_);
    }

}

void JointHandler::updateJointErrorTorque() {
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        joint_.servo_.regulatorJoint(joint_.speedError());
    }
}

void JointHandler::addJointToPacket() {
    bytes_write_ = 0;
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        if (joint_.servo_.addTorqueToPacket(buffer_, bytes_write_, bytes_write_))
        servos_packet_++;
    }
}

/**
 * @brief Sets speed goal to a given joint (based on servo ID in goal)
 * @method JointHandler::setSpeedGoal
 * @param  goal Goal containing speed, speed_slope and ID
 */
void JointHandler::setSpeedGoal(SpeedGoal goal) {
    DebugSerialJHLn("setSpeedGoal: begin of function");
    for (uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        if (joint_[i].setSpeedGoal(goal)) return;
    }

}

/**
 * @brief Cleans buffer used for comunication
 * @method JointHandler::cleanBuffer
 */
void JointHandler::cleanBuffer() {
    for(uint8_t i = 0; i < BUFFER_LEN; i++){
        buffer_[1] = 0;
    }
}


/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} data is the data that have been completed by each servo (by reference)
  * @param {uint8_t} data_len is the length of data
  * @param {uint8_t} instruction is the instruction to send
  * @param {uint8_t} num_servo how many servos had been added to this packet
  * @return {uint8_t} Returns number of uint8_ts that contain usefull info (how many have been written)
  */
uint8_t JointHandler::wrapSyncPacket(uint8_t instruction) {
    DebugSerialJHLn("wrapSyncPacket: begin of function");
    int i = 0;
    char checksum = 0;    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
    instruction_ = instruction;
    buffer_send_[0] = 0xFF;               // 0xFF not included in checksum
    buffer_send_[1] = 0xFF;
    buffer_send_[2] = ALL_SERVO;      checksum +=  buffer[2];
    buffer_send_[3] = bytes_write_+4;     checksum +=  buffer[3];
    buffer_send_[4] = iSYNC_WRITE;    checksum +=  buffer[4];
    buffer_send_[5] = instruction;    checksum +=  buffer[5];
    buffer_send_[6] = servos_packet_;     checksum +=  buffer[6];
    for (i = 0; i < data_len; i++) {
        buffer_send_[i+7] = buffer_[i];
        checksum +=  buffer[i+7];
    }
    buffer_send_[i+7] = ~checksum;                 // Checksum with Bit Inversion
    DebugSerialJHLn("wrapSyncPacket: end of function");
    return buffer_send_[3] + 4;
}

/** @brief wrapPacket adds information needed once servo info has been received.
  * @param {uint8_t *} buffer is the buffer in which the information will be added (by reference)
  * @param {uint8_t *} data is the data that have been completed by each servo (by reference)
  * @return {uint8_t} Returns number of uint8_ts that contain usefull info (how many have been written)
  */
uint8_t JointHandler::wrapSinglePacket() {
    DebugSerialJHLn("wrapSinglePacket: begin of function");
    int i = 0;
    char checksum = 0;    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)

    buffer_send_[0] = 0xFF;               // 0xFF not included in checksum
    buffer_send_[1] = 0xFF;
    buffer_send_[2] = buffer[0];      checksum +=  buffer[2];
    buffer_send_[3] = buffer[1]+4;     checksum +=  buffer[3];
    buffer_send_[4] = iWRITE_DATA;    checksum +=  buffer[4];
    buffer_send_[5] = buffer_[2];    checksum +=  buffer[5];
    for (i = 0; i < buffer_[1]; i++) {
        buffer_send_[i+6] = data[i];
        checksum +=  buffer_[i+6];
    }
    buffer_send_[i+7] = ~checksum;                 // Checksum with Bit Inversion
    DebugSerialJHLn("wrapSinglePacket: end of function");
    return buffer_send_[3] + 4;
}

void JointHandler::sendSyncPacket() {
    packetLength_ = buffer_send_[3];
    if (hardwareSerial_ == true) {
        for (i = 0; i < packetLength_; i++) {
            Serial.write(buffer_send_[i]);
        }
        Serial.flush();
    } else {
        G15Serial_->listen();
        for (i = 0; i < packetLength_; i++) {
            G15Serial_->write(buffer_send_[i]);
        }
        G15Serial_->flush();
    }
}

void JointHandler::sendSinglePacket() {
    packetLength_ = buffer_send_[3];
    if (hardwareSerial_ == true) {
        for (i = 0; i < buffer_send_[3]; i++) {
            Serial.write(buffer_send_[i]);
        }
        Serial.flush();
    } else {
        G15Serial_->listen();
        for (i = 0; i < buffer_send_[3]; i++) {
            G15Serial_->write(buffer_send_[i]);
        }
        G15Serial_->flush();
    }

    if (instruction == iREAD_DATA) {
        parameterLength = data[1];
        packetLength = data[1] + 6;
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
    if (status[2] != id) {
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
            data[0] = status[2];
        } else if (instruction == iREAD_DATA) {
            for (i = 0; i < parameterLength; i++) {
                data[i] = status[i + 5];
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

void JointHandler::init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin){
     rxpin_shield_ = rxpin;
     txpin_shield_ = txpin;
     ctrlpin_shield_ = ctrlpin;
     begin(G15_BAUDRATE);
}

void JointHandler::init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate){
     rxpin_shield_ = rxpin;
     txpin_shield_ = txpin;
     ctrlpin_shield_ = ctrlpin;
     begin(baudrate);
}

void JointHandler::begin(uint32_t baudrate) {
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

 // Send packet
 // Caution: At least 2 bytes of data array need to be passed into the function
uint16_t JointHandler::sendPacket(uint8_t id, uint8_t instruction, uint8_t* data, uint8_t parameterLength) {
    uint8_t readCount = 0;
    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t txBuffer[16];
    uint8_t status[16];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
    uint16_t error = 0;

    setTxMode();

    txBuffer[0] = 0xFF;  // Header is 0xFF
    txBuffer[1] = 0xFF;  // Header is not included in checksum
    txBuffer[2] = id;              checksum += txBuffer[2];  // 0-254, 0xFE = broadcast id
    txBuffer[3] = parameterLength + 2;    checksum += txBuffer[3];  // Instruction + parameters (start add + values) + checksum                                                                                                        // 0xFF and ID not included
    txBuffer[4] = instruction;          checksum += txBuffer[4];

    for (i = 0; i < parameterLength; i++) {
        txBuffer[i + 5] = data[i];
        checksum += txBuffer[i + 5];
    }
    txBuffer[i+5] = ~checksum;

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
    //if ((ID != 0xFE) || (instruction == iPING)) {
    {
     if (instruction == iREAD_DATA) {
         parameterLength = data[1];
         packetLength = data[1] + 6;
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
     if (status[2] != id) {
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
             data[0] = status[2];
         } else if (instruction == iREAD_DATA) {
             for (i = 0; i < parameterLength; i++) {
                 data[i] = status[i + 5];
             }
         }
     }
    }
    return(error);
}

/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 19_Sep_2017
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
    uint8_t buffer[JointHandlerConstants::BUFFER_LEN];
    joint_[1].init(1, CW, A0);

    for (uint8_t i = 0; i < JointHandlerConstants::NUM_JOINT; i++) {
          joint_[i].servo_.setTorqueOnOfToPacket(buffer, ON);
          sendSinglePacket(iWRITE_DATA, buffer);
    }
    for (uint8_t i = 0; i < JointHandlerConstants::NUM_JOINT; i++) {
          joint_[i].servo_.setWheelModeToPacket(buffer);
          sendSinglePacket(iWRITE_DATA, buffer);
    }
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
    uint8_t buffer[JointHandlerConstants::BUFFER_LEN];
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        joint_[i].servo_.updateInfoToPacket(buffer);
        uint16_t error = sendSinglePacket(iREAD_DATA, buffer);
        joint_[i].servo_.updateInfo(buffer, error);
    }

}

void JointHandler::updateJointErrorTorque() {
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        joint_.servo_.calculateTorque(joint_.speedError());
    }
}

void JointHandler::sendJointTorques() {
    uint8_t buffer[JointHandlerConstants::BUFFER_LEN];
    uint8_t buffer_to_send[JointHandlerConstants::BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;
    for(uint8_t servo = 0; i < JointHandlerConstants::NUM_JOINT; i++){
        if (joint_.servo_.addTorqueToPacket(buffer)) {
            num_servo++;
            num_bytes += addToSyncPacket(buffer_to_send[bytes_write_], buffer);
        }
    }
    sendSyncPacket(iWRITE_DATA, buffer_to_send, num_bytes, num_servo)
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

uint8_t addToSyncPacket(uint8_t * &buffer, uint8_t *data) {
  *buffer = data[0]; buffer++;
  for (int i = 0; i < data[2]; i++) {
      *buffer = data[i+3];  buffer++;  // component 2 and 3 are packet_len and instruction
  }
  return data[2] + 1;  // Packet len + servo ID
}
/** @brief wrapPacket adds information needed once all servos had been aded (header, ID, instruction...). This function is used to send just one packet for all servos instead of each sending their respective information
  * @param {uint8_t *} data is the data that have been completed by each servo (by reference)
  * @param {uint8_t} data_len is the length of data
  * @param {uint8_t} instruction is the instruction to send
  * @param {uint8_t} num_servo how many servos had been added to this packet
  * @return {uint8_t} Returns number of uint8_ts that contain usefull info (how many have been written)
  */
uint8_t JointHandler::sendSyncPacket(uint8_t instruction, uint8_t *buffer, uint8_t num_bytes, uint8_t num_servo) {
    DebugSerialJHLn("wrapSyncPacket: begin of function");

    uint8_t readCount = 0;
    uint8_t i;
    uint8_t packetLength = 0;
    uint8_t txBuffer[JointHandlerConstants::BUFFER_LEN];
    uint8_t status[16];
    uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
    uint16_t error = 0;

    setTxMode();


    char checksum = 0;    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)

    txBuffer[0] = 0xFF;               // 0xFF not included in checksum
    txBuffer[1] = 0xFF;
    txBuffer[2] = ALL_SERVO;      checksum +=  txBuffer[2];
    txBuffer[3] = num_bytes+4;     checksum +=  txBuffer[3];
    txBuffer[4] = iSYNC_WRITE;    checksum +=  txBuffer[4];
    txBuffer[5] = instruction;    checksum +=  txBuffer[5];
    txBuffer[6] = num_servo;     checksum +=  txBuffer[6];
    for (i = 0; i < num_bytes; i++) {
        txBuffer[i+7] = buffer_[i];
        checksum +=  txBuffer[i+7];
    }
    txBuffer[i+7] = ~checksum;                 // Checksum with Bit Inversion


    packetLength = txBuffer[3] + 4;  // Number of bytes for the whole packet

    if (hardwareSerial == true) {
        for (i = 0; i < packetLength; i++) {
            Serial.write(txBuffer[i]);
        }
        Serial.flush();
    } else {
        G15Serial->listen();
        for (i = 0; i < packetLength; i++) {
            G15Serial->write(txBuffer[i]);
        }
        G15Serial->flush();
    }
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

uint16_t JointHandler::sendSinglePacket(uint8_t instruction, uint8_t *buffer) {
  uint8_t readCount = 0;
  uint8_t i;
  uint8_t packetLength = 0;
  uint8_t txBuffer[JointHandlerConstants::BUFFER_LEN];
  uint8_t status[16];
  uint8_t checksum = 0;  // Checksum = ~(ID + Length + Instruction + Parameter1 + ... + Parameter n)
  uint16_t error = 0;

  setTxMode();


  char checksum = 0;    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)

  txBuffer[0] = 0xFF;               // 0xFF not included in checksum
  txBuffer[1] = 0xFF;
  txBuffer[2] = buffer[0];      checksum +=  txBuffer[2];
  txBuffer[3] = buffer[1]+2;     checksum +=  txBuffer[3];
  txBuffer[4] = instruction;    checksum +=  txBuffer[4];
  txBuffer[5] = buffer[2];    checksum +=  txBuffer[5];
  for (i = 0; i < buffer[1]; i++) {
      txBuffer[i+6] = buffer[i];
      checksum +=  txBuffer[i+6];
  }
  txBuffer[i+7] = ~checksum;                 // Checksum with Bit Inversion

  packetLength = txBuffer[3] + 4;  // Number of bytes for the whole packet

  if (hardwareSerial == true) {
      for (i = 0; i < packetLength; i++) {
          Serial.write(txBuffer[i]);
      }
      Serial.flush();
  } else {
      G15Serial->listen();
      for (i = 0; i < packetLength; i++) {
          G15Serial->write(txBuffer[i]);
      }
      G15Serial->flush();
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

      if (hardwareSerial == true) {
          readCount = Serial.readBytes(status, packetLength);
      } else {
          readCount = G15Serial->readBytes(status, packetLength);
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

void JointHandler::initSerial(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin){
     rxpin_shield_ = rxpin;
     txpin_shield_ = txpin;
     ctrlpin_shield_ = ctrlpin;
     begin(G15_BAUDRATE);
}

void JointHandler::initSerial(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate){
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

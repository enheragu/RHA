/**
 * @file
 * @brief Implements JointHandler functions defined in joint_handler.h
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 14-Sep-2017
 */


#include "joint_handler.h"

/**
  * @brief
  */
JointHandler::JointHandler(uint64_t timer){
    timer_ = timer;
}

/**
  * @brief
  */
void JointHandler::init(uint64_t timer){
    timer_ = timer;
    joint_.init(1, CW, A0);
}

/**
  * @brief controlLoop() function handles control loop for servo speed
  */
void JointHandler::controlLoop(){
    if(millis() - time_last_ >= timer_){
        // Firt all info have to be updated
        joint_.updateInfo();

        // Ask each joint torque to update speed
        uint8_t error = joint_.speedError();
        uint16_t torque = joint_.regulatorJoint();

        // Add all servo to the buffer
        uint8_t buffer_add[30], goal[2], buffer_wrap[35];
        uint8_t position = 0;
        uint8_t num_servo = 0;

        goal[0] = torque & 0x00FF;
        goal[1] = torque >> 8;  // Goal pos top 8 bits
        joint_.addToPacket(buffer_add, position, goal, sizeof(goal_3), num_servo);
        joint_.wrapPacket(buffer_wrap, buffer_add, position, MOVING_SPEED_L, num_servo);
        //Send buffer

        joint_.sendPacket(iWRITE_DATA, buffer_wrap, position)

        time_last_ = millis();
    }
}




/***********************************************
 *        Inherited from Cytron Library        *
 ***********************************************/


 JointHandler::JointHandler(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin) {
     rxpin_shield__ = rxpin;
     txpin_shield__ = txpin;
     ctrlpin_shield__ = ctrlpin;
 }

 JointHandler::JointHandler(uint8_t ctrlpin) {
     rxpin_shield__ = 0;
     txpin_shield__ = 1;
     ctrlpin_shield__ = ctrlpin;
 }

 void JointHandler::init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin){
     rxpin_shield__ = rxpin;
     txpin_shield__ = txpin;
     ctrlpin_shield__ = ctrlpin;
     begin(G15_BAUDRATE);
 }

 void JointHandler::init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate){
     rxpin_shield__ = rxpin;
     txpin_shield__ = txpin;
     ctrlpin_shield__ = ctrlpin;
     begin(baudrate);
 }

 void JointHandler::begin(uint32_t baudrate) {
   if (rxpin_shield_ == 0 &&txpin_shield_ == 1) {
         hardwareSerial = true;
         Serial.begin(baudrate);
         while (!Serial) {}
         Serial.setTimeout(SerialTimeOut);
     } else {
         hardwareSerial = false;
         pinMode(rxpin_shield_, INPUT);
         pinMode(txpin_shield_, OUTPUT);
         G15Serial = new SoftwareSerial(rxpin_shield_, txpin_shield_);
         G15Serial->begin(baudrate);
         G15Serial->setTimeout(SerialTimeOut);
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
         G15Serial->end();
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
     if ((ID != 0xFE) || (instruction == iPING)) {
         if (instruction == iREAD_DATA) {
             parameterLength = data[1];
             packetLength = data[1] + 6;
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

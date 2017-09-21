/**
 * @file
 * @brief Implements JointHandler class. This object is in charge to sync all joints.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.h
 * @Last modified by:   enheragu
 * @Last modified time: 21_Sep_2017
 */

#ifndef JOINT_HANDLER_H
#define JOINT_HANDLER_H

#include "joint_rha.h"
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "debug.h"
#include "rha_types.h"


namespace JointHandlerConstants {

    #define iPING 0x01  // Obtain a status packet
    #define iREAD_DATA 0x02  // Read Control Table values
    #define iWRITE_DATA 0x03  // Write Control Table values
    #define iREG_WRITE 0x04  // Write and wait for ACTION instruction
    #define iACTION 0x05  // Triggers REG_WRITE instruction
    #define iRESET 0x06  // Set factory defaults
    #define iSYNC_WRITE 0x83  // Simultaneously control multiple actuators

    #define SerialTimeOut 100L
    #define TxMode LOW   // Master transmit to G15
    #define RxMode HIGH   // Master receive from G15
    #define ConvertAngleToPos(angle) (uint16_t)((uint16_t)(angle) * 1088UL  / 360UL)
    #define ConvertPosToAngle(position) static_cast<float>((position) * 360.0  / 1088.0)
    #define ConvertTime(time) (uint16_t)(time * 10UL)
    #define ON 1
    #define OFF 0

    /** ALL_SERVO is ID to broadcast to all servo in bus. */
    #define ALL_SERVO 0xFE
    #define DEFAULT_ID  0x01

    #define G15_BAUDRATE 57600


    /*****************************
     * ERROR COD. G15 CUBE SERVO *
     *****************************/

    /**
      * @defgroup SERROR_GROUP Error Group
      * Defined to check error returned by servo (check as bit mask)
      * @{
      */
    #define SERROR_PING 0X0000
    // Return status:
    #define SERROR_INPUTVOLTAGE 0X0001
    #define SERROR_ANGLELIMIT 0X0002
    #define SERROR_OVERHEATING 0X0004
    #define SERROR_RANGE 0X0008
    #define SERROR_CHECKSUM 0X0010
    #define SERROR_OVERLOAD 0X0020
    #define SERROR_INSTRUCTION 0X0040
    // #define SERROR_ 0X0080
    #define SERROR_PACKETLOST 0X0100
    #define SERROR_WRONGHEADER 0X0200
    #define SERROR_IDMISMATCH 0X0400
    #define SERROR_CHECKSUMERROR 0X0800
    // #define SERROR_ 0X1000
    // #define SERROR_ 0X2000
    // #define SERROR_ 0X4000
    // #define SERROR_ 0X8000
    /**
      * @}
      */

    #define NUM_JOINT 1
    #define BUFFER_LEN 30

    #define EEMPROM_DELAY 25

}  // namespace JointHandlerConstants


class JointHandler {
    Timer control_loop_timer_;

    boolean hardwareSerial_;
    SoftwareSerial* G15Serial_;

    uint8_t txpin_shield_, rxpin_shield_, ctrlpin_shield_;
    
    uint16_t comunicatoin_error_;
 public:
    JointRHA joint_[NUM_JOINT];

    JointHandler() {}
    JointHandler(uint64_t timer);
    virtual void initJoints();
    void setSpeedGoal(SpeedGoal goal);
    void setTimer(uint64_t timer);

    virtual void controlLoop();

    void updateJointInfo();
    void updateJointErrorTorque();
    void sendJointTorques();

    void sendSetWheelModeAll();
    void sendExitWheelModeAll();

    void sendSetTorqueLimitAll(uint16_t torque_limit);
    void sendSetWheelSpeedAll(uint16_t speed, uint8_t direction);

    bool checkConectionAll();

    uint8_t addToSyncPacket(uint8_t *buffer, uint8_t *data);

    void sendSyncPacket(uint8_t instruction, uint8_t *buffer, uint8_t num_bytes, uint8_t num_servo);
    uint16_t sendSinglePacket(uint8_t instruction, uint8_t *buffer);


    /**************************************
     *        Serial Port Handling        *
     **************************************/
    JointHandler(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    JointHandler(uint8_t ctrlpin);
    void initSerial(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate = G15_BAUDRATE);

    void begin(uint32_t baudrate);
    void end(void);

    void setTxMode(void);
    void setRxMode(void);

};

#endif

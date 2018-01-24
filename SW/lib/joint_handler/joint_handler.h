/**
 * @file
 * @brief Implements JointHandler class. This object is in charge to sync all joints.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.h
 * @Last modified by:   enheragu
 * @Last modified time: 31_Oct_2017
 */

#ifndef JOINT_HANDLER_H
#define JOINT_HANDLER_H

#include "debug.h"
#include "rha_types.h"
#include "joint_rha.h"

#include <SoftwareSerial.h>
#include "Arduino.h"

#define __AVR_ATmega1280__

// Arduino mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define Serial_G15_lib Serial2
    // #define CHECK_MEGA_HARDWARESERIAL(rx, tx) (rx == 19 && tx == 18)
    #define CHECK_MEGA_HARDWARESERIAL(rx, tx) (rx == 17 && tx == 16)
    // #define CHECK_MEGA_HARDWARESERIAL(rx, tx) (rx == 15 && tx == 14)
    // Serial1 on pins 19 (RX) and 18 (TX)
    // Serial2 on pins 17 (RX) and 16 (TX)
    // Serial3 on pins 15 (RX) and 14 (TX)
// Arduino Leonardo
#elif defined (__AVR_ATmega32U4__)
    #define Serial_G15_lib Serial1
    #define CHECK_MEGA_HARDWARESERIAL(rx, tx) false
#else
    #define CHECK_MEGA_HARDWARESERIAL(rx, tx) false
    #define Serial_G15_lib Serial
#endif


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

    /** ALL_SERVO is ID to broadcast to all servo in bus. */
    #define ALL_SERVO 0xFE
    #define DEFAULT_ID  0x01

    #define G15_BAUDRATE 460800

    #define NUM_JOINT 1
    #define BUFFER_LEN 30

    #define EEMPROM_WRITE_DELAY 25
}  // namespace JointHandlerConstants


class JointHandler {
    RHATypes::Timer control_loop_timer_;

    uint8_t txpin_shield_, rxpin_shield_, ctrlpin_shield_;

    uint16_t comunication_error_;

 public:
    JointRHA joint_[NUM_JOINT];

    JointHandler() {}
    explicit JointHandler(uint64_t timer);
    virtual void initJoints();
    void setSpeedGoal(RHATypes::SpeedGoal _goal);
    void setTimer(uint64_t timer);

    virtual void controlLoop();

    void updateJointInfo();
    void updateJointErrorTorque();
    void sendJointTorques();

    void sendSetWheelModeAll();
    void sendExitWheelModeAll();

    void sendSetTorqueLimitAll(uint16_t _torque_limit);
    void sendSetWheelSpeedAll(uint16_t _speed = 0, uint8_t _direction = 0);

    void setReturnPacketOption(uint8_t _option);

    bool checkConectionAll();
    uint8_t addToSyncPacket(uint8_t *_buffer, uint8_t *_data, uint8_t _num_bytes);

    void warpSyncPacket(uint8_t *_buffer, uint8_t _adress, uint8_t *_txBuffer, uint8_t _num_bytes, uint8_t _num_servo);
    void warpSinglePacket(uint8_t _instruction, uint8_t *_buffer, uint8_t *_txBuffer);
    uint16_t sendPacket(uint8_t *_buffer);


    /**************************************
     *        Serial Port Handling        *
     **************************************/
    JointHandler(uint8_t _rxpin, uint8_t _txpin, uint8_t _ctrlpin);
    explicit JointHandler(uint8_t _ctrlpin);
    void initSerial(uint8_t _rxpin, uint8_t _txpin, uint8_t _ctrlpin, uint32_t _baudrate = G15_BAUDRATE);

    void begin(uint32_t _baudrate);
    void end(void);

    void setTxMode(void);
    void setRxMode(void);
};

#endif

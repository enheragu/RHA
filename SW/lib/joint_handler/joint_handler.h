/**
 * @file
 * @brief Implements JointHandler class. This object is in charge to sync all joints.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: joint_handler.h
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
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
    #define CW 1
    #define CCW 0
    #define ON 1
    #define OFF 0

    /** ALL_SERVO is ID to broadcast to all servo in bus. */
    #define ALL_SERVO 0xFE
    #define DEFAULT_ID  0x01

    #define G15_BAUDRATE 57600

    /**
      * @defgroup SREGISTER_GROUP Register Group
      * Register directions in servo memory for each parameter listed
      * @{
      */
    enum {
        MODEL_NUMBER_L,   // 0x00
        MODEL_NUMBER_H,   // 0x01
        VERSION,   // 0x02
        ID,   // 0x03
        BAUD_RATE,   // 0x04
        RETURN_DELAY_TIME,   // 0x05
        CW_ANGLE_LIMIT_L,   // 0x06
        CW_ANGLE_LIMIT_H,   // 0x07
        CCW_ANGLE_LIMIT_L,   // 0x08
        CCW_ANGLE_LIMIT_H,   // 0x09
        RESERVED1,   // 0x0A
        LIMIT_TEMPERATURE,   // 0x0B
        DOWN_LIMIT_VOLTAGE,   // 0x0C
        UP_LIMIT_VOLTAGE,   // 0x0D
        MAX_TORQUE_L,   // 0x0E
        MAX_TORQUE_H,   // 0x0F
        STATUS_RETURN_LEVEL,   // 0x10
        ALARM_LED,   // 0x11
        ALARM_SHUTDOWN,   // 0x12
        RESERVED2,   // 0x13
        DOWN_CALIBRATION_L,   // 0x14
        DOWN_CALIBRATION_H,   // 0x15
        UP_CALIBRATION_L,   // 0x16
        UP_CALIBRATION_H,   // 0x17
        TORQUE_ENABLE,   // 0x18
        LED,   // 0x19
        CW_COMPLIANCE_MARGIN,   // 0x1A
        CCW_COMPLIANCE_MARGIN,   // 0x1B
        CW_COMPLIANCE_SLOPE,   // 0x1C
        CCW_COMPLIANCE_SLOPE,   // 0x1D
        GOAL_POSITION_L,   // 0x1E
        GOAL_POSITION_H,   // 0x1F
        MOVING_SPEED_L,   // 0x20
        MOVING_SPEED_H,   // 0x21
        TORQUE_LIMIT_L,   // 0x22
        TORQUE_LIMIT_H,   // 0x23
        PRESENT_POSITION_L,   // 0x24
        PRESENT_POSITION_H,   // 0x25
        PRESENT_SPEED_L,   // 0x26
        PRESENT_SPEED_H,   // 0x27
        PRESENT_LOAD_L,   // 0x28
        PRESENT_LOAD_H,   // 0x29
        PRESENT_VOLTAGE,   // 0x2A
        PRESENT_TEMPERATURE,   // 0x2B
        REGISTERED_INSTRUCTION,  // 0x2C
        RESERVE3,  // 0x2D
        MOVING,  // 0x2E
        LOCK,  // 0x2F
        PUNCH_L,   // 0x30
        PUNCH_H   // 0x31
    };
    /**
      * @}
      */

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
}  // namespace JointHandlerConstants

class JointHandler {
    uint64_t time_last_, timer_;

    JointRHA joint_[JointHandlerConstants::NUM_JOINT];

    boolean hardwareSerial_;
    SoftwareSerial* G15Serial_;
    uint8_t txpin_shield_, rxpin_shield_, ctrlpin_shield_;
    uint16_t comunicatoin_error_;
    uint8_t buffer_[JointHandlerConstants::BUFFER_LEN];
    uint8_t buffer_send_[JointHandlerConstants::BUFFER_LEN];
    uint8_t servos_packet_;
    uint8_t bytes_write_;
 public:
    JointHandler();
    JointHandler(uint64_t timer);
    void initJoints();
    void setSpeedGoal(SpeedGoal goal);
    void setTimer(uint64_t timer);

    void updateJointInfo();
    void updateJointErrorTorque();

    uint8_t wrapSyncPacket();
    uint8_t wrapSinglePacket();

    void sendSyncPacket();

    /**************************************
     *        Serial Port Handling        *
     **************************************/
    JointHandler(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    JointHandler(uint8_t ctrlpin);
    void init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    void init(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate);

    void begin(uint32_t baudrate);
    void end(void);

    void setTxMode(void);
    void setRxMode(void);
    uint16_t sendPacket(uint8_t id, uint8_t instruction, uint8_t* data, uint8_t parameterLength);

    void init(uint64_t timer);

    void controlLoop();
};

#endif

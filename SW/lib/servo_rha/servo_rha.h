/**
 * @file
 * @brief Implements ServoRHA class. This object inherits from CytronG15Servo object to enhance its capabilities
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.h
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */


#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "debug.h"
#include "rha_types.h"

#include "Arduino.h"
#include <stdint.h>
// #include "cytron_g15_servo.h"

namespace ServoRHAConstants {
    #define DELAY1 500  // delay for configuration purposes
    #define TORQUE_CALIBRATION_INTERVAL 5  // In calibrations try every X torque (0, 5, 10, etc in case TORQUE_CALIBRATION_INTERVAL is 5)
    #define MIN_TORQUE_CALIBRATION 0
    #define MAX_TORQUE_CALIBRATION 800

    /** MARGIN_ANGLE_COMPARISON defines an interval in which two speed values will be considered as the same value when compared*/
    #define MARGIN_SPEED_COMPARISON 5
    /** MARGIN_ANGLE_COMPARISON defines an interval in which two angle values will be considered as the same value when compared*/
    #define MARGIN_ANGLE_COMPARISON 5

    #define RETURN_PACKET_ALL 0x02
    #define RETURN_PACKET_NONE 0x00
    #define RETURN_PACKET_READ_INSTRUCTIONS 0x01

    #define TORQUE_OFFSET 100  // under this torque servo does not move

    #define CW 1
    #define CCW 0

    #define ON 1
    #define OFF 0

    enum {  // enumeration for angle and speed compariso
        LESS_THAN,
        EQUAL,
        GREATER_THAN
        };


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

    namespace SpeedRegulatorK {
        #define KP 1.66
        #define KD 0
        #define KI 0
    }
}  // namespace ServoRHAConstants

uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin = 0);
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin = 0);

class ServoRHA {
 protected:
    uint8_t servo_id_;
    uint16_t speed_, speed_dir_, position_, load_, load_dir_, error_comunication_;
    uint8_t voltage_, temperature_;
    uint16_t goal_torque_;

 public:
    RHATypes::Regulator speed_regulator_;

 public:
    ServoRHA() {}
    ServoRHA(uint8_t servo_id);
    void init(uint8_t servo_id);
    void init();

    void updateInfo(uint8_t *data, uint16_t error);
    void setSpeedGoal(RHATypes::SpeedGoal goal);

    void addReturnOptionToPacket(uint8_t *buffer, uint8_t option);
    void addUpadteInfoToPacket(uint8_t *buffer);
    bool addTorqueToPacket(uint8_t *buffer);
    void setTorqueOnOfToPacket(uint8_t *buffer, uint8_t onOff);
    void setWheelModeToPacket(uint8_t *buffer);
    void exitWheelModeToPacket(uint8_t *buffer);
    void wheelModeToPacket(uint8_t *buffer, uint16_t CW_angle, uint16_t CCW_angle);
    void addToPacket(uint8_t *buffer, uint8_t *packet, uint8_t packet_len);
    void pingToPacket(uint8_t *buffer);

    void calculateTorque(float error, float derror = 0, float ierror = 0);


    /**********************************************************************
     *        Set of functions which imitates original G15 library        *
     **********************************************************************/
    void setTorqueLimitToPacket(uint8_t *buffer, uint16_t torque_limit);
    void setWheelSpeedToPacket(uint8_t *buffer, uint16_t torque_limit, uint8_t direction);

    /*************************************
     *        Interface functions        *
     *************************************/
    virtual uint8_t getID() { return servo_id_; }
    virtual uint16_t getSpeed() { return speed_; }
    virtual uint16_t getSpeedDir() { return speed_dir_; }
    virtual uint16_t getPosition() { return position_; }
    virtual uint16_t getLoad() { return load_; }
    virtual uint16_t getLoadDir() { return load_dir_; }
    virtual uint16_t getError() { return error_comunication_; }
    virtual uint8_t getVoltage() { return voltage_; }
    virtual uint8_t getTemperature() { return temperature_; }
    virtual uint16_t getGoalTorque() { return goal_torque_; }

};

#endif

/**
 * @file
 * @brief Implements ServoRHA class. This object inherits from CytronG15Servo object to enhance its capabilities
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.h
 * @Last modified by:   quique
 * @Last modified time: 30-Sep-2017
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

    #define TORQUE_OFFSET 80  // under this torque servo does not move
    #define TORQUE_PREALIMENTATION 1.2  // 1.666  // without load servo goes 60rpm faster with 100torque increment. It will be multiplied by speed_target_

    #define MAX_TORQUE_VALUE 1023
    #define MAX_SPEED_VALUE 65  // in rpm/min

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
        #define KP 10 //1.66
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

    uint8_t direction_target_;
    uint16_t speed_slope_, speed_target_;
    uint64_t time_last_;
    uint64_t time_last_error_;
    float error_, last_error_, derror_, ierror_;

 public:
    RHATypes::Regulator speed_regulator_;

 public:
    ServoRHA() { time_last_error_ = 0; time_last_ = 0; last_error_ = 0;
                error_ = 0; derror_ = 0; ierror_ = 0; }
    ServoRHA(uint8_t _servo_id);
    void init(uint8_t _servo_id);
    void init();

    void updateInfo(uint8_t *_data, uint16_t _error);

    void addReturnOptionToPacket(uint8_t *_buffer, uint8_t _option);
    void addUpadteInfoToPacket(uint8_t *_buffer);
    bool addTorqueToPacket(uint8_t *_buffer);
    void setTorqueOnOfToPacket(uint8_t *_buffer, uint8_t _onOff);
    void setWheelModeToPacket(uint8_t *_buffer);
    void exitWheelModeToPacket(uint8_t *_buffer);
    void wheelModeToPacket(uint8_t *_buffer, uint16_t _CW_angle, uint16_t _CCW_angle);
    void addToPacket(uint8_t *_buffer, uint8_t *_packet, uint8_t _packet_len);
    void pingToPacket(uint8_t *_buffer);

    uint8_t setSpeedGoal(RHATypes::SpeedGoal _goal);
    void speedError();
    void calculateTorque();


    /**********************************************************************
     *        Set of functions which imitates original G15 library        *
     **********************************************************************/
    void setTorqueLimitToPacket(uint8_t *_buffer, uint16_t _torque_limit);
    void setWheelSpeedToPacket(uint8_t *_buffer, uint16_t _torque_limit, uint8_t _direction);

    /*************************************
     *        Interface functions        *
     *************************************/
    virtual uint8_t getID() { return servo_id_; }
    virtual uint16_t getSpeed() { return speed_; }
    virtual uint16_t getSpeedDir() { return speed_dir_; }
    virtual uint16_t getPosition() { return position_; }
    virtual uint16_t getLoad() { return load_; }
    virtual uint16_t getLoadDir() { return load_dir_; }
    virtual uint16_t getCommError() { return error_comunication_; }
    virtual uint8_t getVoltage() { return voltage_; }
    virtual uint8_t getTemperature() { return temperature_; }
    virtual uint16_t getGoalTorque() { return goal_torque_; }
    uint16_t getSpeedTarget() { return speed_target_; }
    uint16_t getSpeedSlope() { return speed_slope_; }
    uint8_t getDirectionTarget() { return direction_target_; }
    float getError() { return error_; }
    float getDError() { return derror_; }
    float getIError() { return ierror_; }


};

#endif

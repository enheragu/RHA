/**
 * @file
 * @brief Implements ServoRHA class. This object inherits from CytronG15Servo object to enhance its capabilities
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.h
 * @Last modified by:   quique
 * @Last modified time: 17-Sep-2017
 */


#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "debug.h"
#include "Arduino.h"
#include <stdint.h>
#include "joint_handler.h"
#include "rha_types.h"
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

    #define MIN_TORQUE_CW 0
    #define MIN_TORQUE_CCW 180
    #define MAX_TORQUE_CW 400
    #define MAX_TORQUE_CCW 400
    #define ACCELERATION_ANGLE 360

    #define RETURN_PACKET_ALL 0x02
    #define RETURN_PACKET_NONE 0x00
    #define RETURN_PACKET_READ_INSTRUCTIONS 0x01


    /** KP K constant of speed control loop for servos. */
    #define KP 100/60  // means toruqe/speed
    #define TORQUE_OFFSET 150  // under this torque servo does not move

    enum {  // enumeration for angle and speed compariso
        LESS_THAN,
        EQUAL,
        GREATER_THAN
        };
}  // namespace ServoRHAConstants

uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin = 0);
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin = 0);

class ServoRHA {
 protected:
    uint8_t servo_id_;
    uint16_t min_torque_cw_, min_torque_ccw_, max_torque_cw_, max_torque_ccw_;  // minimum torque needed to move the servo and max torque allowed
    uint16_t speed_, speed_dir_, position_, load_, load_dir_, error_;
    uint8_t voltage_, temperature_, registered_, is_moving_;
    uint8_t goal_torque_[2];
    uint8_t returnPacketOption_[2];
    uint8_t instruction_, packetLength_;
    float kp_;

 public:
    ServoRHA() {}
    ServoRHA(uint8_t servo_id);
    void init(uint8_t servo_id);
    void init();

    void updateInfo(uint8_t *data);
    void setSpeedGoal(SpeedGoal goal);

    void addReturnOptionToPacket(uint8_t * &buffer, uint8_t option);
    void updateInfoToPacket(uint8_t * &buffer);
    void addTorqueToPacket(uint8_t * &buffer, uint8_t &bytes_write);
    void addToPacket(uint8_t * &buffer, uint8_t *packet, uint8_t packet_len, uint8_t &bytes_write);
    uint16_t setWheelSpeedPercent(uint16_t speed, uint8_t cw_ccw);

    uint16_t regulatorServo(uint16_t error);
    void setRegulatorKp(float kp);

    virtual void calibrateTorque();

    virtual uint8_t getID() { return servo_id_; }
    virtual uint16_t getSpeed() { return speed_; }
    virtual uint16_t getSpeedDir() { return speed_dir_; }
    virtual uint16_t getPosition() { return position_; }
    virtual uint16_t getLoad() { return load_; }
    virtual uint16_t getLoadDir() { return load_dir_; }
    virtual uint16_t getError() { return error_; }
    virtual uint8_t getVoltage() { return voltage_; }
    virtual uint8_t getTemperature() { return temperature_; }
    virtual uint8_t getRegistered() { return registered_; }
    virtual uint8_t getIsMoving() { return is_moving_; }

 protected:
     void calibrateTorqueDir(uint16_t &min_torque, uint16_t direction);
};

#endif

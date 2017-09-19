/**
 * @file
 * @brief Implements ServoRHA class. This object inherits from CytronG15Servo object to enhance its capabilities
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: servo_rha.h
 * @Last modified by:   enheragu
 * @Last modified time: 19_Sep_2017
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
    uint16_t speed_, speed_dir_, position_, load_, load_dir_, error_comunication_;
    uint8_t voltage_, temperature_;
    uint16_t goal_torque_;

    float kp_;

 public:
    ServoRHA() {}
    ServoRHA(uint8_t servo_id);
    void init(uint8_t servo_id);
    void init();

    void updateInfo(uint8_t *data, uint16_t error);
    void setSpeedGoal(SpeedGoal goal);

    void addReturnOptionToPacket(uint8_t *buffer, uint8_t option);
    void updateInfoToPacket(uint8_t *buffer);
    bool addTorqueToPacket(uint8_t *buffer, uint16_t speed = goal_torque_);
    void setTorqueOnOfToPacket(uint8_t buffer, uint8_t onOff);
    void setWheelModeToPacket(uint8_t *buffer);
    void exitWheelModeToPacket(uint8_t *buffer);
    void wheelModeToPacket(uint8_t *buffer, uint16_t CW_angle, uint16_t CCW_angle);
    void addToPacket(uint8_t *buffer, uint8_t *packet, uint8_t packet_len);


    void calculateTorque(uint16_t target_speed)
    uint16_t regulatorServo(float error);
    void setRegulatorKp(float kp);

    virtual uint8_t getID() { return servo_id_; }
    virtual uint16_t getSpeed() { return speed_; }
    virtual uint16_t getSpeedDir() { return speed_dir_; }
    virtual uint16_t getPosition() { return position_; }
    virtual uint16_t getLoad() { return load_; }
    virtual uint16_t getLoadDir() { return load_dir_; }
    virtual uint16_t getError() { return error_comunication_; }
    virtual uint8_t getVoltage() { return voltage_; }
    virtual uint8_t getTemperature() { return temperature_; }

};

#endif

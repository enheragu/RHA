#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "debug.h"
#include "cytron_g15_servo.h"

#define DELAY1 500  // delay for configuration purposes
#define TORQUE_CALIBRATION_INTERVAL 5  // In calibrations try every X torque (0, 5, 10, etc in case TORQUE_CALIBRATION_INTERVAL is 5)
#define MIN_TORQUE_CALIBRATION 0
#define MAX_TORQUE_CALIBRATION 800

#define MARGIN_SPEED_COMPARISON 5
#define MARGIN_ANGLE_COMPARISON 5  // configure an interval in which angles will be considered as the same value when compared

#define MIN_TORQUE_CW 0
#define MIN_TORQUE_CCW 180
#define MAX_TORQUE_CW 400
#define MAX_TORQUE_CCW 400
#define ACCELERATION_ANGLE 360

#define RETURN_PACKET_ALL 0x02
#define RETURN_PACKET_NONE 0x00
#define RETURN_PACKET_READ_INSTRUCTIONS 0x01

#define KP 100/60 // means toruqe/speed

#define ALL_SERVO 0xFE

enum {  // enumeration for angle and speed compariso
    LESS_THAN,
    EQUAL,
    GREATER_THAN
    };

uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin = 0);
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin = 0);

class ServoRHA : public Cytron_G15_Servo {
 protected:
    uint16_t min_torque_cw_, min_torque_ccw_, max_torque_cw_, max_torque_ccw_;  // minimum torque needed to move the servo and max torque allowed
    uint16_t speed_, position_;
 public:
    ServoRHA(){}  // It'll be only used for testing purposes
    ServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
    virtual void init(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, uint32_t baudrate);
    virtual void init();

    uint16_t angleRead();
    uint16_t speedRead();
    bool isMoving();
    void updateInfo();

    virtual uint16_t returnPacketSet(uint8_t option);
    void addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo);
    uint8_t wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo);
    virtual uint16_t setWheelSpeed(uint16_t speed, uint8_t cw_ccw);

    uint16_t regulator(uint16_t error);

    virtual void calibrateTorque();

 protected:
     void calibrateTorqueDir(uint16_t &min_torque, uint16_t direction);
};

#endif

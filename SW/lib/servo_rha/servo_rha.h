#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "debug.h"
#include "Cytron_G15Shield.h"

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

// Acting as an encoder it needs an interval to check if it made or not a round.
// This has to be defined having into account time spent in the rest of the program.
// It has to go through doNext function before the servo leaves this interval [-ENCODER_MARGIN, +ENCODER_MARGIN]
// If it does not, a encoder round will be lost
#define ENCODER_MARGIN 10

// Compliance margin is the allowable goal position error margin
#define COMPLIANCE_MARGIN 5

#define ALL_SERVO 0xFE

enum {   LESS_THAN,  // enumeration for angle comparison
      EQUAL,
      GREATER_THAN
    };

uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin = 0);
uint8_t compareSpeed(uint16_t speed1, uint16_t speed2, uint8_t speed_margin = 0);

class ServoRHA : public Cytron_G15Shield {
 protected:
  uint16_t min_torque_cw_, min_torque_ccw_, max_torque_cw_, max_torque_ccw_;  // minimum torque needed to move the servo
  float acceleration_slope_, acceleration_angle_;
  uint8_t flag_moving_, flag_accelerating_, flag_decelerating_, flag_first_time_accel_decel_;
  uint16_t current_pose_, goal_direction_, init_pose_, encoder_current_, encoder_flag_;
  float goal_pose_encoder_;

 public:
  ServoRHA() {  }  // It'll be only used for testing purposes
  ServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
  virtual void initServo();

  uint16_t angleRead();
  uint16_t speedRead();

  virtual uint16_t returnPacketSet(uint8_t option);
  void addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo);
  uint8_t wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo);
  virtual uint16_t setWheelSpeed(uint16_t speed, uint8_t cw_ccw);

  void setGoalEncoder(float goal_rotation, uint8_t cw_ccw);
  void doNext();
  void encoderModeRotation();
  void accelerate(uint16_t &speed);
  void decelerate(uint16_t &speed, uint16_t angle_left);

  bool isMoving();

  virtual void calibrateTorque();

 protected:
  void calibrateTorqueDir(uint16_t &min_torque, uint16_t direction);
};

#endif

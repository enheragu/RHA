#ifndef SERVO_RHA_H
#define SERVO_RHA_H

#include "Cytron_G15Shield.h"

#define DELAY1 500 //delay for configuration purposes
#define TORQUE_CALIBRATION_INTERVAL 10 //In calibrations try every X torque (0, 5, 10, etc in case TORQUE_CALIBRATION_INTERVAL is 5)
#define ANGLE_MARGIN 0 // configure an interval in which angles will be considered as the same value when compared

#define MIN_TORQUE_CW 0
#define MIN_TORQUE_CCW 180
#define MAX_TORQUE_CW 1023
#define MAX_TORQUE_CCW 1023
#define ACCELERATION_ANGLE 180

#define RETURN_PACKET_ALL 0x02
#define RETURN_PACKET_NONE 0x00
#define RETURN_PACKET_READ_INSTRUCTIONS 0x01

//Acting as an encoder it needs an interval to check if it made or not a round.
//This has to be defined having into account time spent in the rest of the program.
//It has to go through doNext function before the servo leaves this interval [-ENCODER_MARGIN, +ENCODER_MARGIN]
//If it does not, a encoder round will be lost
#define ENCODER_MARGIN 10

//Compliance margin is the allowable goal position error margin
#define COMPLIANCE_MARGIN 5

#define ALL_SERVO 0xFE

enum { LESS_THAN, //enumeration for angle comparison
      EQUAL,
      GREATER_THAN
    };

/******************************************
 *       Debugging macro definition       *
 ******************************************/

#ifdef DEBUG_SERVO_RHA
  #define DebugSerialSRHALn(a) {Serial.print("[-]  "); Serial.println(a);}
  #define DebugSerialSRHALn2(a,b) {Serial.print("[-]  ");Serial.print(a); Serial.println(b);}
  #define DebugSerialSRHALn4(a,b,c,d) {Serial.print("[-]  ");Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d);}
#else
  #define DebugSerialSRHALn(a)
  #define DebugSerialSRHALn2(a,b)
  #define DebugSerialSRHALn4(a,b,c,d)
#endif

uint8_t compareAngles(uint16_t angle1, uint16_t angle2, uint8_t angle_margin=0);

class ServoRHA : public Cytron_G15Shield {
protected:
  uint16_t min_torque_cw_, min_torque_ccw_, max_torque_cw_, max_torque_ccw_; //minimum torque needed to move the servo
  float acceleration_slope_, acceleration_angle_;
  uint8_t flag_moving_;
  uint16_t current_pose_, goal_pose_encoder_, goal_direction_, init_pose_, encoder_current_, encoder_flag_;
public:
  ServoRHA(){} // It'll be only used for testing purposes
  ServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin);
  uint16_t angleRead();
  virtual uint16_t returnPacketSet(uint8_t option);
  void addToPacket(uint8_t *buffer, uint8_t &position, uint8_t *goal, uint8_t goal_len, uint8_t &num_servo);
  uint8_t wrapPacket(uint8_t *buffer, uint8_t *data, uint8_t data_len, uint8_t instruction, uint8_t num_servo);
  virtual uint16_t setWheelSpeed(uint16_t speed, uint8_t cw_ccw);

  void setGoalEncoder(float goal_rotation, uint8_t cw_ccw);
  void doNext();
  void encoderModeRotation();
private:
  virtual void calibrateTorque();
  void calibrateTorqueDir(uint16_t &min_torque, uint16_t direction);
};

#endif

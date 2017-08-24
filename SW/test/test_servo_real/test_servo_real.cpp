#include "servo_rha.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 1
#define DELAY_MOVE 5000
#define POSITION 5.30
#define SPEED 300
#define BAUDRATE 19200.
#define MARGIN 5

class TestServoRHA : public ServoRHA {
public:
  uint16_t g15_speed;
  uint8_t direction;
public:
  TestServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):
    ServoRHA(servo_id, rxpin, txpin, ctrlpin){}

  uint8_t getServoID(){return _servo_id;}
  uint8_t getTxPin(){return _txpin;}
  uint8_t getRxPin(){return _rxpin;}
  uint8_t getCtrlPin(){return _ctrlpin;}

  uint16_t getMinTorqueCw(){return min_torque_cw_;}
  uint16_t getMinTorqueCCw(){return min_torque_ccw_;}
  uint16_t getMaxTorqueCw(){return max_torque_cw_;}
  uint16_t getMaxTorqueCCw(){return max_torque_ccw_;}
  float getAccelerationSlope(){return acceleration_slope_;}
  float getAccelerationAngle(){return acceleration_angle_;}
  uint8_t getFlagMoving(){return flag_moving_;}
  uint16_t geturrentPose(){return current_pose_;}
  float getGoalPoseEncoder(){return goal_pose_encoder_;}
  uint16_t getGoalDirection(){return goal_direction_;}
  uint16_t getInitPose(){return init_pose_;}
  uint16_t getEncoderCurrent(){return encoder_current_;}
  uint16_t getEncoderFlag(){return encoder_flag_;}
};

void test_function_setGoalEncoder(void) {
  TestServoRHA servo_test1(1, 2, 3, 8);
  servo_test1.setGoalEncoder(8.56, CCW);

  TEST_ASSERT_EQUAL_FLOAT(8.56,servo_test1.getGoalPoseEncoder());
  TEST_ASSERT_EQUAL_UINT8(CCW,servo_test1.getGoalDirection());
}

void test_function_encoderModeRotation(void){

}


void process() {
  UNITY_BEGIN();

  RUN_TEST(test_function_encoderModeRotation);
  //RUN_TEST();
  //RUN_TEST();
  //RUN_TEST();
  UNITY_END();
}


void setup () {
  delay(3000);
  process();
}

void loop () {
  // some code...
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(500);
}

#endif

#include "servo_rha.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 1
#define DELAY_MOVE 5000
#define DELAY_MOVE_SHORT 500
#define POSITION 8.56
#define ANGLE_POSITION 180
#define SPEED 300
#define BAUDRATE 19200
#define MARGIN 5
#define MAX_SPEED_ALLOWED 600
#define MARGIN_SPEED_COMPARISON 5

#ifdef DEBUG_TEST_SERVO_RHA
  #define DebugSerialTSRHALn(a) {Serial.println(a);}
  #define DebugSerialTSRHA(a) {Serial.print(a);}
#else
  #define DebugSerialTSRHALn(a)
  #define DebugSerialTSRHA(a)
#endif

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
  uint16_t getCurrentPose(){return current_pose_;}
  float getGoalPoseEncoder(){return goal_pose_encoder_;}
  uint16_t getGoalDirection(){return goal_direction_;}
  uint16_t getInitPose(){return init_pose_;}
  uint16_t getEncoderCurrent(){return encoder_current_;}
  uint16_t getEncoderFlag(){return encoder_flag_;}

  void setInitPose(uint16_t pose) {init_pose_ = pose;}
  void setCurrentPose(uint16_t pose) {current_pose_ = pose;}
};

void test_function_setGoalEncoder(void) {
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  //init servo not needeed in this case
  servo_test1.setGoalEncoder(POSITION, CCW);
  TEST_ASSERT_EQUAL_FLOAT(POSITION, servo_test1.getGoalPoseEncoder());
  TEST_ASSERT_EQUAL_UINT8(CCW, servo_test1.getGoalDirection());
}

void test_function_isMoving(void){
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(false, servo_test1.isMoving());
  servo_test1.setWheelMode();
  servo_test1.setWheelSpeed(MAX_SPEED_ALLOWED,CW);
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(true, servo_test1.isMoving());
  servo_test1.exitWheelMode();
}

void test_function_readAngle(void){
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();
  servo_test1.exitWheelMode();
  servo_test1.setSpeed(SPEED,iWRITE_DATA);
  servo_test1.setPosAngle(ANGLE_POSITION,iWRITE_DATA);
  while (servo_test1.isMoving()) delay(DELAY_MOVE_SHORT);
  TEST_ASSERT_EQUAL_UINT16(ANGLE_POSITION, servo_test1.angleRead());
}


TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
uint16_t real_speed_low, real_speed_high, speed=1;
uint8_t data[10];

void configureServoForAccelDecelTest(){
  DebugSerialTSRHA("Begin of configuration of servo for acceleration and deceleration test.");
  servo_test1.initServo();
  servo_test1.setWheelMode();
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(0, servo_test1.speedRead());

  uint16_t real_speed_low, real_speed_high, speed=1;
  uint8_t data[10];
  servo_test1.setWheelSpeed(servo_test1.getMinTorqueCw(),CW);
  delay(DELAY_MOVE);
  real_speed_low = servo_test1.speedRead();
  servo_test1.setWheelSpeed(servo_test1.getMaxTorqueCw(),CW);
  delay(DELAY_MOVE);
  real_speed_high = servo_test1.speedRead();
  DebugSerialTSRHA("End of configuration of servo for acceleration and deceleration test.");
}

void test_function_accelerateCW(void){
  DebugSerialTSRHA("Begin of acceleration test");
  servo_test1.setWheelSpeed(speed,CW);
  servo_test1.setGoalEncoder(0, CW); //want to set goal direction
  servo_test1.setInitPose(servo_test1.angleRead());
  servo_test1.setCurrentPose(servo_test1.angleRead());

  while(1){
    DebugSerialTSRHA("Loop on acceleration test. Accelerating.");
    servo_test1.accelerate(speed);
    delay(DELAY_MOVE);
    servo_test1.setCurrentPose(servo_test1.angleRead());
    if (compareSpeed(servo_test1.speedRead(), real_speed_high, MARGIN_SPEED_COMPARISON)==EQUAL) break;
  }
  TEST_ASSERT_EQUAL(EQUAL, compareSpeed(servo_test1.speedRead(), real_speed_high, MARGIN_SPEED_COMPARISON));
  servo_test1.exitWheelMode();
  DebugSerialTSRHA("End of acceleration test");
}

void test_function_decelerateCW(void){
  DebugSerialTSRHA("Begin of deceleration test");
  servo_test1.setWheelSpeed(servo_test1.getMaxTorqueCw(),CW);
  servo_test1.setGoalEncoder(0, CW); //want to set goal direction
  servo_test1.setInitPose(servo_test1.angleRead());
  servo_test1.setCurrentPose(servo_test1.angleRead());

  while(1){
    DebugSerialTSRHA("Loop on deceleration test. Decelerating.");
    uint16_t angle_left = servo_test1.getAccelerationAngle() - (servo_test1.getCurrentPose() - servo_test1.getInitPose());
    servo_test1.decelerate(speed, angle_left);
    delay(DELAY_MOVE);
    servo_test1.setCurrentPose(servo_test1.angleRead());
    if (compareSpeed(servo_test1.speedRead(), 0, MARGIN_SPEED_COMPARISON)==EQUAL) break;
  }
  TEST_ASSERT_EQUAL(EQUAL, compareSpeed(servo_test1.speedRead(), 0, MARGIN_SPEED_COMPARISON));
  servo_test1.exitWheelMode();
  DebugSerialTSRHA("End of deceleration test");

}

void test_function_encoderModeRotation(void){

}


void process() {
  UNITY_BEGIN();

  RUN_TEST(test_function_setGoalEncoder);
  RUN_TEST(test_function_isMoving);
  RUN_TEST(test_function_readAngle);

  configureServoForAccelDecelTest();
  RUN_TEST(test_function_accelerateCW);
  RUN_TEST(test_function_decelerateCW);
  //RUN_TEST(test_function_encoderModeRotation);
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

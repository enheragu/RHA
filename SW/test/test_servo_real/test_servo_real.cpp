#include "debug.h"
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

class TestServoRHA : public ServoRHA {
 public:
  uint16_t g15_speed;
  uint8_t direction;

 public:
  TestServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin):
    ServoRHA(servo_id, rxpin, txpin, ctrlpin) {  }

  // Init servo is overwritten to be able to test calibrateTorque() function apart
  virtual void initServo() {
    DebugSerialSRHALn("ServoRHA::initServo: begin of inicialitationfunction");

    Cytron_G15Shield::begin(19200);
    delay(DELAY1);
    // calibrateTorque();

    max_torque_ccw_ = MAX_TORQUE_CCW;
    max_torque_cw_ = MAX_TORQUE_CW;
    acceleration_angle_ = ACCELERATION_ANGLE;
    flag_moving_ = false;
    current_pose_ = 0; goal_pose_encoder_ = 0; init_pose_ = 0; encoder_current_ = 0;
    acceleration_slope_ = (static_cast<float>100 - static_cast<float>0) / static_cast<float>acceleration_angle_;
    flag_accelerating_ = false;
    flag_decelerating_ = false;
    flag_first_time_accel_decel_ = true;

    returnPacketSet(RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions

    DebugSerialSRHALn("ServoRHA::initServo: end of inicialitation function");
  }

  uint8_t getServoID() {  return _servo_id; }
  uint8_t getTxPin() {  return _txpin; }
  uint8_t getRxPin() {  return _rxpin; }
  uint8_t getCtrlPin() {  return _ctrlpin; }

  uint16_t getMinTorqueCw() {  return min_torque_cw_; }
  uint16_t getMinTorqueCcw() {  return min_torque_ccw_; }
  uint16_t getMaxTorqueCw() {  return max_torque_cw_; }
  uint16_t getMaxTorqueCcw() {  return max_torque_ccw_; }
  float getAccelerationSlope() {  return acceleration_slope_; }
  float getAccelerationAngle() {  return acceleration_angle_; }
  uint8_t getFlagMoving() {  return flag_moving_; }
  uint16_t getCurrentPose() {  return current_pose_; }
  float getGoalPoseEncoder() {  return goal_pose_encoder_; }
  uint16_t getGoalDirection() {  return goal_direction_; }
  uint16_t getInitPose() {  return init_pose_; }
  uint16_t getEncoderCurrent() {  return encoder_current_; }
  uint16_t getEncoderFlag() {  return encoder_flag_; }
  uint8_t getFlagAccelerating() {  return flag_accelerating_; }
  uint8_t getFlagDecelerating() {  return flag_decelerating_; }

  void setInitPose(uint16_t pose) {  init_pose_ = pose; }
  void setCurrentPose(uint16_t pose) {  current_pose_ = pose; }
  void setFlagFirstTimeAccDecel(uint8_t option) {  flag_first_time_accel_decel_ =  option; }
  void setFlagAccelerating(uint8_t option) {  flag_accelerating_ = option; }
  void setFlagDecelerating(uint8_t option) {  flag_decelerating_ = option; }
};

void test_function_setGoalEncoder(void) {
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  // init servo not needeed in this case
  servo_test1.setGoalEncoder(POSITION, CCW);
  TEST_ASSERT_EQUAL_FLOAT(POSITION, servo_test1.getGoalPoseEncoder());
  TEST_ASSERT_EQUAL_UINT8(CCW, servo_test1.getGoalDirection());
}

void test_function_setInitServo(void) {
  DebugSerialTSRHARealLn("Begin of init test for servo real.");
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();

  TEST_ASSERT_EQUAL_UINT16(MAX_TORQUE_CCW, servo_test1.getMaxTorqueCcw());
  TEST_ASSERT_EQUAL_UINT16(MAX_TORQUE_CW, servo_test1.getMaxTorqueCw());
  TEST_ASSERT_EQUAL_UINT16(ACCELERATION_ANGLE, servo_test1.getAccelerationAngle());
  uint16_t acceleration_slope = (static_cast<float>100 - static_cast<float>0) / static_cast<float>servo_test1.getAccelerationAngle();
  TEST_ASSERT_EQUAL_UINT16(acceleration_slope, servo_test1.getAccelerationSlope());
  DebugSerialTSRHARealLn("End of init test for servo real.");
}

void test_function_isMoving(void) {
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(false, servo_test1.isMoving());
  servo_test1.setWheelMode();
  servo_test1.setWheelSpeed(100, CW);
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(true, servo_test1.isMoving());
  servo_test1.exitWheelMode();
}

void test_function_readAngle(void) {
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();
  servo_test1.exitWheelMode();
  servo_test1.setSpeed(SPEED, iWRITE_DATA);
  servo_test1.setPosAngle(ANGLE_POSITION, iWRITE_DATA);
  while (servo_test1.isMoving()) delay(DELAY_MOVE_SHORT);
  TEST_ASSERT_EQUAL(EQUAL, compareAngles(ANGLE_POSITION, servo_test1.angleRead(), MARGIN_ANGLE_COMPARISON));
}

void test_function_calibrateTorque(void) {
  DebugSerialTSRHARealLn("test_calibrateTorque: Begin of test.");
  TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
  servo_test1.initServo();
  DebugSerialTSRHARealLn("test_calibrateTorque: Calling calibrateTorque.");
  servo_test1.calibrateTorque();
  DebugSerialTSRHARealLn2("test_calibrateTorque: min_torque_cw_ is now: ", servo_test1.getMinTorqueCw());
  DebugSerialTSRHARealLn2("test_calibrateTorque: min_torque_ccw_ is now: ", servo_test1.getMinTorqueCcw());
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(false, servo_test1.isMoving());
  servo_test1.setWheelMode();
  servo_test1.setWheelSpeed(100, CW);
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL_MESSAGE(true, servo_test1.isMoving(), "Servo should be moving, speed was set to MAX_SPEED_ALLOWED");

  servo_test1.setWheelSpeed(0, CW);  // servo stoped
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL_MESSAGE(false, servo_test1.isMoving(), "Servo should not be moving, speed was set under min_torque (CW dir)");

  servo_test1.setWheelSpeed(1, CW);  // servo min speed
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL_MESSAGE(true, servo_test1.isMoving(), "Servo should be moving, speed was set over min_torque (CW dir)");

  servo_test1.setWheelSpeed(0, CCW);  // servo stoped
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL_MESSAGE(false, servo_test1.isMoving(), "Servo should not be moving, speed was set under min_torque (CCW dir)");

  servo_test1.setWheelSpeed(1, CCW);  // servo min speed
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL_MESSAGE(true, servo_test1.isMoving(), "Servo should be moving, speed was set over min_torque (CCW dir)");
  DebugSerialTSRHARealLn("test_calibrateTorque: End of test.");

  servo_test1.exitWheelMode();
}


TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
uint16_t real_speed_low_cw, real_speed_high_cw, speed = 1;
uint8_t data[10];

void configureServoForAccelDecelTestCw() {
  DebugSerialTSRHARealLn("Begin of configuration of servo for acceleration and deceleration test.");
  servo_test1.initServo();
  servo_test1.calibrateTorque();  // in original this function would go inside initServo()
  servo_test1.setWheelMode();
  delay(DELAY_MOVE);
  TEST_ASSERT_EQUAL(0, servo_test1.speedRead());

  servo_test1.setWheelSpeed(1, CW);  // set speed to minimum
  delay(DELAY_MOVE);
  real_speed_low_cw = servo_test1.speedRead();
  servo_test1.setWheelSpeed(100, CW);  // set speed to 100%
  delay(DELAY_MOVE);
  real_speed_high_cw = servo_test1.speedRead();
  DebugSerialTSRHARealLn("End of configuration of servo for acceleration and deceleration test.");
}

void test_function_accelerateCW(void) {
  DebugSerialTSRHARealLn("test_accelerateCW: Begin of test");
  servo_test1.setWheelSpeed(1, CW);
  servo_test1.setGoalEncoder(0, CW);  // want to set goal direction
  servo_test1.setInitPose(servo_test1.angleRead());
  servo_test1.setCurrentPose(servo_test1.angleRead());
  servo_test1.setFlagFirstTimeAccDecel(true);
  servo_test1.setFlagAccelerating(true);
  uint16_t counter = 0;

  while (servo_test1.getFlagAccelerating() == true && TEST_PROTECT()) {
    DebugSerialTSRHARealLn("test_accelerateCW: Loop on acceleration test. Accelerating.");
    servo_test1.accelerate(speed);
    servo_test1.setCurrentPose(servo_test1.angleRead());
    DebugSerialTSRHARealLn2("test_accelerateCW: init_pose_ is: ", servo_test1.getInitPose());
    DebugSerialTSRHARealLn2("test_accelerateCW: current_pose_ is: ", servo_test1.getCurrentPose());
    uint8_t speed_test = abs(servo_test1.getCurrentPose() - servo_test1.getInitPose())*servo_test1.getAccelerationSlope();
    DebugSerialTSRHARealLn2("test_accelerateCW: speed is: ", speed_test);

    counter++;
    if (counter > 500) TEST_ABORT();
  }
  TEST_ASSERT_EQUAL(EQUAL, compareSpeed(servo_test1.speedRead(), real_speed_high_cw, MARGIN_SPEED_COMPARISON));
  TEST_ASSERT_EQUAL(false, servo_test1.getFlagAccelerating());
  servo_test1.exitWheelMode();
  DebugSerialTSRHARealLn("test_accelerateCW: End of test");
}

void test_function_decelerateCW(void) {
  DebugSerialTSRHARealLn("test_decelerateCW: Begin of test");
  servo_test1.setWheelSpeed(100, CW);
  servo_test1.setGoalEncoder(0, CW);  // want to set goal direction
  servo_test1.setInitPose(servo_test1.angleRead());
  servo_test1.setCurrentPose(servo_test1.angleRead());
  servo_test1.setFlagFirstTimeAccDecel(true);
  servo_test1.setFlagDecelerating(true);
  uint16_t counter = 0;

  while (servo_test1.getFlagDecelerating() == true && TEST_PROTECT()) {
    DebugSerialTSRHARealLn("test_decelerateCW: Loop on deceleration test. Decelerating.");
    uint16_t angle_left = servo_test1.getAccelerationAngle() - abs((servo_test1.getCurrentPose() - servo_test1.getInitPose()));
    servo_test1.decelerate(speed, angle_left);
    DebugSerialTSRHARealLn2("test_decelerateCW: Angle left is now: ", angle_left);
    DebugSerialTSRHARealLn4("test_decelerateCW: Speed set to: ", speed, "% of max speed", ".");
    DebugSerialTSRHARealLn2("test_decelerateCW: Real speed is now: ", servo_test1.speedRead());
    delay(DELAY_MOVE);
    servo_test1.setCurrentPose(servo_test1.angleRead());

    counter++;
    if (counter > 500) TEST_ABORT();
  }

  DebugSerialTSRHARealLn("test_decelerateCW: Deceleration ended");
  DebugSerialTSRHARealLn2("test_decelerateCW: Real speed is now: ", servo_test1.speedRead());
  TEST_ASSERT_EQUAL(EQUAL, compareSpeed(servo_test1.speedRead(), 0, MARGIN_SPEED_COMPARISON));
  TEST_ASSERT_EQUAL(false, servo_test1.getFlagDecelerating());
  servo_test1.exitWheelMode();
  DebugSerialTSRHARealLn("test_decelerateCW: End of test");
}

void test_function_encoderModeRotation(void) {
}


void process() {
  UNITY_BEGIN();

  // RUN_TEST(test_function_setGoalEncoder);
  // RUN_TEST(test_function_setInitServo);
  // RUN_TEST(test_function_isMoving);
  // RUN_TEST(test_function_readAngle);
  // RUN_TEST(test_function_calibrateTorque);

  configureServoForAccelDecelTestCw();
  RUN_TEST(test_function_accelerateCW);
  RUN_TEST(test_function_decelerateCW);
  // RUN_TEST(test_function_encoderModeRotation);
  // RUN_TEST();
  // RUN_TEST();
  // RUN_TEST();
  // RUN_TEST();
  // RUN_TEST();
  UNITY_END();
}


void setup() {
  delay(3000);
  process();
}

void loop() {
  // some code...
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(500);
}

#endif

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

        Cytron_G15_Servo::begin(19200);
        delay(DELAY1);
        // calibrateTorque();

        max_torque_ccw_ = MAX_TORQUE_CCW;
        max_torque_cw_ = MAX_TORQUE_CW;
        acceleration_angle_ = ACCELERATION_ANGLE;
        flag_moving_ = false;
        current_pose_ = 0; goal_pose_encoder_ = 0; init_pose_ = 0; encoder_current_ = 0;
        acceleration_slope_ = (static_cast<float>(100) - static_cast<float>(0)) / static_cast<float>(acceleration_angle_);
        flag_accelerating_ = false;
        flag_decelerating_ = false;
        flag_first_time_accel_decel_ = true;

        returnPacketSet(RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions

        DebugSerialSRHALn("ServoRHA::initServo: end of inicialitation function");
    }

    uint8_t getServoID() {  return servo_id_; }
    uint8_t getTxPin() {  return txpin_; }
    uint8_t getRxPin() {  return rxpin_; }
    uint8_t getCtrlPin() {  return ctrlpin_; }

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


void test_function_setInitServo(void) {
    DebugSerialTSRHARealLn("Begin of init test for servo real.");
    TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
    servo_test1.initServo();

    TEST_ASSERT_EQUAL_UINT16(MAX_TORQUE_CCW, servo_test1.getMaxTorqueCcw());
    TEST_ASSERT_EQUAL_UINT16(MAX_TORQUE_CW, servo_test1.getMaxTorqueCw());
    TEST_ASSERT_EQUAL_UINT16(ACCELERATION_ANGLE, servo_test1.getAccelerationAngle());
    uint16_t acceleration_slope = (static_cast<float>(100) - static_cast<float>(0)) / static_cast<float>(servo_test1.getAccelerationAngle());
    TEST_ASSERT_EQUAL_UINT16(acceleration_slope, servo_test1.getAccelerationSlope());
    DebugSerialTSRHARealLn("End of init test for servo real.");
}

void test_function_isMoving(void) {
    DebugSerialTSRHARealLn("Begin of isMoving test for servo real.");
    TestServoRHA servo_test1(SERVO_ID, 2, 3, 8);
    servo_test1.initServo();
    delay(DELAY_MOVE);
    TEST_ASSERT_EQUAL(false, servo_test1.isMoving());
    servo_test1.setWheelMode();
    servo_test1.setWheelSpeed(100, CW);
    delay(DELAY_MOVE);
    TEST_ASSERT_EQUAL(true, servo_test1.isMoving());
    servo_test1.exitWheelMode();
    DebugSerialTSRHARealLn("End of isMoving test for servo real.");
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

void process() {
    UNITY_BEGIN();

    RUN_TEST(test_function_setGoalEncoder);
    RUN_TEST(test_function_setInitServo);
    RUN_TEST(test_function_isMoving);
    RUN_TEST(test_function_readAngle);
    RUN_TEST(test_function_calibrateTorque);

    // RUN_TEST();
    // RUN_TEST();
    // RUN_TEST();
    // RUN_TEST();
    // RUN_TEST();
    UNITY_END();
}


void setup() {
    Serial.begin(9600);
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

#include "debug.h"
#include "servo_rha.h"
#include "unity.h"

#ifdef UNIT_TEST

/**
  * This set of tests are intended to test ServoRHA functionalities that doesn't depend on
  * the servo itself. TestServoRHA is a redefinition of ServoRHA so it does not need to
  * use Cytron functions.
  */


#define MIN_TORQUE_CW 0
#define MIN_TORQUE_CCW 180
#define MAX_TORQUE_CW 1023
#define MAX_TORQUE_CCW 1023
#define ACCELERATION_ANGLE 180

class TestServoRHA : public ServoRHA {
 public:
  uint16_t g15_speed;
  uint8_t direction;

  TestServoRHA(uint8_t servo_id, uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin) {
      _servo_id = servo_id;
      _rxpin = rxpin;
      _txpin = txpin;
      _ctrlpin = ctrlpin;
  }

  virtual void initServo() {
      // This constructor is copied from servo_rha so it uses the overwritten funcions from TestServoRHA
      TestServoRHA::begin(19200);
      delay(DELAY1);
      TestServoRHA::calibrateTorque();

      max_torque_ccw_ = MAX_TORQUE_CCW;
      max_torque_cw_ = MAX_TORQUE_CW;
      acceleration_angle_ = ACCELERATION_ANGLE;
      flag_moving_ = false;
      current_pose_ = 0; goal_pose_encoder_ = 0; init_pose_ = 0; encoder_current_ = 0;
      acceleration_slope_ = (static_cast<float>100 - static_cast<float>0) / static_cast<float>acceleration_angle_;

      TestServoRHA::returnPacketSet(RETURN_PACKET_READ_INSTRUCTIONS);  // Servo only respond to read data instructions
  }
  virtual void calibrateTorque() {
      min_torque_cw_ = MIN_TORQUE_CW;
      min_torque_ccw_ = MIN_TORQUE_CCW;
  }

  virtual uint16_t returnPacketSet(uint8_t option) {
      return option;
  }

  virtual uint16_t SetWheelSpeed(uint16_t speed, uint8_t cw_ccw) {
      g15_speed = -1;
      direction = cw_ccw;
      if (cw_ccw == CW) g15_speed = (uint16_t)(map(speed, 0, 100, min_torque_cw_, max_torque_cw_));
      else if (cw_ccw == CCW) g15_speed = (uint16_t)(map (speed, 0, 100, min_torque_ccw_, max_torque_ccw_));
      DebugSerialTSRHA("Speed passed was: "); DebugSerialTSRHA(speed); DebugSerialTSRHA(". Speed for servo is: "); DebugSerialTSRHALn(g15_speed);
      return 0;
  }

  virtual void begin(uint32_t baudrate) {
      return;
  }
};

void test_function_compareAngles(void) {
    TEST_ASSERT_EQUAL(GREATER_THAN, compareAngles(6, 5));
    TEST_ASSERT_EQUAL(EQUAL, compareAngles(6, 5, 2));
    TEST_ASSERT_EQUAL(LESS_THAN, compareAngles(5, 6));
    TEST_ASSERT_EQUAL(EQUAL, compareAngles(5, 6, 2));
}

void test_function_compareSpeed(void) {
    TEST_ASSERT_EQUAL(GREATER_THAN, compareAngles(180, 10));
    TEST_ASSERT_EQUAL(EQUAL, compareAngles(180, 185, 10));
    TEST_ASSERT_EQUAL(LESS_THAN, compareAngles(40, 300));
    TEST_ASSERT_EQUAL(GREATER_THAN, compareAngles(180, 10, 10));
    TEST_ASSERT_EQUAL(LESS_THAN, compareAngles(40, 300, 10));
    TEST_ASSERT_EQUAL(EQUAL, compareAngles(200, 185, 50));
}

// These goals are used in test_function_addToPacket and test_function_warpPacket
uint8_t goal_0[4] = {  0x10, 0x00, 0x50, 0x01};
uint8_t goal_1[4] = {  0x20, 0x02, 0x60, 0x03};
uint8_t goal_2[4] = {  0x30, 0x00, 0x70, 0x01};
uint8_t goal_3[4] = {  0x20, 0x02, 0x80, 0x03};

void test_function_addToPacket(void) {
    // G15ShieldInit(19200, 3, 8);
    TestServoRHA servo_test1(1, 2, 3, 8);
    servo_test1.initServo();

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t position = 0;
    uint8_t num_servo = 0;
    uint8_t buffer_test1[5] = {  0x01, 0x20, 0x02, 0x80, 0x03};

    servo_test1.addToPacket(buffer, position, goal_3, sizeof(goal_3), num_servo);
    TEST_ASSERT_EQUAL_UINT8(1, num_servo);
    TEST_ASSERT_EQUAL_UINT8(5, position);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, position);

    uint8_t buffer_test2[10] = {  0x01, 0x20, 0x02, 0x80, 0x03, 0x01, 0x20, 0x02, 0x60, 0x03};

    servo_test1.addToPacket(buffer, position, goal_1, sizeof(goal_1), num_servo);
    TEST_ASSERT_EQUAL_UINT8(2, num_servo);
    TEST_ASSERT_EQUAL_UINT8(10, position);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test2, buffer, position);
}

void test_function_warpPacket(void) {
    TestServoRHA servo_test0(0, 2, 3, 8);
    servo_test0.initServo();
    TestServoRHA servo_test1(1, 2, 3, 8);
    servo_test1.initServo();
    TestServoRHA servo_test2(2, 2, 3, 8);
    servo_test2.initServo();
    TestServoRHA servo_test3(3, 2, 3, 8);
    servo_test3.initServo();

    uint8_t data[30];
    uint8_t buffer[30];
    uint8_t buffer_test[30] = {   0xFF, 0xFF, 0xFE, 0x18, 0x83, 0x1E, 0x04,  // header, ID, length, instruction, adress, data length
                            0x00, 0x10, 0x00, 0x50, 0x01,  // servo ID 0 to position 0x010 with speed of 0x150
                            0x01, 0x20, 0x02, 0x60, 0x03,  // servo ID 1 to position 0x220 with speed of 0x360.
                            0x02, 0x30, 0x00, 0x70, 0x01,  // servo ID 2 to position 0x030 with speed of 0x170
                            0x03, 0x20, 0x02, 0x80, 0x03,  // servo ID 3 to position 0x220 with speed of 0x380
                            0x12 };  // checksum

    uint8_t position = 0;
    uint8_t num_servo = 0;

    servo_test0.addToPacket(data, position, goal_0, sizeof(goal_0), num_servo);
    servo_test1.addToPacket(data, position, goal_1, sizeof(goal_1), num_servo);
    servo_test2.addToPacket(data, position, goal_2, sizeof(goal_2), num_servo);
    servo_test3.addToPacket(data, position, goal_3, sizeof(goal_3), num_servo);
    TEST_ASSERT_EQUAL_UINT8(4, num_servo);
    TEST_ASSERT_EQUAL_UINT8(20, position);
    uint8_t buffer_len = 0;

    buffer_len = servo_test1.wrapPacket(buffer, data, position, GOAL_POSITION_L, num_servo);

    for (int i = 0; i < buffer_len; i++) {
      DebugSerialTSRHA("0x"); DebugSerialTSRHA((buffer[i], HEX)); DebugSerialTSRHA(", ");
    } DebugSerialTSRHALn(" ");

    TEST_ASSERT_EQUAL_UINT8(28, buffer_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test, buffer, buffer_len);
}

void test_function_SetWheelSpeed(void) {
    TestServoRHA servo_test1(1, 2, 3, 8);
    servo_test1.initServo();
    servo_test1.SetWheelSpeed(60, CW);  // speed 60% in CW direction
    TEST_ASSERT_EQUAL_INT(613, servo_test1.g15_speed);  // 613.8 is the exact number
    TEST_ASSERT_EQUAL_INT(CW, servo_test1.direction);

    servo_test1.SetWheelSpeed(60, CCW);  // speed 60% in CCW direction
    TEST_ASSERT_EQUAL_INT(685, servo_test1.g15_speed);  // 685.8 is the exact number
    TEST_ASSERT_EQUAL_INT(CCW, servo_test1.direction);
}

void process() {
  UNITY_BEGIN();

  RUN_TEST(test_function_compareSpeed);
  RUN_TEST(test_function_compareAngles);
  RUN_TEST(test_function_addToPacket);
  RUN_TEST(test_function_warpPacket);
  RUN_TEST(test_function_SetWheelSpeed);
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

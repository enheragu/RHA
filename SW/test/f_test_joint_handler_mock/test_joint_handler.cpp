/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Aug_31
 * @Project: RHA
 * @Filename: test_cytron_g15_servo.cpp
 * @Last modified by:   quique
 * @Last modified time: 24-Sep-2017
 */



#include "debug.h"
#include "servo_rha.h"
#include "joint_handler.h"
// #include "cytron_g15_servo.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 1


void test_function_warpSinglePacket(void) {
    JointHandler jh_test1;
    jh_test1.joint_[0].init(SERVO_ID, CW, A0);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    uint8_t checksum = ~(SERVO_ID + 0x04 + iWRITE_DATA + ServoRHAConstants::TORQUE_ENABLE + ON);
    uint8_t buffer_test1[8] = { 0xFF, 0xFF, SERVO_ID, 0x04, iWRITE_DATA, ServoRHAConstants::TORQUE_ENABLE, ON, checksum};

    jh_test1.joint_[0].servo_.setTorqueOnOfToPacket(buffer, ON);
    uint8_t txBuffer[BUFFER_LEN];
    jh_test1.warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, txBuffer, txBuffer[3]+4);
}

// These goals are used in test_function_addToPacket and test_function_warpPacket
uint8_t goal_0[5] = {  0x1E, 0x10, 0x00, 0x50, 0x01};
uint8_t goal_1[5] = {  0x1E, 0x20, 0x02, 0x60, 0x03};
uint8_t goal_2[5] = {  0x1E, 0x30, 0x00, 0x70, 0x01};
uint8_t goal_3[5] = {  0x1E, 0x20, 0x02, 0x80, 0x03};

void test_function_addToSyncPacket(void) {
    JointHandler jh_test1;
    uint8_t buffer[10];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;

    uint8_t buffer_test[10] = {   0x00, 0x10, 0x00, 0x50, 0x01,  // servo ID 0 to position 0x010 with speed of 0x150
                            0x01, 0x20, 0x02, 0x60, 0x03  // servo ID 1 to position 0x220 with speed of 0x360.
                        };  // without single data lenght

    jh_test1.joint_[0].init(0, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_0, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, num_bytes, "Num bytes");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, num_servo, "Num servo");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buffer_test, buffer_to_send, 4, "Test syncPacket 1");

    jh_test1.joint_[0].init(1, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_1, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(10, num_bytes, "Num bytes");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, num_servo, "Num servo");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buffer_test, buffer_to_send, 10, "Test syncPacket 2");
}

void test_function_warpSyncPacket(void) {
    JointHandler jh_test1;
    uint8_t buffer[10];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;

    jh_test1.joint_[0].init(0, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_0, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    jh_test1.joint_[0].init(1, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_1, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    jh_test1.joint_[0].init(2, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_2, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    jh_test1.joint_[0].init(3, CW, A0);
    jh_test1.joint_[0].servo_.addToPacket(buffer, goal_3, 5);
    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    jh_test1.warpSyncPacket(buffer_to_send, ServoRHAConstants::GOAL_POSITION_L, txBuffer, num_bytes, num_servo);

    uint8_t buffer_test[30] = {   0xFF, 0xFF, 0xFE, 0x18, 0x83, 0x1E, 0x04,  // header, ID, length, instruction, adress, data length
                            0x00, 0x10, 0x00, 0x50, 0x01,  // servo ID 0 to position 0x010 with speed of 0x150
                            0x01, 0x20, 0x02, 0x60, 0x03,  // servo ID 1 to position 0x220 with speed of 0x360.
                            0x02, 0x30, 0x00, 0x70, 0x01,  // servo ID 2 to position 0x030 with speed of 0x170
                            0x03, 0x20, 0x02, 0x80, 0x03,  // servo ID 3 to position 0x220 with speed of 0x380
                            0x12 };  // checksum

    TEST_ASSERT_EQUAL_UINT8(4, num_servo);
    TEST_ASSERT_EQUAL_UINT8(20, num_bytes);

    // for (int i = 0; i < txBuffer[4]+4; i++) {
    //   DebugSerialTJHMock("0x"); DebugSerialTJHMock((buffer[i], HEX)); DebugSerialTJHMock(", ");
    // } DebugSerialTJHMockLn(" ");

    TEST_ASSERT_EQUAL_UINT8(28, txBuffer[3]+4);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test, txBuffer, txBuffer[3]+4);
}

void test_function_checkConection_mock(void) {
    JointHandler jh_test1;
    uint8_t buffer[10];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];

    jh_test1.joint_[0].init(SERVO_ID, CW, A0);
    jh_test1.joint_[0].servo_.pingToPacket(buffer);

    jh_test1.warpSinglePacket(iPING, buffer, txBuffer);

    uint8_t checksum = ~ (SERVO_ID + 0x02 + iPING);
    uint8_t buffer_test[6] = { 0xFF, 0xFF, SERVO_ID, 0x02, iPING, checksum};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test, txBuffer, txBuffer[3]+4);
}

void test_function_sendSetWheelModeAll_mock(void) {
    JointHandler jh_test1;
    uint8_t buffer[10];
    uint8_t txBuffer[BUFFER_LEN];

    jh_test1.joint_[0].init(SERVO_ID, CW, A0);RHATypes::Timer eeprom_timer;
    eeprom_timer.setTimer(EEMPROM_WRITE_DELAY);
    eeprom_timer.activateTimer();

    // First set angle limit
    jh_test1.joint_[0].servo_.setWheelModeToPacket(buffer);
    jh_test1.warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
    uint8_t checksum = ~(SERVO_ID + 0x07 + iWRITE_DATA + ServoRHAConstants::CW_ANGLE_LIMIT_L + 0x00 + 0x00 + 0x00 + 0x00);

    uint8_t buffer_test[11] = { 0xFF, 0xFF, SERVO_ID, 0x07, iWRITE_DATA, ServoRHAConstants::CW_ANGLE_LIMIT_L, 0x00, 0x00, 0x00, 0x00, checksum};

    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buffer_test, txBuffer, txBuffer[3]+4, "Test set wheel mode");

    eeprom_timer.checkWait();
    eeprom_timer.activateTimer();

    // Then set torque ON
    jh_test1.joint_[0].servo_.setTorqueOnOfToPacket(buffer, ON);
    jh_test1.warpSinglePacket(iWRITE_DATA, buffer, txBuffer);
    checksum = ~(SERVO_ID + 0x04 + iWRITE_DATA + ServoRHAConstants::TORQUE_ENABLE + ON);

    uint8_t buffer_test1[8] = { 0xFF, 0xFF, SERVO_ID, 0x04, iWRITE_DATA, ServoRHAConstants::TORQUE_ENABLE, ON, checksum};

    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buffer_test1, txBuffer, txBuffer[3]+4, "Test set torque on");

    eeprom_timer.checkWait();
}

void test_function_controlLoop_oneJoint(void) {
    JointHandler jh_test1;
    uint8_t buffer[10];
    uint8_t buffer_to_send[BUFFER_LEN];
    uint8_t txBuffer[BUFFER_LEN];
    uint8_t num_bytes = 0, num_servo = 0;

    jh_test1.joint_[0].init(0, CW, A0);

    jh_test1.joint_[0].servo_.speed_regulator_.setKRegulator(10.0F, 10.0F, 10.0F);
    jh_test1.joint_[0].servo_.calculateTorque(-1.0F, 0.0F, 0.0F);  // changes init dir to CCW
    // Output must be torque = 10;
    jh_test1.joint_[0].servo_.addTorqueToPacket(buffer);

    num_bytes = jh_test1.addToSyncPacket(buffer_to_send, buffer, num_bytes);
    num_servo++;

    jh_test1.warpSyncPacket(buffer_to_send, ServoRHAConstants::MOVING_SPEED_L, txBuffer, num_bytes, num_servo);

    uint8_t bit1 = 10 & 0x00FF;
    uint8_t bit2 = 10 >> 8;
    uint8_t checksum = ~(0xFE + 0x07 + 0x83 + 0x20 + 0x02 + 0x00 + bit1 + bit2);
    uint8_t buffer_test[11] = {   0xFF, 0xFF, 0xFE, 0x07, 0x83, 0x20, 0x02,  // header, ID, length, instruction, adress, data length
                            0x00, bit1, bit2,  // servo ID to speed 0x0A 0x0 10 rpm
                            checksum };  // checksum

    TEST_ASSERT_EQUAL_UINT8(1, num_servo);
    TEST_ASSERT_EQUAL_UINT8(3, num_bytes);

    uint8_t buffer_test1[3] = { 0x00, bit1, bit2 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buffer_test1, buffer_to_send, 3, "Test addToSyncPacket 1");

    // for (int i = 0; i < txBuffer[4]+4; i++) {
    //   DebugSerialTJHMock("0x"); DebugSerialTJHMock((buffer[i], HEX)); DebugSerialTJHMock(", ");
    // } DebugSerialTJHMockLn(" ");

    TEST_ASSERT_EQUAL_UINT8(11, txBuffer[3]+4);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test, txBuffer, txBuffer[3]+4);
}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_function_warpSinglePacket);
  RUN_TEST(test_function_addToSyncPacket);
  RUN_TEST(test_function_warpSyncPacket);
  RUN_TEST(test_function_checkConection_mock);
  RUN_TEST(test_function_sendSetWheelModeAll_mock);
  RUN_TEST(test_function_controlLoop_oneJoint);
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

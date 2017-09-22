/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Aug_31
 * @Project: RHA
 * @Filename: test_servo_mock.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */



#include "debug.h"
#include "servo_rha.h"
#include "unity.h"

#ifdef UNIT_TEST

/**
  * This set of tests are intended to test ServoRHA functionalities.
  */

#define SERVO_ID 0x01

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


void test_function_addToPacket(void) {
    ServoRHA servo_test1(SERVO_ID);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t goal_1[4] = {  0x20, 0x02, 0x80, 0x03};
    uint8_t buffer_test1[6] = {  SERVO_ID, 0x04, 0x20, 0x02, 0x80, 0x03};

    servo_test1.addToPacket(buffer, goal_1, 4);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 6);
}

void test_function_updateInfoToPacket(void) {
    ServoRHA servo_test1(SERVO_ID);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buffer_test1[4] = { SERVO_ID, 0x02, ServoRHAConstants::PRESENT_POSITION_L, 0x08};

    servo_test1.addUpadteInfoToPacket(buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);
}

void test_function_addReturnOptionToPacket(void) {
    ServoRHA servo_test1(SERVO_ID);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buffer_test1[4] = {  SERVO_ID, 0x02, ServoRHAConstants::STATUS_RETURN_LEVEL, RETURN_PACKET_NONE};

    servo_test1.addReturnOptionToPacket(buffer, RETURN_PACKET_NONE);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);

    buffer_test1[4] = RETURN_PACKET_ALL;
    servo_test1.addReturnOptionToPacket(buffer, RETURN_PACKET_ALL);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);

    buffer_test1[4] =  RETURN_PACKET_READ_INSTRUCTIONS;
    servo_test1.addReturnOptionToPacket(buffer, RETURN_PACKET_READ_INSTRUCTIONS);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);
}

void test_function_setTorqueOnOfToPacket(void) {
    ServoRHA servo_test1(SERVO_ID);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buffer_test1[4] = {  SERVO_ID, 0x02, ServoRHAConstants::TORQUE_ENABLE, ON};

    servo_test1.setTorqueOnOfToPacket(buffer, ON);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);

    buffer_test1[4] =  OFF;
    servo_test1.setTorqueOnOfToPacket(buffer, OFF);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 4);
}

void test_function_set_exit_WheelModeToPacket(void) {
    ServoRHA servo_test1(SERVO_ID);

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buffer_test1[7] = {  SERVO_ID, 0x05, ServoRHAConstants::CW_ANGLE_LIMIT_L, 0,0,0,0};
    txBuffer[3] = 0 & 0x00FF;   // CW limit bottom 8 bits
    txBuffer[4] = 0 >> 8;       // CW limit top 8 bits
    txBuffer[5] = 0 & 0x00FF;  // CCW limit bottom 8 bits
    txBuffer[6] = 0 >> 8;      // CCW limit top 8 bits

    servo_test1.setWheelModeToPacket(buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 7);


    txBuffer[3] = 0 & 0x00FF;   // CW limit bottom 8 bits
    txBuffer[4] = 0 >> 8;       // CW limit top 8 bits
    txBuffer[5] = 1087 & 0x00FF;  // CCW limit bottom 8 bits
    txBuffer[6] = 1087 >> 8;      // CCW limit top 8 bits
    servo_test1.exitWheelModeToPacket(buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 7);
}

void test_function_updateInfo(void) {
    ServoRHA servo_test1(SERVO_ID);

    // position, position, speed, speed, load, load, voltage, temperature
    // 0xF4, 0x01 -> 500 degrees
    // 0x32, 0x02-> 50 rpm CW (10th bit: 1 = CW, 0 = CCW)
    // 0x37, 0x02-> 55 load CW (10th bit: 1 = CW, 0 = CCW)
    // 0x0C -> 12V
    // 0x20 -> 32 degrees
    uint8_t data[8] = {  0xF4, 0x01, 0x32, 0x02, 0x37, 0x02, 0x0C, 0x20};
    servo_test1.updateInfo(data, SERROR_INSTRUCTION);
    Test_ASSERT_EQUAL_UINT16(servo_test1.getPosition(), 500);
    Test_ASSERT_EQUAL_UINT16(servo_test1.getSpeed(), 50);
    Test_ASSERT_EQUAL_UINT8(servo_test1.getSpeedDir(), CW);
    Test_ASSERT_EQUAL_UINT16(servo_test1.getLoad(), 55);
    Test_ASSERT_EQUAL_UINT8(servo_test1.getLoadDir(), CW);
    Test_ASSERT_EQUAL_UINT8(servo_test1.getVoltage(), 12);
    Test_ASSERT_EQUAL_UINT8(servo_test1.getTemperature(), 32);
    Test_ASSERT_EQUAL_UINT16(servo_test1.getError(), SERROR_INSTRUCTION);
}

void test_function_calculateTorque(void) {
    TestServoRHA servo_test1(SERVO_ID);
    servo_test1.speed_regulator_.setKRegulator(10.0F, 10.0F, 10.0F);

    uint8_t data[8] = {  0xF4, 0x01, 0x32, 0x02, 0x37, 0x02, 0x0C, 0x20};
    servo_test1.updateInfo(data, SERROR_INSTRUCTION);  // sets init state of servo

    servo_test1.calculateTorque(-1.0F, 0.0F, 0.0F);  // changes init dir to CCW
    uint16_t torque_test = 10;
    TEST_ASSERT_EQUAL_UINT16(servo_test1.getGoalTorque(), torque_test);

    servo_test1.calculateTorque(160.0F, 0.0F, 0.0F);  // overloads servo saturation
    torque_test = 1023;
    TEST_ASSERT_EQUAL_UINT16(servo_test1.getGoalTorque(), torque_test);

    servo_test1.calculateTorque(10.0F, 0.0F, 0.0F);  // keeps direction
    torque_test = 10;
    TEST_ASSERT_EQUAL_UINT16(servo_test1.getGoalTorque(), torque_test);

    servo_test1.calculateTorque(10.0F, 0.0F, 0.0F);  // changes init dir to CW
    torque_test = 10;
    torque_test = torque_test | 0x0400
    TEST_ASSERT_EQUAL_UINT16(servo_test1.getGoalTorque(), torque_test);
}

void test_function_addTorqueToPacket(void) {
    TestServoRHA servo_test1(SERVO_ID);
    servo_test1.speed_regulator_.setKRegulator(10.0F, 10.0F, 10.0F);

    uint8_t data[8] = {  0xF4, 0x01, 0x32, 0x02, 0x37, 0x02, 0x0C, 0x20};
    servo_test1.updateInfo(data, SERROR_INSTRUCTION);  // sets init state of servo

    servo_test1.calculateTorque(-1.0F, 0.0F, 0.0F);  // changes init dir to CCW; goal_torque_ = 10

    uint8_t buffer[10] = {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // 0x0A, 0x00 -> torque = 10 (Speed bottom 8 bits, speed top 8 bits)
    uint8_t buffer_test1[5] = {  SERVO_ID, 0x04, ServoRHAConstants::MOVING_SPEED_L, 0x0A, 0x00};

    servo_test1.addTorqueToPacket(buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(buffer_test1, buffer, 5);

}

void process() {
  UNITY_BEGIN();

  RUN_TEST(test_function_compareSpeed);
  RUN_TEST(test_function_compareAngles);
  RUN_TEST(test_function_updateInfoToPacket);
  RUN_TEST(test_function_updateInfo);
  RUN_TEST(test_function_addReturnOptionToPacket);
  RUN_TEST(test_function_setTorqueOnOfToPacket);
  RUN_TEST(test_function_set_exit_WheelModeToPacket);
  RUN_TEST(test_function_updateInfo);
  RUN_TEST(test_function_calculateTorque);
  // RUN_TEST(test_function_warpPacket);
  // RUN_TEST(test_function_SetWheelSpeed);
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

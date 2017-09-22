/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Aug_31
 * @Project: RHA
 * @Filename: test_servo_real.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 22_Sep_2017
 */


#include "debug.h"
#include "joint_rha.h"
#include "unity.h"

#ifdef UNIT_TEST

#define SERVO_ID 0x01
#define UP_DIRECTION CW
#define POT_PIN A0

void test_function_setSpeedGoal(void) {
    JointRHA joint_test1(SERVO_ID, UP_DIRECTION, POT_PIN);
    RHATypes::SpeedGoal speed_goal(SERVO_ID, 50, 0, CW);  // target for servo with id=SERVO_ID to 50 rpm with no accel slope in CW dir

    uint8_t flag = joint_test1.setSpeedGoal(speed_goal);
    TEST_ASSERT_TRUE(flag);
    Test_ASSERT_EQUAL_UINT16(joint_test1.getSpeedTarget(), 50);
    Test_ASSERT_EQUAL_UINT16(joint_test1.getSpeedSlope(), 0);
    Test_ASSERT_EQUAL_UINT8(joint_test1.getDirectionTarget(), CW);

    RHATypes::SpeedGoal speed_goal2(SERVO_ID+1, 110, 0, CC W);  // target for servo with id=SERVO_ID+1 to 110 rpm with no accel slope in CCW dir
    // Servo ID is not the same, this goal is not intender for this servo son nothing changes

    flag = joint_test1.setSpeedGoal(speed_goal2);
    TEST_ASSERT_FALSE(flag);
    Test_ASSERT_EQUAL_UINT16(joint_test1.getSpeedTarget(), 50);
    Test_ASSERT_EQUAL_UINT16(joint_test1.getSpeedSlope(), 0);
    Test_ASSERT_EQUAL_UINT8(joint_test1.getDirectionTarget(), CW);
}

void test_function_speedError(void) {
  JointRHA joint_test1(SERVO_ID, UP_DIRECTION, POT_PIN);
  RHATypes::SpeedGoal speed_goal(SERVO_ID, 80, 0, CW);  // target for servo with id=SERVO_ID to 50 rpm with no accel slope in CW dir

  // 0xF4, 0x01 -> 500 degrees
  // 0x32, 0x02-> 50 rpm CW (10th bit: 1 = CW, 0 = CCW)
  // 0x37, 0x02-> 55 load CW (10th bit: 1 = CW, 0 = CCW)
  // 0x0C -> 12V
  // 0x20 -> 32 degrees
  uint8_t data[8] = {  0xF4, 0x01, 0x32, 0x02, 0x37, 0x02, 0x0C, 0x20};
  joint_test1.servo_.updateInfo(data, SERROR_INSTRUCTION);

  // Wants to go 80rpm CW and it goes at 50rpm CW
  uint8_t flag = joint_test1.setSpeedGoal(speed_goal);
  float output = servo_test1.speedError();
  TEST_ASSERT_EQUAL_FLOAT(output, 30);

  RHATypes::SpeedGoal speed_goal2(SERVO_ID, 80, 0, CCW);  // target for servo with id=SERVO_ID to 50 rpm with no accel slope in CCW dir
  // Wants to go 80rpm CCW and it goes at 50rpm CW
  uint8_t flag = joint_test1.setSpeedGoal(speed_goal2);
  float output = servo_test1.speedError();
  TEST_ASSERT_EQUAL_FLOAT(output, -30);

  RHATypes::SpeedGoal speed_goal2(SERVO_ID, 30, 0, CCW);  // target for servo with id=SERVO_ID to 30 rpm with no accel slope in CCW dir
  // Wants to go 30rpm CCW and it goes at 50rpm CW
  // goes faster than expected so it changes sign, then changes again as wants to go in the other dir
  uint8_t flag = joint_test1.setSpeedGoal(speed_goal2);
  float output = servo_test1.speedError();
  TEST_ASSERT_EQUAL_FLOAT(output, 20);

  RHATypes::SpeedGoal speed_goal2(SERVO_ID, 30, 0, CW);  // target for servo with id=SERVO_ID to 30 rpm with no accel slope in CCW dir
  // Wants to go 30rpm CW and it goes at 50rpm CW
  // goes faster than expected so it changes sign
  uint8_t flag = joint_test1.setSpeedGoal(speed_goal2);
  float output = servo_test1.speedError();
  TEST_ASSERT_EQUAL_FLOAT(output, -20);
}

void process() {
    UNITY_BEGIN();

    RUN_TEST(test_function_setSpeedGoal);
    RUN_TEST(test_function_speedError);
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

/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Aug_31
 * @Project: RHA
 * @Filename: test_servo_real.cpp
 * @Last modified by:   quique
 * @Last modified time: 23-Sep-2017
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
    DebugSerialTJRHALn2("Servo ID is: ", joint_test1.servo_.getID());
    DebugSerialTJRHALn2("Speed goal ID is: ", speed_goal.servo_id);
    uint8_t flag = joint_test1.setSpeedGoal(speed_goal);
    TEST_ASSERT_TRUE(flag);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(50, joint_test1.getSpeedTarget(), "Speed target");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, joint_test1.getSpeedSlope(), "Speed slope");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(CW, joint_test1.getDirectionTarget(), "Direction target");

    RHATypes::SpeedGoal speed_goal2(SERVO_ID+1, 110, 20, CCW);  // target for servo with id=SERVO_ID+1 to 110 rpm with no accel slope in CCW dir
    // Servo ID is not the same, this goal is not intender for this servo son nothing changes

    flag = joint_test1.setSpeedGoal(speed_goal2);
    TEST_ASSERT_FALSE(flag);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(50, joint_test1.getSpeedTarget(), "Speed target");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, joint_test1.getSpeedSlope(), "Speed slope");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(CW, joint_test1.getDirectionTarget(), "Direction target");
}

void test_function_speedError(void) {
  JointRHA joint_test1(SERVO_ID, UP_DIRECTION, POT_PIN);

  // 0xF4, 0x01 -> 500 degrees
  // 0x32, 0x02-> 50 rpm CW (10th bit: 1 = CW, 0 = CCW)
  // 0x37, 0x02-> 55 load CW (10th bit: 1 = CW, 0 = CCW)
  // 0x0C -> 12V
  // 0x20 -> 32 degrees
  uint8_t data[8] = {  0xF4, 0x01, 0x32, 0x04, 0x37, 0x04, 0x0C, 0x20};
  joint_test1.servo_.updateInfo(data, SERROR_INSTRUCTION);

  RHATypes::SpeedGoal speed_goal(SERVO_ID, 80, 0, CW);  // target for servo with id=SERVO_ID to 80 rpm with no accel slope in CW dir
  // Wants to go 80rpm CW and it goes at 50rpm CW
  joint_test1.setSpeedGoal(speed_goal);
  float output = joint_test1.speedError();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5, 30.0F, output, "Test 1");

  RHATypes::SpeedGoal speed_goal2(SERVO_ID, 80, 0, CCW);  // target for servo with id=SERVO_ID to 80 rpm with no accel slope in CCW dir
  // Wants to go 80rpm CCW and it goes at 50rpm CW
  joint_test1.setSpeedGoal(speed_goal2);
  output = joint_test1.speedError();
  DebugSerialTJRHALn2("Servo speed dir is: ", joint_test1.servo_.getSpeedDir());
  DebugSerialTJRHALn2("Speed goal dir is: ", speed_goal2.direction);
  DebugSerialTJRHALn2("speedError output: ", output);

  int8_t sign = 1;
  if ( joint_test1.getDirectionTarget() != joint_test1.servo_.getSpeedDir()) sign = -1;
  DebugSerialTJRHALn2("Expected output: ", sign*((float)speed_goal2.speed - (float)joint_test1.servo_.getSpeed()));
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5, -30.0F, output, "Test 2");

  RHATypes::SpeedGoal speed_goal3(SERVO_ID, 30, 0, CCW);  // target for servo with id=SERVO_ID to 30 rpm with no accel slope in CCW dir
  // Wants to go 30rpm CCW and it goes at 50rpm CW
  // goes faster than expected so it changes sign, then changes again as wants to go in the other dir
  joint_test1.setSpeedGoal(speed_goal3);
  output = joint_test1.speedError();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5, 20.0F, output, "Test 3");

  RHATypes::SpeedGoal speed_goal4(SERVO_ID, 30, 0, CW);  // target for servo with id=SERVO_ID to 30 rpm with no accel slope in CCW dir
  // Wants to go 30rpm CW and it goes at 50rpm CW
  // goes faster than expected so it changes sign
  joint_test1.setSpeedGoal(speed_goal4);
  output = joint_test1.speedError();
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5, -20.0F, output, "Test 4");
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

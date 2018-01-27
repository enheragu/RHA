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


void test_function_calculateTorque(void) {
    JointRHA joint_test1(SERVO_ID);
    // Needs speed goal in order to calculate the prealimentation correctly
    RHATypes::SpeedGoal speed_goal(SERVO_ID, 50, 0, CW);  // target for servo with id=SERVO_ID to 50 rpm with no accel slope in CW dir
    joint_test1.setSpeedGoal(speed_goal);
    joint_test1.speed_regulator_.setKRegulator(10.0F, 10.0F, 10.0F);

    uint8_t data[8] = {  0xF4, 0x01, 0xC5, 0x05, 0x37, 0x04, 0x0C, 0x20};
    joint_test1.updateInfo(data, SERROR_INSTRUCTION);  // sets init state of servo starting in CW dir

    joint_test1.calculateTorque(-1.0F, 0.0F, 0.0F);  // changes init dir to CCW  // TODO(eeha): for now it just stops the servo, no changeing direction is involved
    uint16_t torque_test = 64;  // TODO(eeha): check if its right
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(torque_test, joint_test1.getGoalTorque(), "Torque test 1");

    joint_test1.calculateTorque(160.0F, 0.0F, 0.0F);  // overloads servo saturation
    torque_test = 1023;
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(torque_test, joint_test1.getGoalTorque(), "Torque test 2");

    uint8_t data2[8] = {  0xF4, 0x01, 0x32, 0x00, 0x37, 0x00, 0x0C, 0x20};
    joint_test1.updateInfo(data2, SERROR_INSTRUCTION);  // sets init state of servo starting in CCW dir

    joint_test1.calculateTorque(-10.0F, 0.0F, 0.0F);  // TODO(eeha): for now it just stops the servo, no changeing direction is involved
    torque_test = 11;  // TODO(eeha): check if its right
    // torque_test = torque_test | 0x0400;
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(torque_test, joint_test1.getGoalTorque(), "Torque test 3");

    joint_test1.calculateTorque(10.0F, 0.0F, 0.0F);  // keeps direction
    uint16_t torque_offset = joint_test1.getLoad() - uint16_t(50 / TORQUE_PREALIMENTATION_SLOPE);
    uint16_t prealimentation = torque_offset + TORQUE_PREALIMENTATION_SLOPE*float(50);
    torque_test = 100 + prealimentation;
    torque_test = 211;  // TODO(eeha): check if its right
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(10.0F, joint_test1.getError(), "Test error. Torque test 4");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(50, joint_test1.getSpeedTarget(), "Speed Target. Torque test 4");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(torque_test, joint_test1.getGoalTorque(), "Torque test 4");
}

void process() {
    UNITY_BEGIN();

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

/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   23-Dec-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 23-Dec-2017
 */

#include "debug.h"
#include "rha_types.h"
#include "unity.h"

#ifdef UNIT_TEST


/**
  * This set of tests are intended to test RHATypes functionalities and data types.
  */

#define TIMER_TEST_TIME_MILIS 100
#define TIMER_TEST_TIME_MICROS 2500
#define TIME_SPENT_EXECUTING_COMMANDS_MILIS 2
#define TIME_SPENT_EXECUTING_COMMANDS_MICROS 2000

void test_class_timer_function_checkwait(void) {
    RHATypes::Timer timer_test;
	timer_test.setTimer(TIMER_TEST_TIME_MILIS);
    long init_time = millis();
    timer_test.activateTimer();
    timer_test.checkWait();
    TEST_ASSERT_TRUE(millis() >= (init_time + TIMER_TEST_TIME_MILIS) );
}

void test_class_timer_function_checkcontinue(void) {
    RHATypes::Timer timer_test;
	timer_test.setTimer(TIMER_TEST_TIME_MILIS);
    long init_time = millis();
    timer_test.activateTimer();
    timer_test.checkContinue();
    TEST_ASSERT_TRUE(millis() < (init_time + TIME_SPENT_EXECUTING_COMMANDS_MILIS) );
}

void test_class_timer_micros_function_checkwait(void) {
    RHATypes::Timer timer_test;
	timer_test.setTimer(TIMER_TEST_TIME_MILIS);
    long init_time = micros();
    timer_test.activateTimer();
    timer_test.checkWait();
    TEST_ASSERT_TRUE(micros() > (init_time + TIMER_TEST_TIME_MICROS) );
}

void test_class_timer_micros_function_checkcontinue(void) {
    RHATypes::Timer timer_test;
	timer_test.setTimer(TIMER_TEST_TIME_MILIS);
    long init_time = micros();
    timer_test.activateTimer();
    timer_test.checkContinue();
    TEST_ASSERT_TRUE(micros() < (init_time + TIME_SPENT_EXECUTING_COMMANDS_MICROS) );
}


void process() {
  UNITY_BEGIN();

  RUN_TEST(test_class_timer_function_checkwait);
  RUN_TEST(test_class_timer_function_checkcontinue);
  RUN_TEST(test_class_timer_micros_function_checkwait);
  RUN_TEST(test_class_timer_micros_function_checkcontinue);
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

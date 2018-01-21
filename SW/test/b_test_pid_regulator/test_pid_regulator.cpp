/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   23-Dec-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 23-Dec-2017
 */

#include "debug.h"
#include "pid_regulator.h"
#include "rha_types.h"
#include "unity.h"

#ifdef UNIT_TEST
/**
  * This set of tests are intended to test regulator functionalities in pid_regulator.h
  */

#define KP_TEST 10
#define KI_TEST 5
#define KD_TEST 1

#define ERROR_TEST 10
#define DERROR_TEST 2
#define IERROR_TEST 1

#define REGULATOR_IERROR_ACUM_OFFSET 5

void test_class_regulator_function_setRegulator(void) {
    RHATypes::Regulator regulator_test;
    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);
    TEST_ASSERT_EQUAL(KP_TEST, regulator_test.getKp());
    TEST_ASSERT_EQUAL(KI_TEST, regulator_test.getKi());
    TEST_ASSERT_EQUAL(KD_TEST, regulator_test.getKd());
}

void test_class_regulator_function_resetRegulator(void) {
    RHATypes::Regulator regulator_test;
    TEST_ASSERT_EQUAL(0, regulator_test.getKp());
    TEST_ASSERT_EQUAL(0, regulator_test.getKi());
    TEST_ASSERT_EQUAL(0, regulator_test.getKd());

    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);

    regulator_test.resetRegulator();
    TEST_ASSERT_EQUAL(0, regulator_test.getKp());
    TEST_ASSERT_EQUAL(0, regulator_test.getKi());
    TEST_ASSERT_EQUAL(0, regulator_test.getKd());
}

// Test regulator function when it hasn't accumulated ierror
void test_class_regulator_function_regulator_simple(void) {
    RHATypes::Regulator regulator_test;
    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);

    float result_1 = regulator_test.regulator(ERROR_TEST, DERROR_TEST, IERROR_TEST);
    // Regulator should return in first iteration:
    float expected_result1 = KP_TEST * ERROR_TEST + KD_TEST * DERROR_TEST + KI_TEST * IERROR_TEST;
    TEST_ASSERT_EQUAL_FLOAT(expected_result1, result_1);
}

void test_class_regulator_function_regulator_simple_acumulative_1(void) {
    RHATypes::Regulator regulator_test;
    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);

    regulator_test.regulator(ERROR_TEST, DERROR_TEST, IERROR_TEST);
    float result_2 = regulator_test.regulator(ERROR_TEST, DERROR_TEST, IERROR_TEST);
    // Regulator should return in second iteration (accumulates ierror twice, the one from before and this one now):
    float expected_result2 = KP_TEST * ERROR_TEST + KD_TEST * DERROR_TEST + KI_TEST * IERROR_TEST*2;
    TEST_ASSERT_EQUAL_FLOAT(expected_result2, result_2);
}

// Test that ierror adds all values from the vector (now with full ierror)
void test_class_regulator_function_regulator_simple_acumulative_2(void) {
    RHATypes::Regulator regulator_test;
    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);

    float result_3 = 0;
    float ierror_accumulated = 0;
    for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) {
        result_3 = regulator_test.regulator(ERROR_TEST, DERROR_TEST, IERROR_TEST + i);
        ierror_accumulated += IERROR_TEST + i;
    }
    // Regulator should return in INTEGER_INTERVALth iteration (accumulates ieror INTEGER_INTERVAL times):
    float expected_result3 = KP_TEST * ERROR_TEST + KD_TEST * DERROR_TEST + KI_TEST * ierror_accumulated;
    TEST_ASSERT_EQUAL_FLOAT(expected_result3, result_3);
}

// Test how ierror adds new info once the vector overloads
void test_class_regulator_function_regulator_simple_acumulative_overload(void) {
    RHATypes::Regulator regulator_test;
    regulator_test.setKRegulator(KP_TEST, KI_TEST, KD_TEST);

    // Test that the ierror is overloaded properly
    float result_4 = 0;
    float ierror_accumulated2 = 0;
    // First the vector is loaded with some info, once it overloads it should rewrite previous values

    float ierror_expected[INTEGER_INTERVAL];
    int index = 0;

    for (uint8_t i = 0; i < INTEGER_INTERVAL*3; i++) {
        result_4 = regulator_test.regulator(ERROR_TEST, DERROR_TEST, IERROR_TEST + i);
        ierror_expected[index] = IERROR_TEST + i;
        index++;
        if (index > INTEGER_INTERVAL) index = 0;
    }

    float sum_i_error_expected = 0;
    for (uint8_t i = 0; i < INTEGER_INTERVAL; i++) sum_i_error_expected += ierror_expected[i];

    // Regulator should return in INTEGER_INTERVALth iteration (accumulates ieror INTEGER_INTERVAL times):
    float expected_result4 = KP_TEST * ERROR_TEST + KD_TEST * DERROR_TEST + KI_TEST * sum_i_error_expected;
    TEST_ASSERT_EQUAL_FLOAT(expected_result4, result_4);
}



void process() {
  UNITY_BEGIN();

  RUN_TEST(test_class_regulator_function_setRegulator);
  RUN_TEST(test_class_regulator_function_resetRegulator);
  RUN_TEST(test_class_regulator_function_regulator_simple);
  RUN_TEST(test_class_regulator_function_regulator_simple_acumulative_1);
  RUN_TEST(test_class_regulator_function_regulator_simple_acumulative_2);
  RUN_TEST(test_class_regulator_function_regulator_simple_acumulative_overload);
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

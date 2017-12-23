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

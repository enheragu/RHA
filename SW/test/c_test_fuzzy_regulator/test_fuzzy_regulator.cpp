/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   23-Dec-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 23-Dec-2017
 */

#include "debug.h"
#include "joint_rha.h"
#include "unity.h"

#ifdef UNIT_TEST


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

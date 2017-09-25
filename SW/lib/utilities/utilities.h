/**
 * @file
 * @brief Implements a set of utilities to measure, experimentally, some interesting parameters.
 *
 * Measures real speed of servo, time spent with packet handling, etc
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: utilities.h
 * @Last modified by:   quique
 * @Last modified time: 24-Sep-2017
 */

#include "debug.h"
#include "servo_rha.h"
#include "joint_handler.h"
// #include "cytron_g15_servo.h"
#include <Arduino.h>


void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation);

namespace ServoUtilities {
    void setServoId(uint8_t new_id);
    #define LED 13

    void fullFactoryResetBR();
    void setServoId(uint8_t new_id);
} // End of ServoUtilities namespace

namespace RegulatorTestData {
    #define SAMPLE_REGULATOR 150
    #define SAMPLE_KP 6
    #define KP_SAMPLES {1.66, 5, 10, 20, 50, 100};
}  // End of RegulatorTestData namespace

namespace CheckSpeedTestData {
    #define SPEED 800  // torque send to check speed
}  // End of CheckSpeedTestData namespace


class JHUtilitiesJH : public JointHandler {
    uint16_t error_;
    uint8_t IDcurrent_;
    int baudrateMode_;

 public:
     JHUtilitiesJH() {
         baudrateMode_ = 0;
     }

    void initJoints(uint8_t joint_to_test);
    void extractRegulatorData(uint8_t joint_to_test);
    void checkTimeGetInfo(uint8_t repetitions, uint8_t joint_to_test);
    void checkComSucces(uint16_t repetitions);
    void checkSpeed(uint8_t joint_to_test);

};

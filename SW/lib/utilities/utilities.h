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
 * @Last modified time: 30-Sep-2017
 */

#include "debug.h"
#include "servo_rha.h"
#include "joint_handler.h"
// #include "cytron_g15_servo.h"
#include <Arduino.h>

namespace MeasureUtilities {
    void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation);
}

namespace ServoUtilities {
    void setServoId(uint8_t new_id);
    #define LED 13

    void fullFactoryResetBR();
    void setServoId(uint8_t new_id);
} // End of ServoUtilities namespace

namespace RegulatorTestData {
    #define SAMPLE_REGULATOR 500
    #define SPEED_REGULATOR_TEST 120
    #define SAMPLE_KP 3
    #define KP_SAMPLES {2, 2, 2};  // 1
    #define KD_SAMPLES {0,0.5,0};  // 1
    #define KI_SAMPLES {0,0,0.1};  // 1
    // #define SAMPLE_KP 6
    // #define KP_SAMPLES {1.6,  20,  20,  20,  20,  20};  // Mixed presentation
    // #define KD_SAMPLES {0,    0,    1,  15,  15,  15};  // Mixed presentation
    // #define KI_SAMPLES {0,    0,    0,   0, 0.1,   1};  // Mixed presentation

    // #define KP_SAMPLES {20, 50, 70, 90, 110, 130};  // 6
    // #define KP_SAMPLES {20, 20, 20, 20, 20, 20};  // 2, 3, 4, 5, 7
    // #define KP_SAMPLES {1.66, 5, 10, 20, 50, 100};  // 1
    // #define KD_SAMPLES {0,0,0,0,0,0};  // 1
    // #define KI_SAMPLES {0,0,0,0,0,0};  // 1
    // #define KD_SAMPLES {15,15,15,15,15,15};  // 4, 5, 6 y 7
    // #define KD_SAMPLES {1,5,10,15,20,25};  // 3
    // #define KD_SAMPLES {1,2,3,4,5,6};  // 2
    // #define KI_SAMPLES {1,2,3,4,5,6};  // 4
    // #define KI_SAMPLES {0.1,0.5,1.0,1.5,2.0,2.5};  // 5
    // #define KI_SAMPLES {0.01,0.05,0.10,0.15,0.20,0.25};  // 7
    // #define KI_SAMPLES {1,5,10,15,20,25};  // 1, 2, 3, 4 y 5
}  // End of RegulatorTestData namespace

namespace StepTest {
    #define SAMPLE_STEP 80
    #define SAMPLE_TEST_STEP 5
    #define STEP_SPEED 1023
}

namespace SlopeTest {
    #define SAMPLE_SLOPE 110
    #define SAMPLE_TEST_SLOPE 20
    #define SLOPE_SPEED 0.1
}


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
    void extractStepInputData(uint8_t joint_to_test);
    void extractSlopeInputData(uint8_t joint_to_test);
    void checkTimeGetInfo(uint8_t repetitions, uint8_t joint_to_test);
    void checkComSucces(uint16_t repetitions);
    void checkSpeed(uint8_t joint_to_test);

};

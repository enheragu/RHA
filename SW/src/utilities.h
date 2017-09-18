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
 * @Last modified time: 17-Sep-2017
 */

#include <Arduino.h>
#include "debug.h"
#include "servo_rha.h"
#include "joint_handler.h"
// #include "cytron_g15_servo.h"

/**
  * @brief Analyses error and prints error msgs
  */
void printServoStatusError (uint16_t error);



namespace MeasureUtilities{

    #define SPEED 800  // torque send to check speed
    #define SPEED_TARGET 80  // speed target in rpm
    #define KP_REGULATOR 150  // kp for extractRegulatorData function
    #define LOOP_FREQUENCY 100  // in ms
    #define SAMPLE_REGULATOR 150 // samples taken every LOOP_FREQUENCY

    #define CHAUVENET_REPETITIONS 50  // too many repetitions cause memory overfload

    void checkSpeed();


    #define KN 1.54 // Chauvenet coeficient for n = 4

    void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation);

    void checkTimeGetInfo(uint8_t repetitions);

    void checkTimeSpeedRead(uint8_t repetitions);

    void extractRegulatorData();

    void checkComSucces(uint16_t repetitions);
} //end of MeasureUtilities namespace

/***************************************************************
 *        This code is derived from Cytron G15 examples        *
 ***************************************************************/
namespace ServoUtilities{


    void setServoId(uint8_t new_id);


    #define LED 13


    void fullFactoryResetBR();
} // End of ServoUtilities namespace


class Utilities {
    ServoRHA servo_test1_;
    uint8_t data_[10];
    uint16_t error_;
    uint8_t IDcurrent_;
    int baudrateMode_;

 public:
    Utilities();
    void checkTimeGetInfo(uint8_t repetitions);
};

Utilities::Utilities(){
    baudrateMode_ = 0;
    ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
    servo_broadcast.init();

    error_ = servo_broadcast.ping(data_);
    IDcurrent_ = data_[0];
    DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent_);
    if (error_ != 0 && error_ != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
        printServoStatusError(error_);
        DebugSerialUtilitiesLn("Error in servo comunication, end of checkTimeGetInfo");
        return;
    }
    //ServoRHA servo_test1();
    servo_test1_.init(IDcurrent_,2,3,8);

}

void Utilities::checkTimeGetInfo(uint8_t repetitions){
    DebugSerialSeparation(1);

    uint32_t initTime = 0;
    uint32_t timeSpent [repetitions];

    DebugSerialUtilitiesLn("Begin of loop to take data");
    for (uint8_t i = 0; i < repetitions; i++){
        initTime = millis();
        servo_test1_.updateInfo();
        timeSpent[i] = millis()-initTime;
        if (servo_test1_.getError() != 0){
            DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
            printServoStatusError(servo_test1_.getError());
            return;
        }
    }
    DebugSerialUtilitiesLn("Data taken, calling averageChauvenet()");
    float average_time = 0, standard_deviation = 0;
    MeasureUtilities::averageChauvenet(timeSpent,repetitions,average_time, standard_deviation);
    DebugSerialUtilitiesLn2("checkTimeGetInfo: Time spent with ServoRHA::updateInfo: ", average_time);
    DebugSerialUtilitiesLn2("checkTimeGetInfo: number of repetitions: ", repetitions);
    DebugSerialUtilitiesLn("checkTimeSpeedRead: 9 bytes read");
    DebugSerialSeparation(1);
}

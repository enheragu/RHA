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
 * @Last modified time: 21-Sep-2017
 */

#include <Arduino.h>
#include "debug.h"
#include "servo_rha.h"
#include "joint_handler.h"
// #include "cytron_g15_servo.h"


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
    #define KP_SAMPLES { float kp_samples[SAMPLE_KP] = {1.66, 5, 10, 20, 50, 100}; }
    #define KP_SAMPLES(a) { kp_samples[a] }
}  // End of RegulatorTestData namespace

namespace CheckSpeedTestData {
    #define SPEED 800  // torque send to check speed
}  // End of CheckSpeedTestData namespace


class Utilities : public JointHandler {
    uint8_t data_[10];
    uint16_t error_;
    uint8_t IDcurrent_;
    int baudrateMode_;

 public:
    Utilities();
    void checkTimeGetInfo(uint8_t repetitions);

    virtual void initJoints();
    virtual void controlLoop();
    void extractRegulatorData();
    void checkComSucces(uint16_t repetitions);
};

Utilities::Utilities(){
    baudrateMode_ = 0;
}

void Utilities::initJoints() {
    DebugSerialUtilitiesLn("initJoints: begin of function");
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
    joint_[1].init(IDcurrent_, CW, A0);
}


/**
 * @brief extractRegulatorData tests ServoRHA regulator and prints through serial monitor all info (python list style) to make a graphic. It can test one servo. Autodetects ID of the one connected
 * @method extractRegulatorData
 */
void Utilities::extractRegulatorData() {
    int sample = 0;
    KP_SAMPLES;  // macro defined in the top of utilities.h

    SpeedGoal speed_goal(1,50,0);  // Id, speed, speed_slope
    setSpeedGoal(speed_goal);
    for (int samples = 0; samples < SAMPLE_KP; samples++) {
        joint_[0].servo_.speed_regulator_.setKRegulator(KP_SAMPLES(sample),0,0);  // KP_SAMPLES(a) defined in the top of utilities.h
        Serial.print("n_data"); Serial.print(sample); Serial.print(" = "); Serial.println(SAMPLE_REGULATOR);
        Serial.print("speed_target"); Serial.print(sample); Serial.print("  = "); Serial.println(joint_handler.joint_[0].getSpeedTarget());
        Serial.print("regulatorTest"); Serial.print(sample); Serial.print("  = [");
        Serial.print("['"); Serial.print(joint_[0].servo_.getKp()); Serial.print("']");

        for (int counter = 0, counter < SAMPLE_REGULATOR; counter++) {
            unsigned long time_init = millis();
            controlLoop();
            Serial.print(",['"); Serial.print(joint_[0].servo_.getSpeed()); Serial.print("','");
            Serial.print(time_init); Serial.println("']\\");
        }

        Serial.println("]");
    }
    return;
}

/**
 * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::updateInfo() . It can test one servo. Autodetects ID of the one connected
 * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions). Max of 255 (danger of memory overload)
 * @see checkTimeSpeedRead(). Both are used together to compare speed rate in comunication.
 * @see averageChauvenet()
 */
void Utilities::checkTimeGetInfo(uint8_t repetitions) {
    DebugSerialSeparation(1);

    uint64_t initTime = 0;
    uint32_t timeSpent [repetitions];

    DebugSerialUtilitiesLn("Begin of loop to take data");
    for (uint8_t i = 0; i < repetitions; i++){
        initTime = millis();
        updateJointInfo();
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
    DebugSerialUtilitiesLn("checkTimeSpeedRead: 8 bytes read");
    DebugSerialSeparation(1);
}

/**
 * @brief This function is intended to test new baudrates and it's success comunication ratio
 * @method checkPingSucces
 * @param  repetitions   number of repetitions to perform
 */
void Utilities::checkComSucces(uint16_t repetitions) {
    DebugSerialUtilitiesLn("checkComSucces: begin of function");
    DebugSerialSeparation(1);

    uint16_t succes_ping = 0;
    for (uint16_t i = 0; i < repetitions; i++)
    {

        if (checkConection()) succes_ping++;
        delay(5);
    }
    DebugSerialUtilitiesLn2("checkComSucces: Success total (ping): ", succes_ping);
    DebugSerialUtilitiesLn2("checkComSucces: number of repetitions: ", repetitions);
    DebugSerialSeparation(1);
}


/**
 * @brief checkSpeed implements an encoder mode to measure real speed in RPM and check agains the measure returned by servo and torque value sent. It can test one servo. Autodetects ID of the one connected
 */
void Utilities::checkSpeed(uint8_t joint_to_test){
    DebugSerialUtilitiesLn("checkSpeed: begin of function");

    uint32_t encoderTemp = 0,
         encoderCurrent = 0,
         encoderFullRotation = 100,
         encoderTotal = 0;

    uint16_t speed_set = SPEED,
        torque_set = 1023;

    uint8_t encoderFlag = 0;

    uint16_t pos = 0;

    uint32_t initTime = 0;
    uint32_t currentTime = 0;

    sendSetTorqueLimit(torque_set);
    delay(25);
    sendSetWheelMode();
    delay(25);
    delay(25);

    updateJointInfo();            //get the current position from servo
    pos = joint_[joint_to_test].servo_getPos();
    encoderTemp = pos;
    encoderFlag = 1;
    initTime = millis();

    DebugSerialUtilitiesLn("Configuration done.");
    delay(2000);
    DebugSerialUtilitiesLn("Start moving");

    sendSetWheelSpeed(speed_set, CW);
    delay(25);

    while (1) {
        //DebugSerialUtilitiesLn("Init while loop");
        updateJointInfo();            //get the current position from servo
        pos = joint_[joint_to_test].servo_getPos();
        encoderCurrent = pos;
        //DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

        if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
            encoderTotal++;
            currentTime = millis();
            float time_whole = ((float)(currentTime - initTime) / 1000);
            float speed_now = ((float)encoderTotal / (float)time_whole)*60;
            updateJointInfo();            //get the current position from servo
            float speed_read = joint_[joint_to_test].servo_getSpeed();
            DebugSerialSeparation(1);
            DebugSerialUtilitiesLn2("  -  Torque set is: ", speed_set);
            DebugSerialUtilitiesLn2("  -  Speed calculated is (in rpm): ", speed_now);
            DebugSerialUtilitiesLn2("  -  Speed calculated is (in rad/s): ", speed_now/(2*PI/60));
            DebugSerialUtilitiesLn2("  -  Speed read is (in rpm): ", speed_read*(2*PI/60));
            DebugSerialUtilitiesLn2("  -  Speed read is (in rad/s): ", speed_read);
            DebugSerialUtilitiesLn2("  -  Encoder whole is: ", encoderTotal);
            DebugSerialUtilitiesLn2("  -  Time spent is (in s): ", time_whole);
            DebugSerialSeparation(1);
            encoderFlag = 1;
            if (encoderTotal == encoderFullRotation ) {
                sendSetWheelSpeed(0, CW);
                break;
            }
        }
        if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
    }
}

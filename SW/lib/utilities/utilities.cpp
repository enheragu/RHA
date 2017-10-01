/**
 * @file
 * @brief Implements a set of utilities to measure, experimentally, some interesting parameters.
 *
 * Measures real speed of servo, time spent with packet handling, etc
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_13
 * @Project: RHA
 * @Filename: utilities.h
 * @Last modified by:   quique
 * @Last modified time: 30-Sep-2017
 */

#include "debug.h"
#include "servo_rha.h"
// #include "cytron_g15_servo.h"
#include "utilities.h"
#include "joint_handler.h"


    #define SPEED 800  // torque send to check speed
    #define SPEED_TARGET 80  // speed target in rpm
    #define KP_REGULATOR 150  // kp for extractRegulatorData function
    #define LOOP_FREQUENCY 100  // in ms
    #define BAUD_RATE_G15 57600

    #define CHAUVENET_REPETITIONS 50  // too many repetitions cause memory overfload
    #define KN 1.54 // Chauvenet coeficient for n = 4

namespace MeasureUtilities {

    /**
     * @brief Calculates the average applying chauvenets criterion
     * @method averageChauvenet
     * @param  data data to calculate the average
     * @param  n amount of data (max of 255)
     * @return  Returns the average
     */
    void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation) {
        DebugSerialUtilitiesLn("averageChauvenet: begin of function");
        float values_kept [n];
        uint8_t n_values = 0;
        for (uint8_t i = 0; i < n; i++) arithmetic_average += data[i];
        arithmetic_average /= (float)n;
        //DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated: ", arithmetic_average);
        for (uint8_t i = 0; i < n; i++) standard_deviation += pow((data[i]-arithmetic_average), 2);
        //DebugSerialUtilitiesLn2("averageChauvenet: standard_deviation al cuadrado: ", standard_deviation);
        standard_deviation = sqrt(standard_deviation / (n-1)); // n-1 is used due to Bessel correction
        //DebugSerialUtilitiesLn2("averageChauvenet: standard deviation calculated: ", standard_deviation);
        // Data with more than 2 times standar deviation from average are discarded
        for (uint8_t i = 0; i < n; i++){
            if(abs(data[i]-arithmetic_average) > KN*standard_deviation){
                //DebugSerialUtilitiesLn4("Value discarded:", data[i], ", in position: ", n);
            }
            else{
                values_kept[n_values] = data[i];
                n_values++;
            }
        }
        for (uint8_t i = 0; i < n_values; i++) arithmetic_average += values_kept[i]; // Once all the values have been discarded the average is made again
        arithmetic_average /= n_values;
        //DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated after discard values: ", arithmetic_average);
    }


} //end of MeasureUtilities namespace




void JHUtilitiesJH::initJoints(uint8_t joint_to_test) {
    DebugSerialUtilitiesLn("initJoints: begin of function");
    ServoRHA servo_broadcast(ALL_SERVO);

    uint8_t buffer[BUFFER_LEN], txBuffer[BUFFER_LEN];

    servo_broadcast.pingToPacket(buffer);
    JointHandler::warpSinglePacket(iPING, buffer, txBuffer);
    error_ = sendPacket(txBuffer);
    IDcurrent_ = txBuffer[0];
    DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent_);
    if (error_ != 0 && error_ != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
        DebugSerialJHLn4Error(error_, ALL_SERVO);
        DebugSerialUtilitiesLn("Error in servo comunication, end of initJoints");
        //return;
    }
    joint_[joint_to_test].init(IDcurrent_, CW, A0);

    JointHandler::sendExitWheelModeAll();
    delay(500);
    JointHandler::sendSetTorqueLimitAll(1023);
    delay(25);

    JointHandler::sendSetWheelModeAll();
}


/**
 * @brief extractRegulatorData tests ServoRHA regulator and prints through serial monitor all info (python list style) to make a graphic. It can test one servo. Autodetects ID of the one connected
 * @method extractRegulatorData
 */
void JHUtilitiesJH::extractRegulatorData(uint8_t joint_to_test) {
    float kp_samples[SAMPLE_KP] = KP_SAMPLES;  // macro defined in the top of utilities.h
    float ki_samples[SAMPLE_KP] = KI_SAMPLES;  // macro defined in the top of utilities.h
    float kd_samples[SAMPLE_KP] = KD_SAMPLES;  // macro defined in the top of utilities.h


    RHATypes::SpeedGoal speed_goal(joint_[joint_to_test].servo_.getID(),SPEED_REGULATOR_TEST,0,CW);  // Id, speed, speed_slope
    setSpeedGoal(speed_goal);
    for (uint8_t samples = 0; samples < SAMPLE_KP; samples++) {
        joint_[joint_to_test].servo_.speed_regulator_.setKRegulator(kp_samples[samples],ki_samples[samples],kd_samples[samples]);  // KP_SAMPLES(a) defined in the top of utilities.h
        Serial.print("n_data"); Serial.print(samples); Serial.print(" = "); Serial.println(SAMPLE_REGULATOR);
        Serial.print("speed_target"); Serial.print(samples); Serial.print("  = "); Serial.println(joint_[joint_to_test].servo_.getSpeedTarget());
        Serial.print("regulator_offset"); Serial.print(samples); Serial.print("  = "); Serial.println(TORQUE_OFFSET);
        Serial.print("regulator_prealimentation"); Serial.print(samples); Serial.print("  = "); Serial.println(TORQUE_PREALIMENTATION);
        Serial.print("regulatorTest"); Serial.print(samples); Serial.print("  = [");
        Serial.print("['"); Serial.print(joint_[joint_to_test].servo_.speed_regulator_.getKp()); Serial.print("','"); Serial.print(joint_[joint_to_test].servo_.speed_regulator_.getKi()); Serial.print("','"); Serial.print(joint_[joint_to_test].servo_.speed_regulator_.getKd());
        Serial.print("']");

        JointHandler::sendSetWheelModeAll();

        for (int counter = 0; counter < SAMPLE_REGULATOR; counter++) {
            unsigned long time_init = millis();
            JointHandler::controlLoop();

            Serial.print(",['"); Serial.print(joint_[joint_to_test].servo_.getSpeed()); Serial.print("','");
            Serial.print(time_init);
            Serial.print("','");
            Serial.print(joint_[joint_to_test].servo_.getGoalTorque() & ~0x0400);
            Serial.println("']\\");

            if(joint_[joint_to_test].servo_.getError() != 0) {
                DebugSerialJHLn4Error(joint_[joint_to_test].servo_.getCommError(), joint_[joint_to_test].servo_.getID());
                // return;
            }
        }


        JointHandler::sendExitWheelModeAll();
        delay(1000);

        Serial.println("]");
    }
    return;
}


void JHUtilitiesJH::extractStepInputData(uint8_t joint_to_test) {

    Serial.println("stepTest = []");
    Serial.print("n_samples_step"); Serial.print(" = "); Serial.println(SAMPLE_TEST_STEP);
    Serial.print("n_data_step"); Serial.print(0); Serial.print(" = "); Serial.println(SAMPLE_STEP);
    for (uint8_t samples = 0; samples < SAMPLE_TEST_STEP; samples++) {
        Serial.print("stepTest.append(");  Serial.print(" [0");
        JointHandler::sendSetWheelModeAll();
        JointHandler::updateJointInfo();
        for (int counter = 0; counter < SAMPLE_STEP; counter++) {
            unsigned long time_init = millis();
            JointHandler::updateJointInfo();
            JointHandler::sendSetWheelSpeedAll(STEP_SPEED,CW);
            /*uint8_t buffer[BUFFER_LEN];
            uint8_t txBuffer[BUFFER_LEN];
            uint8_t buffer_to_send[BUFFER_LEN];
            joint_[joint_to_test].servo_.setWheelSpeedToPacket(buffer,STEP_SPEED,CW);
            uint8_t num_bytes = 0, num_servo = 0;
            num_servo++;
            num_bytes = addToSyncPacket(buffer_to_send, buffer, num_bytes);
            JointHandler::warpSyncPacket(buffer_to_send, buffer[2], txBuffer, num_bytes, num_servo);
            uint16_t error = JointHandler::sendPacket(txBuffer);
            if(error != 0) {
                DebugSerialJHLn4Error(joint_[joint_to_test].servo_.getCommError(), joint_[joint_to_test].servo_.getID());
                // return;
            }*/
            Serial.print(",['"); Serial.print(joint_[joint_to_test].servo_.getSpeed()); Serial.print("','"); Serial.print(STEP_SPEED); Serial.print("','");
            Serial.print(time_init); Serial.println("']\\");
        }

        Serial.println(" ] ) ");
        JointHandler::sendExitWheelModeAll();
        delay(2000);
    }

    return;
}


void JHUtilitiesJH::extractSlopeInputData(uint8_t joint_to_test) {

    Serial.println("slopeTest = []");
    Serial.print("n_samples_slope"); Serial.print(" = "); Serial.println(SAMPLE_TEST_SLOPE);
    Serial.print("n_data_slope"); Serial.print(0); Serial.print(" = "); Serial.println(SAMPLE_SLOPE);
    JointHandler::updateJointInfo();
    for (uint8_t samples = 0; samples < SAMPLE_TEST_SLOPE; samples++) {
        JointHandler::sendSetWheelModeAll();
        Serial.print("slopeTest.append("); Serial.print(" [0");
        unsigned long time_init = 0;
        time_init = millis();
        for (int counter = 0; counter < SAMPLE_SLOPE; counter++) {
            int torque = SLOPE_SPEED*(millis() - time_init);
            if (torque > 1023) torque = 1023;
            JointHandler::updateJointInfo();
            JointHandler::sendSetWheelSpeedAll(torque,CW);
            Serial.print(",['"); Serial.print(joint_[joint_to_test].servo_.getSpeed()); Serial.print("','"); Serial.print(torque); Serial.print("','");
            Serial.print(millis()); Serial.println("']\\");
            delay(100);
        }

        Serial.println("] )");
        JointHandler::sendExitWheelModeAll();
        delay(2000);
    }

    return;
}

/**
 * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::updateInfo() . It can test one servo. Autodetects ID of the one connected
 * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions). Max of 255 (danger of memory overload)
 * @see checkTimeSpeedRead(). Both are used together to compare speed rate in comunication.
 * @see averageChauvenet()
 */
void JHUtilitiesJH::checkTimeGetInfo(uint8_t repetitions, uint8_t joint_to_test) {
    DebugSerialSeparation(1);

    uint64_t initTime = 0;
    uint32_t timeSpent [repetitions];

    DebugSerialUtilitiesLn("Begin of loop to take data");
    for (uint8_t i = 0; i < repetitions; i++){
        initTime = millis();
        JointHandler::updateJointInfo();
        timeSpent[i] = millis()-initTime;
        if (joint_[joint_to_test].servo_.getError() != 0){
            DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
            DebugSerialJHLn4Error(joint_[joint_to_test].servo_.getError(), joint_[joint_to_test].servo_.getID());
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
void JHUtilitiesJH::checkComSucces(uint16_t repetitions) {
    DebugSerialUtilitiesLn("checkComSucces: begin of function");
    DebugSerialSeparation(1);

    uint16_t succes_ping = 0;
    for (uint16_t i = 0; i < repetitions; i++)
    {

        if (JointHandler::checkConectionAll()) succes_ping++;
        delay(5);
    }
    DebugSerialUtilitiesLn2("checkComSucces: Success total (ping): ", succes_ping);
    DebugSerialUtilitiesLn2("checkComSucces: number of repetitions: ", repetitions);
    DebugSerialSeparation(1);
}


/**
 * @brief checkSpeed implements an encoder mode to measure real speed in RPM and check agains the measure returned by servo and torque value sent. It can test one servo. Autodetects ID of the one connected
 */
void JHUtilitiesJH::checkSpeed(uint8_t joint_to_test){
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

    JointHandler::sendSetTorqueLimitAll(torque_set);
    delay(25);
    JointHandler::sendSetWheelModeAll();
    delay(25);
    delay(25);

    JointHandler::updateJointInfo();            //get the current position from servo
    pos = joint_[joint_to_test].servo_.getPosition();
    encoderTemp = pos;
    encoderFlag = 1;
    initTime = millis();

    DebugSerialUtilitiesLn("Configuration done.");
    delay(2000);
    DebugSerialUtilitiesLn("Start moving");

    JointHandler::sendSetWheelSpeedAll(speed_set, CW);
    delay(25);

    while (1) {
        //DebugSerialUtilitiesLn("Init while loop");
        updateJointInfo();            //get the current position from servo
        pos = joint_[joint_to_test].servo_.getPosition();
        encoderCurrent = pos;
        //DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

        if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
            encoderTotal++;
            currentTime = millis();
            float time_whole = ((float)(currentTime - initTime) / 1000);
            float speed_now = ((float)encoderTotal / (float)time_whole)*60;
            JointHandler::updateJointInfo();            //get the current position from servo
            float speed_read = joint_[joint_to_test].servo_.getSpeed();
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
                JointHandler::sendSetWheelSpeedAll(0, CW);
                break;
            }
        }
        if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
    }
}

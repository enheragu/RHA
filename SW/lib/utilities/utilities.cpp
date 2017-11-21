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
 * @Last modified time: 29-Oct-2017
 */

#include "debug.h"
#include "servo_rha.h"
// #include "cytron_g15_servo.h"
#include "utilities.h"
#include "joint_handler.h"


#define SPEED_TARGET 80  // speed target in rpm
#define KP_REGULATOR 150  // kp for extractRegulatorData function
#define LOOP_FREQUENCY 100  // in ms
#define BAUD_RATE_G15 460800

#define CHAUVENET_REPETITIONS 50  // too many repetitions cause memory overfload
#define KN 1.54  // Chauvenet coeficient for n = 4

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
        float values_kept[n];
        uint8_t n_values = 0;
        uint8_t i = 0;

        for (i = 0; i < n; i++) arithmetic_average += data[i];
        arithmetic_average /= (float)n;
        // DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated: ", arithmetic_average);
        for (i = 0; i < n; i++) standard_deviation += pow((data[i]-arithmetic_average), 2);
        // DebugSerialUtilitiesLn2("averageChauvenet: standard_deviation al cuadrado: ", standard_deviation);
        standard_deviation = sqrt(standard_deviation / (n-1));  // n-1 is used due to Bessel correction
        // DebugSerialUtilitiesLn2("averageChauvenet: standard deviation calculated: ", standard_deviation);
        // Data with more than 2 times standar deviation from average are discarded
        for (i = 0; i < n; i++) {
            if (abs(data[i]-arithmetic_average) > KN*standard_deviation) {
                // DebugSerialUtilitiesLn4("Value discarded:", data[i], ", in position: ", n);
            } else {
                values_kept[n_values] = data[i];
                n_values++;
            }
        }
        for (i = 0; i < n_values; i++) arithmetic_average += values_kept[i];  // Once all the values have been discarded the average is made again
        arithmetic_average /= n_values;
        // DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated after discard values: ", arithmetic_average);
    }
}  // namespace MeasureUtilities




void JHUtilitiesJH::initJoints(uint8_t _joint_to_test) {
    DebugSerialUtilitiesLn("initJoints: begin of function");
    ServoRHA servo_broadcast(ALL_SERVO);

    uint8_t buffer[BUFFER_LEN], txBuffer[BUFFER_LEN];

    servo_broadcast.pingToPacket(buffer);
    JointHandler::warpSinglePacket(iPING, buffer, txBuffer);
    error_ = sendPacket(txBuffer);
    IDcurrent_ = txBuffer[0];
    DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent_);
    if (error_ != 0 && error_ != SERROR_IDMISMATCH) {  // Ignore ID mismatch as we broadcast to all servo
        DebugSerialJHLn4Error(error_, ALL_SERVO);
        DebugSerialUtilitiesLn("Error in servo comunication, end of initJoints");
        // return;
    }
    joint_[_joint_to_test].init(IDcurrent_, CW, A0);

    /*JointHandler::sendExitWheelModeAll();
    delay(500);
    JointHandler::sendSetTorqueLimitAll(1023);
    delay(25);

    JointHandler::sendSetWheelModeAll();*/
}


/**
 * @brief extractRegulatorData tests ServoRHA regulator and prints through serial monitor all info (python list style) to make a graphic. It can test one servo. Autodetects ID of the one connected
 * @method extractRegulatorData
 */
void JHUtilitiesJH::extractRegulatorData(uint8_t _joint_to_test) {
    float kp_samples[SAMPLE_KP] = KP_SAMPLES;  // macro defined in the top of utilities.h
    float ki_samples[SAMPLE_KP] = KI_SAMPLES;  // macro defined in the top of utilities.h
    float kd_samples[SAMPLE_KP] = KD_SAMPLES;  // macro defined in the top of utilities.h
    uint8_t samples = 0;
    int counter = 0;
    uint64_t time_init = 0;

    RHATypes::SpeedGoal speed_goal(joint_[_joint_to_test].servo_.getID(), SPEED_REGULATOR_TEST, CW);  // Id, speed, speed_slope
    setSpeedGoal(speed_goal);
    for (uint8_t samples = 0; samples < SAMPLE_KP; samples++) {
        joint_[joint_to_test].servo_.speed_regulator_.setKRegulator(kp_samples[samples],ki_samples[samples],kd_samples[samples]);  // KP_SAMPLES(a) defined in the top of utilities.h
        output("n_data"); output(samples); output(" = "); outputln(SAMPLE_REGULATOR);
        output("speed_target"); output(samples); output("  = "); outputln(joint_[joint_to_test].servo_.getSpeedTarget());
        output("regulator_offset"); output(samples); output("  = "); outputln(TORQUE_OFFSET);
        output("regulator_prealimentation"); output(samples); output("  = "); outputln(TORQUE_PREALIMENTATION);
        output("regulatorTest"); output(samples); output("  = [");
        output("['"); output(joint_[joint_to_test].servo_.speed_regulator_.getKp()); output("','"); output(joint_[joint_to_test].servo_.speed_regulator_.getKi()); output("','"); output(joint_[joint_to_test].servo_.speed_regulator_.getKd());
        output("']");

        JointHandler::sendSetWheelModeAll();

        for (counter = 0; counter < SAMPLE_REGULATOR; counter++) {
            time_init = millis();
            JointHandler::controlLoopTorque();
<<<<<<< HEAD
=======

>>>>>>> Raspberry adaptation first commit
            output(",['"); output(joint_[joint_to_test].servo_.getSpeed()); output("','");
            output(time_init);
            output("','");
            output(joint_[joint_to_test].servo_.getGoalTorque() & ~0x0400);
            outputln("']\\");

            if (joint_[_joint_to_test].servo_.getError() != 0) {
                //DebugSerialJHLn4Error(joint_[_joint_to_test].servo_.getCommError(), joint_[_joint_to_test].servo_.getID());
                // return;
            }
        }


        JointHandler::sendExitWheelModeAll();
        delay(1000);

        outputln("]");
    }
    return;
}

void blinkLed(uint8_t pin_led, uint8_t time_blink) {
    RHATypes::Timer blink_period;
    blink_period.setTimer(time_blink);
    blink_period.activateTimer();
    uint8_t i = 0;
    for (i = 0; i < 10; i++) {
            blink_period.checkWait();
            blink_period.activateTimer();
            digitalWrite(pin_led, HIGH);
            blink_period.checkWait();
            blink_period.activateTimer();
            digitalWrite(pin_led, LOW);
    }
}

void testOnProcess(uint8_t true_false) {
    if (true_false) {
        digitalWrite(LED_ROJO, HIGH);
        digitalWrite(LED_VERDE, LOW);
    } else if (!true_false) {
        digitalWrite(LED_VERDE, HIGH);
        digitalWrite(LED_ROJO, LOW);
    }
}

void JHUtilitiesJH::resetEncoder() {
    encoderTemp_ = 0;
    encoderCurrent_ = 0;
    encoderTotal_ = 0;
    encoderFlag_ = 1;
    angleEndPositionFlag_ = 0;
    pos_ = 0;
}

#define ENCODER_ANGLE_MARGIN 15

void JHUtilitiesJH::updateEncoder(uint8_t _joint_to_test) {
    pos_ = joint_[_joint_to_test].servo_.getPosition();
<<<<<<< HEAD
    // printf("Pos read from servo is: "); printfln(pos_);
=======
    // output("Pos read from servo is: "); outputln(pos_);
>>>>>>> Raspberry adaptation first commit
    encoderCurrent_ = pos_;
    if (encoderCurrent_ < (encoderTemp_ + ENCODER_ANGLE_MARGIN)  && encoderCurrent_ > (encoderTemp_ - ENCODER_ANGLE_MARGIN) && encoderFlag_ == 0) {
        encoderTotal_++;
        encoderFlag_ = 1;
    }
    if (encoderCurrent_ > (encoderTemp_ + ENCODER_ANGLE_MARGIN) || encoderCurrent_ < (encoderTemp_ - ENCODER_ANGLE_MARGIN)) encoderFlag_ = 0;
}

void JHUtilitiesJH::startEncoder(uint8_t _joint_to_test) {
    pos_ = joint_[_joint_to_test].servo_.getPosition();
    encoderTemp_ = pos_;
    encoderFlag_ = 1;
}

void JHUtilitiesJH::returnToStartPositionTest(uint8_t _joint_to_test, uint8_t direction) {
    unsigned int turns_to_undo = encoderTotal_;

    int angle_end_position = 0;
    if (!(encoderCurrent_ < (encoderTemp_ + ENCODER_ANGLE_MARGIN)  && encoderCurrent_ > (encoderTemp_ - ENCODER_ANGLE_MARGIN))) {
        angle_end_position = encoderCurrent_ - encoderTemp_;
        if (angle_end_position > 1087) angle_end_position -= 1087;
        else if (angle_end_position < 0) angle_end_position += 1087;
    }

<<<<<<< HEAD
    // printf("## Turns made during test to be undone: "); printfln(turns_to_undo);
    // printf("## Angle last position: "); printfln(angle_end_position);
=======
    // output("## Turns made during test to be undone: "); outputln(turns_to_undo);
    // output("## Angle last position: "); outputln(angle_end_position);
>>>>>>> Raspberry adaptation first commit
    JointHandler::updateJointInfo();
    resetEncoder();
    startEncoder(_joint_to_test);

    JointHandler::sendSetWheelModeAll();
    delay(25);

    while (true) {
        JointHandler::updateJointInfo();
        updateEncoder(_joint_to_test);
        if (angleEndPositionFlag_ == 0 && (encoderTotal_ >= turns_to_undo || turns_to_undo == 0) && angle_end_position != 0) {
            resetEncoder();
            startEncoder(_joint_to_test);
            if (direction == CW)  // CW means down
                encoderTemp_ -= angle_end_position;  // this was -
            else if (direction == CCW)  // CCW means UP
                encoderTemp_ += angle_end_position;

            if (encoderTemp_ > 1087) encoderTemp_ -= 1087;
            else if (encoderTemp_ < 0) encoderTemp_ += 1087;

            turns_to_undo = 1;
            angleEndPositionFlag_ = 1;
            angle_end_position = 0;
<<<<<<< HEAD
            // printf("## Reset encoder to got to: "); printfln(encoderTemp_);
=======
            // output("## Reset encoder to got to: "); outputln(encoderTemp_);
>>>>>>> Raspberry adaptation first commit
        }
        else if ((encoderTotal_ >= turns_to_undo || turns_to_undo == 0) && angle_end_position == 0)
            break;
        if (direction == UP) {
            JointHandler::sendSetWheelSpeedAll(STEP_SPEED, direction);
        } else if (direction == DOWN) {
            JointHandler::sendSetWheelSpeedAll(STEP_SPEED/3, direction);
        }
        delay(25);

    }

    JointHandler::sendExitWheelModeAll();
    delay(25);
}


void JHUtilitiesJH::extractStepSlopeData(uint8_t _joint_to_test, uint8_t _option) {
    pinMode(LED_ROJO, OUTPUT);
    pinMode(LED_VERDE, OUTPUT);
    pinMode(PULSADOR, INPUT);

    unsigned int num_test, num_repetitions;
    if (_option == STEP) {
<<<<<<< HEAD
        printfln("##  ===== START OF DATA SET, STEP TEST ===== ");
        printf("## n_samples_step"); printf(" = "); printfln(SAMPLE_TEST_STEP);
        printf("## n_data_step"); printf(0); printf(" = "); printfln(SAMPLE_STEP);
        num_repetitions = SAMPLE_STEP;
        num_test = SAMPLE_TEST_STEP;
    } else if (_option == SLOPE) {
        printfln("##  ===== START OF DATA SET, SLOPE TEST ===== ");
        printf("## n_samples_slope"); printf(" = "); printfln(SAMPLE_TEST_SLOPE);
        printf("## n_data_slope"); printf(0); printf(" = "); printfln(SAMPLE_SLOPE);
=======
        outputln("##  ===== START OF DATA SET, STEP TEST ===== ");
        output("## n_samples_step"); output(" = "); outputln(SAMPLE_TEST_STEP);
        output("## n_data_step"); output(0); output(" = "); outputln(SAMPLE_STEP);
        num_repetitions = SAMPLE_STEP;
        num_test = SAMPLE_TEST_STEP;
    } else if (_option == SLOPE) {
        outputln("##  ===== START OF DATA SET, SLOPE TEST ===== ");
        output("## n_samples_slope"); output(" = "); outputln(SAMPLE_TEST_SLOPE);
        output("## n_data_slope"); output(0); output(" = "); outputln(SAMPLE_SLOPE);
>>>>>>> Raspberry adaptation first commit
        num_repetitions = SAMPLE_SLOPE;
        num_test = SAMPLE_TEST_SLOPE;
    }

    uint8_t direction = DOWN;
    uint8_t back_direction = UP;
<<<<<<< HEAD
    printfln("## This test is performed with 0.5 kg, in favour");
    printfln("## Data printed is, on each column: speed, torque sent, and time");
=======
    outputln("## This test is performed with 0.5 kg, in favour");
    outputln("## Data printed is, on each column: speed, torque sent, and time");
>>>>>>> Raspberry adaptation first commit

    testOnProcess(false);
    blinkLed(LED_VERDE, 100);
    JointHandler::sendSetWheelModeAll();
    delay(25);
    while (!digitalRead(PULSADOR)) {
        delay(25);
<<<<<<< HEAD
        // printf(digitalRead(PULSADOR));
=======
        // output(digitalRead(PULSADOR));
>>>>>>> Raspberry adaptation first commit
    }
    delay(1000);
    while (!digitalRead(PULSADOR)) {
        if (back_direction == UP) {
            JointHandler::sendSetWheelSpeedAll(STEP_SPEED, back_direction);
        } else if (back_direction == DOWN) {
            JointHandler::sendSetWheelSpeedAll(STEP_SPEED/3, back_direction);
        }
        delay(25);
<<<<<<< HEAD
        // printf(digitalRead(PULSADOR));
=======
        // output(digitalRead(PULSADOR));
>>>>>>> Raspberry adaptation first commit
    }
    JointHandler::sendExitWheelModeAll();
    delay(25);
    delay(1000);
    while (!digitalRead(PULSADOR)) {
        delay(25);
<<<<<<< HEAD
        // printf(digitalRead(PULSADOR));
=======
        // output(digitalRead(PULSADOR));
>>>>>>> Raspberry adaptation first commit
    }
    blinkLed(LED_ROJO, 100);
    testOnProcess(true);

    JointHandler::updateJointInfo();

    uint64_t time_init = micros();
    uint64_t time_now = 0;
    uint8_t samples = 0;
    unsigned int counter = 0;

    for ( samples = 0; samples < num_test; samples++) {
<<<<<<< HEAD
        // printf("stepTest.append(");  printf(" [0");
=======
        // output("stepTest.append(");  output(" [0");
>>>>>>> Raspberry adaptation first commit
        JointHandler::sendSetWheelModeAll();
        delay(25);
        JointHandler::updateJointInfo();
        if (_option == STEP) {
<<<<<<< HEAD
            printf("Step(:,:,"); printf(samples+1); printf(") = [");
        } else if (_option == SLOPE) {
            printf("Slope(:,:,"); printf(samples+1); printf(") = [");
=======
            output("Step(:,:,"); output(samples+1); output(") = [");
        } else if (_option == SLOPE) {
            output("Slope(:,:,"); output(samples+1); output(") = [");
>>>>>>> Raspberry adaptation first commit
        }
        time_init = micros();
        time_now = 0;

        RHATypes::Timer measure_period;
        if (_option == STEP) measure_period.setTimer(3);
        else if (_option == SLOPE) measure_period.setTimer(10);

        unsigned int torque;

        resetEncoder();
        startEncoder(_joint_to_test);

        for (counter = 0; counter < num_repetitions; counter++) {
            if (_option == STEP) torque = STEP_SPEED;
            else if (_option == SLOPE) torque = SLOPE_SPEED*(millis() - time_init/1000);
            if (torque > 1023) torque = 1023;
            JointHandler::updateJointInfo();
            JointHandler::sendSetWheelSpeedAll(torque, direction);
<<<<<<< HEAD
            if (counter != 0)  printf(";");
            time_now = micros() - time_init;
            printf(" "); printf(joint_[_joint_to_test].servo_.getSpeed()); printf(" "); printf(torque); printf(" ");
            printf((unsigned long)time_now);  // printfln("']\\");
=======
            if (counter != 0)  output(";");
            time_now = micros() - time_init;
            output(" "); output(joint_[_joint_to_test].servo_.getSpeed()); output(" "); output(torque); output(" ");
            output((unsigned long)time_now);  // outputln("']\\");
>>>>>>> Raspberry adaptation first commit
            // measure_period.checkWait();
            while (!measure_period.checkContinue()) {  // while time has not been finished
                JointHandler::updateJointInfo();
                updateEncoder(_joint_to_test);
            }
            measure_period.activateTimer();
        }
        double average_time = (time_now / num_repetitions);
<<<<<<< HEAD
        printfln("];");
        printf("## Average time between each mesaure = "); printfln(average_time);
        printfln("##  ===== END OF DATA SET ===== ");
=======
        outputln("];");
        output("## Average time between each mesaure = "); outputln(average_time);
        outputln("##  ===== END OF DATA SET ===== ");

>>>>>>> Raspberry adaptation first commit
        JointHandler::sendExitWheelModeAll();
        delay(25);

        returnToStartPositionTest(_joint_to_test, back_direction);

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
void JHUtilitiesJH::checkTimeGetInfo(uint8_t repetitions, uint8_t _joint_to_test) {
    DebugSerialSeparation(1);

    uint64_t initTime = 0;
    uint32_t timeSpent[repetitions];
    uint8_t i = 0;

    DebugSerialUtilitiesLn("Begin of loop to take data");
    for (i = 0; i < repetitions; i++) {
        initTime = millis();
        JointHandler::updateJointInfo();
        timeSpent[i] = millis()-initTime;
        if (joint_[_joint_to_test].servo_.getError() != 0) {
            DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
            DebugSerialJHLn4Error(joint_[_joint_to_test].servo_.getError(), joint_[_joint_to_test].servo_.getID());
            return;
        }
    }
    DebugSerialUtilitiesLn("Data taken, calling averageChauvenet()");
    float average_time = 0, standard_deviation = 0;
    MeasureUtilities::averageChauvenet(timeSpent, repetitions, average_time, standard_deviation);
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
    uint16_t i = 0;
    for (i = 0; i < repetitions; i++) {
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
void JHUtilitiesJH::checkSpeed(uint8_t _joint_to_test) {
    DebugSerialUtilitiesLn("checkSpeed: begin of function");

    uint32_t encoderTemp = 0,
         encoderCurrent = 0,
         encoderFullRotation = 500,
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

    JointHandler::updateJointInfo();            // get the current position from servo
    pos = joint_[_joint_to_test].servo_.getPosition();
    encoderTemp = pos;
    encoderFlag = 1;
    initTime = millis();

    DebugSerialUtilitiesLn("Configuration done.");
    delay(2000);float time_whole
    DebugSerialUtilitiesLn("Start moving");

    JointHandler::sendSetWheelSpeedAll(speed_set, CW);
    delay(25);

    float time_whole = 0;
    float speed_now = 0;
    float speed_read = 0;

    while (1) {
        // DebugSerialUtilitiesLn("Init while loop");
        updateJointInfo();            // get the current position from servo
        pos = joint_[_joint_to_test].servo_.getPosition();
        encoderCurrent = pos;
        // DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

        if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
            encoderTotal++;
            currentTime = millis();
            time_whole = ((float)(currentTime - initTime) / 1000);
            speed_now = ((float)encoderTotal / (float)time_whole)*60;
            JointHandler::updateJointInfo();            // get the current position from servo
            speed_read = joint_[_joint_to_test].servo_.getSpeed();
            DebugSerialSeparation(1);
            DebugSerialUtilitiesLn2("  -  Torque set is: ", speed_set);
            DebugSerialUtilitiesLn2("  -  Speed calculated is (in rpm): ", speed_now);
            DebugSerialUtilitiesLn2("  -  Speed calculated is (in rad/s): ", speed_now*(2*PI/60));
            // DebugSerialUtilitiesLn2("  -  Speed read is (in rpm): ", speed_read*(2*PI/60));
            DebugSerialUtilitiesLn2("  -  Speed read is (directly from register): ", speed_read);
            DebugSerialUtilitiesLn2("  -  Encoder whole is: ", encoderTotal);
            DebugSerialUtilitiesLn2("  -  Time spent is (in s): ", time_whole);
            DebugSerialSeparation(1);
            encoderFlag = 1;
            if (encoderTotal == encoderFullRotation) {
                JointHandler::sendSetWheelSpeedAll(0, CW);
                break;
            }
        }
        if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
    }
}

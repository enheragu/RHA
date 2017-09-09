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
 * @Last modified by:   enheragu
 * @Last modified time: 10-Sep-2017
 */

#include <Arduino.h>
#include "debug.h"
#include "servo_rha.h"
#include "cytron_g15_servo.h"

/**
  * @brief Analyses error and prints error msgs
  */
void printServoStatusError (uint16_t error){
    // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
    if (error & SERROR_PING)  {DebugSerialUtilitiesLn("Ping error in servo");}
    if (error & SERROR_INPUTVOLTAGE)  {DebugSerialUtilitiesLn("Input voltage error in servo");}          // bit 0
    if (error & SERROR_ANGLELIMIT)  {DebugSerialUtilitiesLn("Angle limit error in servo");}           // bit 1
    if (error & SERROR_OVERHEATING)  {DebugSerialUtilitiesLn("Overheating error in servo");}          // bit 2
    if (error & SERROR_RANGE)  {DebugSerialUtilitiesLn("Range error in servo");}             // bit 3
    if (error & SERROR_CHECKSUM)  {DebugSerialUtilitiesLn("Checksum error in servo");}             // bit 4
    if (error & SERROR_OVERLOAD)  {DebugSerialUtilitiesLn("Overload error in servo");}             // bit 5
    if (error & SERROR_INSTRUCTION)  {DebugSerialUtilitiesLn("Instruction error in servo");}            // bit 7
    if (error & SERROR_PACKETLOST)  {DebugSerialUtilitiesLn("Packet lost or receive time out in servo");}    // bit 8
    if (error & SERROR_WRONGHEADER)  {DebugSerialUtilitiesLn("Wrong header in servo");}              // bit 9
    if (error & SERROR_IDMISMATCH)  {DebugSerialUtilitiesLn("ID mismatch in servo");}                  // bit 10
    if (error & SERROR_CHECKSUMERROR)  {DebugSerialUtilitiesLn("Checksum error in servo");}               // bit 13
}

namespace MeasureUtilities{

    #define SPEED 800  // torque send to check speed
    #define SPEED_TARGET 10  // speed target in rpm
    #define KP_REGULATOR 100/60  // kp for extractRegulatorData function
    #define LOOP_FREQUENCY 100  // in ms
    #define SAMPLE_REGULATOR 50

    /**
     * @brief checkSpeed implements an encoder mode to measure real speed in RPM and check agains the measure returned by servo and torque value sent. It can test one servo. Autodetects ID of the one connected
     */
    void checkSpeed(){
        DebugSerialUtilitiesLn("checkSpeed: begin of function");
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkSpeed")
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        long encoderTemp = 0,
             encoderCurrent = 0,
             encoderFullRotation = 100,
             encoderTotal = 0;

        int speed_set = SPEED,
            torque_set = 1023;

        char encoderFlag = 0;

        word pos = 0;

        long initTime = 0;
        long currentTime = 0;

        error = servo_test1.setTorqueLimit( torque_set);
        delay(25);
        printServoStatusError(error);
        servo_test1.setWheelMode();
        delay(25);
        servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);
        delay(25);

        servo_test1.getPos(data);             //get the current position from servo
        pos = data[0];
        pos = pos | ((data[1]) << 8);
        encoderTemp = pos;
        encoderFlag = 1;
        initTime = millis();

        DebugSerialUtilitiesLn("Configuration done.");
        delay(2000);
        DebugSerialUtilitiesLn("Start moving");

        error = servo_test1.setTorqueOnOff(ON, iREG_WRITE);
        delay(25);
        printServoStatusError(error);
        error = servo_test1.setWheelSpeed(speed_set, CW, iWRITE_DATA);
        delay(25);
        printServoStatusError(error);

        while (1) {
            //DebugSerialUtilitiesLn("Init while loop");
            servo_test1.getPos(data); //get the current position from servo
            pos = data[0];
            pos = pos | ((data[1]) << 8);
            encoderCurrent = pos;
            //DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

            if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
                encoderTotal++;
                currentTime = millis();
                double time_whole = ((currentTime - initTime) / 1000);
                double speed_now = ((double)encoderTotal / (double)time_whole)*60;
                double speed_read = servo_test1.speedRead();
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
                    servo_test1.setTorqueOnOff(ON, iREG_WRITE);
                    servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);
                    break;
                }
            }
            if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
        }
    }


    #define KN 1.54 // Chauvenet coeficient for n = 4

    /**
     * @brief Calculates the average applying chauvenets criterion
     * @method averageChauvenet
     * @param  data data to calculate the average
     * @param  n amount of data
     * @return  Returns the average
     */
    double averageChauvenet(long *data, long n){
        DebugSerialUtilitiesLn("averageChauvenet: begin of function");
        double arithmetic_average = 0, standard_deviation = 0;
        double values_kept [n];
        long n_values = 0;
        for (int i = 0; i < n; i++) arithmetic_average += data[i];
        arithmetic_average /= (double)n;
        DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated: ", arithmetic_average);
        for (int i = 0; i < n; i++) standard_deviation += pow((data[i]-arithmetic_average), 2);
        DebugSerialUtilitiesLn2("averageChauvenet: standard_deviation²: ", standard_deviation);
        standard_deviation = sqrt(standard_deviation / (n-1)); // n-1 is used due to Bessel correction
        DebugSerialUtilitiesLn2("averageChauvenet: standard deviation calculated: ", standard_deviation);
        // Data with more than 2 times standar deviation from average are discarded
        for (int i = 0; i < n; i++){
            if(abs(data[i]-arithmetic_average) > KN*standard_deviation){
                DebugSerialUtilitiesLn4("Value discarded:", data[i], ", in position: ", n);
            }
            else{
                values_kept[n_values] = data[i];
                n_values++;
            }
        }
        for (int i = 0; i < n_values; i++) arithmetic_average += values_kept[i]; // Once all the values have been discarded the average is made again
        arithmetic_average /= n_values;
        DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated after discard values: ", arithmetic_average);
        return arithmetic_average;
    }

    /**
     * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::updateInfo() . It can test one servo. Autodetects ID of the one connected
     * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions)
     * @see checkTimeSpeedRead(). Both are used together to compare speed rate in comunication.
     * @see averageChauvenet()
     */
    void checkTimeGetInfo(long repetitions){
        DebugSerialSeparation(1);
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkTimeGetInfo");
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        long initTime = 0;
        long timeSpent [repetitions];

        DebugSerialUtilitiesLn("Begin of loop to take data");
        for (int i = 0; i < repetitions; i++){
            initTime = millis();
            servo_test1.updateInfo();
            timeSpent[i] = millis()-initTime;
            if (servo_test1.getError() != 0){
                DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
                return;
            }
        }
        DebugSerialUtilitiesLn("Data taken, calling averageChauvenet()");
        double average_time = averageChauvenet(timeSpent,repetitions);
        DebugSerialUtilitiesLn2("checkTimeGetInfo: Time spent with ServoRHA::updateInfo: ", average_time);
        DebugSerialUtilitiesLn2("checkTimeGetInfo: number of repetitions: ", repetitions);
        DebugSerialUtilitiesLn("checkTimeSpeedRead: 11 bytes read");
        DebugSerialSeparation(1);
    }

    /**
      * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::SpeedRead(). It can test one servo. Autodetects ID of the one connected.
      * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions)
      * @see checkTimeGetInfo(). Both are used together to compare speed rate in comunication.
      * @see averageChauvenet()
      */
    void checkTimeSpeedRead(long repetitions){
        DebugSerialSeparation(1);
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkTimeSpeedRead");
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        long initTime = 0;
        long timeSpent [repetitions];

        DebugSerialUtilitiesLn("Begin of loop to take data");
        for (int i = 0; i < repetitions; i++){
            initTime = millis();
            servo_test1.speedRead();
            timeSpent[i] = millis()-initTime;
            if (servo_test1.getError() != 0){
                DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
                printServoStatusError(error);
                return;
            }
        }
        double average_time = averageChauvenet(timeSpent,repetitions);
        DebugSerialUtilitiesLn2("checkTimeSpeedRead: Time spent with ServoRHA::speedRead: ", average_time);
        DebugSerialUtilitiesLn2("checkTimeSpeedRead: number of repetitions: ", repetitions);
        DebugSerialUtilitiesLn("checkTimeSpeedRead: 2 bytes read");
        DebugSerialSeparation(1);
    }

    /**
     * @brief extractRegulatorData tests ServoRHA regulator and prints through serial monitor all info (python list style) to make a graphic. It can test one servo. Autodetects ID of the one connected
     * @method extractRegulatorData
     */
    void extractRegulatorData(){
        DebugSerialUtilitiesLn("extractRegulatorData: begin of function");
        DebugSerialSeparation(1);

        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            DebugSerialUtilitiesLn("Error in servo comunication, end of extractRegulatorData");
            printServoStatusError(error);
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        servo_test1.setWheelMode();
        servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);

        uint16_t torque = 0,
                 speed_current = 0,
                 speed_target = SPEED_TARGET,
                 error_speed = 0;
        unsigned long time_init = 0,
                      timer = LOOP_FREQUENCY;
        uint8_t counter = 0;

        time_init = millis();
        Serial.print("regulatorTest = [");
        Serial.print("['"); Serial.print(KP_REGULATOR); Serial.print("']");
        while(true){
            if (millis() - time_init > timer){
                speed_current = servo_test1.speedRead();
                error_speed = speed_target - speed_current;
                torque = servo_test1.regulatorServo(error_speed, KP_REGULATOR);
                servo_test1.setWheelSpeed(torque, CW, iWRITE_DATA);
                time_init = millis();

                Serial.print(",['"); Serial.print(speed_current); Serial.print("',");
                Serial.print(time_init); Serial.print("']");
                counter++;
                if(counter > SAMPLE_REGULATOR){
                    Serial.println("]");
                    DebugSerialSeparation(1);
                    DebugSerialUtilitiesLn("extractRegulatorData: end of function");
                    return;
                }
            }
        }
    }

} //end of MeasureUtilities

#define LED       13

/**
 * Function to change the ID of the servo connected to the bus (only one servo can be connected)
 * @method setServoId
 * @param  {uint8_t} new_id ID for the servo
 */
void setServoId(uint8_t new_id){
    Cytron_G15_Servo g15(ALL_SERVO, 2, 3, 8); // SoftwareSerial: Rx, Tx and Control pin
    //Cytron_G15Shield g15(8); // HardwareSerial: Control pin

    word error = 0;
    byte data[10];

    g15.begin(19200);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    delay(1000);
    g15.setID(new_id);

    error = g15.ping(data);

    if(error == 0 || error == 0x0400) {// Ignore ID mistmatch since broadcast ID is used to ping the servo
        if(data[0] == new_id) { // Succeed change to new ID!
            while(1) {
                g15.setLED(new_id, ON);
                delay(500);
                g15.setLED(new_id, OFF);
                delay(500);
            }
        } else {  // Fail, new ID is different
            while(1) {
                digitalWrite(LED, LOW);
                delay(500);
                digitalWrite(LED, HIGH);
                delay(500);
            }
        }
    } else {  // Fail, other error occur
        while(1) {
            digitalWrite(LED, LOW);
            delay(100);
            digitalWrite(LED, HIGH);
            delay(100);
        }
    }
}

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
 * @Last modified time: 08_Sep_2017
 */



#include <Arduino.h>
#include "debug.h"

/**
  * @brief Analyses error and prints error msgs
  */
void printServoStatusError (uint16_t error){

    if (error & SERROR_PING)  DebugSerialUtilitiesLn("Ping error in servo: ");
    if (error & SERROR_INPUTVOLTAGE)  DebugSerialUtilitiesLn("Input voltage error in servo: ")    ;          // bit 0
    if (error & SERROR_ANGLELIMIT)  DebugSerialUtilitiesLn("Angle limit error in servo: ")      ;           // bit 1
    if (error & SERROR_OVERHEATING)  DebugSerialUtilitiesLn("Overheating error in servo: ")     ;          // bit 2
    if (error & SERROR_RANGE)  DebugSerialUtilitiesLn("Range error in servo: ")               ;             // bit 3
    if (error & SERROR_CHECKSUM)  DebugSerialUtilitiesLn("Checksum error in servo: ")          ;             // bit 4
    if (error & SERROR_OVERLOAD)  DebugSerialUtilitiesLn("Overload error in servo: ")          ;             // bit 5
    if (error & SERROR_INSTRUCTION)  DebugSerialUtilitiesLn("Instruction error in servo: ");            // bit 7
    if (error & SERROR_PACKETLOST)  DebugSerialUtilitiesLn("Packet lost or receive time out in servo: ");    // bit 8
    if (error & SERROR_WRONGHEADER)  DebugSerialUtilitiesLn("Wrong header in servo: ")      ;              // bit 9
    if (error & SERROR_IDMISMATCH)  DebugSerialUtilitiesLn("ID mismatch in servo: ")       ;                  // bit 10
    if (error & SERROR_CHECKSUMERROR)  DebugSerialUtilitiesLn("Checksum error in servo: ")    ;               // bit 13
    else {
        DebugSerialUtilitiesLn2("Default case. Error: ", error);
    }
}


/**
  * @brief checkSpeed implements an encoder mode to measure real speed in RPM and check agains the measure returned by servo
  */
void checkSpeed(){
    ServoRHA servo_test1(1, 2, 3, 8);

    long encoderTemp = 0,
         encoderSmallRotation = 0,
         encoderFullRotation = 0,
         encoderCurrent = 0,
         encoderTotal = 0;

    int delay1 = 1000,
        selection = 0,
        key,
        speedSet = 0,
        speedRead,
        torqueSet = 0,
        i = 0;

    char flag = 1,
         encoderFlag = 0;

    word stat, pos = 0, load = 0,
               angleRead,
               CWSet = 0,
               CCWSet = 359,
               angleSet;

    byte data[10];

    long initTime = 0;
    long currentTime = 0;

    servo_test1.setTorqueLimit( torqueSet);
    servo_test1.setWheelMode();
    servo_test1.setWheelSpeed( 0, CCW);

    servo_test1.getPos(data);             //get the current position from servo
    pos = data[0];
    pos = pos | ((data[1]) << 8);
    encoderTemp = pos;
    DebugSerialUtilitiesLn("Configuration done.");
    delay(2000);
    DebugSerialUtilitiesLn("Start moving");

    uint16_t error;
    servo_test1.setTorqueOnOff(ON, iREG_WRITE);
    error = servo_test1.setWheelSpeed(speedSet, CCW);
    printServoStatusError(error);

    encoderFlag = 1;
    initTime = millis();
    while (1) {
        DebugSerialUtilitiesLn("Init while loop");
        servo_test1.getPos(data); //get the current position from servo
        pos = data[0];
        pos = pos | ((data[1]) << 8);
        encoderCurrent = pos;
        DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

        if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
            encoderTotal++;
            currentTime = millis();
            long speedNow = encoderTotal*1000/(initTime - currentTime);
            DebugSerialUtilitiesLn2("Torque set es: ", torqueSet);
            DebugSerialUtilitiesLn2("  -  Velocidad calculada es: ", speedNow);
            long speedGet = servo_test1.speedRead();
            DebugSerialUtilitiesLn2("  -  Velocidad obtenida es: ", speedGet);
            encoderFlag = 1;
            if (encoderTotal == encoderFullRotation ) {
                servo_test1.setTorqueOnOff(ON, iREG_WRITE);
                servo_test1.setWheelSpeed( 0, CCW);
                break;
            }
        }
        if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
    }
}

/**
  * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::updateInfo()
  * @param repetitions: num of repetitions the test is made (time is the average of this repetitions)
  * @see checkTimeSpeedRead(). Both are used together to compare speed rate in comunication.
  */
void checkTimeGetInfo(int repetitions){
    DebugSerialSeparation(1);
    ServoRHA servo_test1();
    servo_test1.init(1, 2, 3, 8, 19200);

    long initTime = 0;
    long timeSpent = 0;

    for int i = 0; i < repetitions; i++{
        initTime = millis();
        servo_test1.updateInfo();
        timeSpent += millis()-initTime;
    }
    timeSpent /= repetitions;
    DebugSerialUtilitiesLn2("checkTimeGetInfo: Time spent with ServoRHA::updateInfo: ", timeSpent);
    DebugSerialUtilitiesLn2("checkTimeGetInfo: with number of repetitions: ", repetitions);
    DebugSerialUtilitiesLn("checkTimeSpeedRead: 11 bytes read");
    DebugSerialSeparation(1);
}

/**
  * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::SpeedRead()
  * @param repetitions: num of repetitions the test is made (time is the average of this repetitions)
  * @see checkTimeGetInfo(). Both are used together to compare speed rate in comunication.
  */
void checkTimeSpeedRead(int repetitions){
    DebugSerialSeparation(1);
    ServoRHA servo_test1();
    servo_test1.init(1, 2, 3, 8, 19200);

    long initTime = 0;
    long timeSpent = 0;

    for int i = 0; i < repetitions; i++{
        initTime = millis();
        servo_test1.speedRead();
        timeSpent += millis()-initTime;
    }
    timeSpent /= repetitions;
    DebugSerialUtilitiesLn2("checkTimeSpeedRead: Time spent with ServoRHA::speedRead: ", timeSpent);
    DebugSerialUtilitiesLn2("checkTimeSpeedRead: with number of repetitions: ", repetitions);
    DebugSerialUtilitiesLn("checkTimeSpeedRead: 2 bytes read");
    DebugSerialSeparation(1);
}

/**
 * @file
 * @brief Implements debugging macros with Serial printig that can be activated or not for each different librari or file.
 *
 * Each set of macros has a define option, if it's been defined all debugging options in the set will be printed. If it's not defined Debug commands en that file will be ignored.
 * Each set has different macros which admit different number of parameters to print.
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: debug.h
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */

#ifndef DEBUG_H
#define DEBUG_H

// #include "joint_handler.h"  // Needs error definitions
#include <Arduino.h>
#include <stdio.h>
//#include <string>
//#include <iostream>

#if defined(__RASPBERRY_PI_3B__)
	#define output(x) { printf(x); } // { std::cout << std::string(x); }  // 
	#define outputln(x) { printf(x); printf("\n"); } // { std::cout << std::string(x) << std::endl; }  // 
#else
	#define output(x) Serial.print(x);
	#define outputln(x) Serial.println(x);
#endif

/*****************************
 * ERROR COD. G15 CUBE SERVO *
 *****************************/

/**
  * @defgroup SERROR_GROUP Error Group
  * Defined to check error returned by servo (check as bit mask)
  * @{
  */
#define SERROR_PING 0X0000
// Return status:
#define SERROR_INPUTVOLTAGE 0X0001
#define SERROR_ANGLELIMIT 0X0002
#define SERROR_OVERHEATING 0X0004
#define SERROR_RANGE 0X0008
#define SERROR_CHECKSUM 0X0010
#define SERROR_OVERLOAD 0X0020
#define SERROR_INSTRUCTION 0X0040
// #define SERROR_ 0X0080
#define SERROR_PACKETLOST 0X0100
#define SERROR_WRONGHEADER 0X0200
#define SERROR_IDMISMATCH 0X0400
#define SERROR_CHECKSUMERROR 0X0800
// #define SERROR_ 0X1000
// #define SERROR_ 0X2000
// #define SERROR_ 0X4000
// #define SERROR_ 0X8000
/**
  * @}
  */

/*********************************
 *       Debugging options       *
 *********************************/

/*
 * To activate debugging in any file uncomment next defines (one for each file)
 */

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_TEST_JOINT_RHA
// #define DEBUG_JOINT_HANDLER  // -> Not stable
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO
 #define DEBUG_UTILITIES
// #define DEBUG_ROBOT_RHA
// #define DEBUG_RHA_TYPES
// #define DEBUG_JOINT_RHA
 #define PRINT_SERVO_ERROR_MSG false

void printServoStatusError(uint16_t error, uint8_t ID);
void printServoStatus(uint16_t pos, uint16_t speed, uint8_t speed_dir, uint16_t load, uint8_t load_dir, uint8_t voltage, uint8_t temperature, uint16_t error);


/******************************************
 *       Debugging macro definition       *
 ******************************************/

/** DEBUG_CYTRON_G15_SERVO implements debug macros for cytron_g15_servo.h and .cpp files */
#ifdef DEBUG_CYTRON_G15_SERVO
    #define DebugSerialG15Ln(a) {  output("[DC]  CYTRON_G15_SERVO::"); outputln(a); }
    #define DebugSerialG15Ln2(a, b) {  output("[DC]  CYTRON_G15_SERVO::"); output(a); outputln(b); }
    #define DebugSerialG15Ln4(a, b, c, d) {  output("[DC]  CYTRON_G15_SERVO::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialG15Ln(a)
    #define DebugSerialG15Ln2(a, b)
    #define DebugSerialG15Ln4(a, b, c, d)
#endif

/** DEBUG_SERVO_RHA implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_SERVO_RHA
    #define DebugSerialSRHALn(a) {  output("[DC]  ServoRHA::"); outputln(a); }
    #define DebugSerialSRHALn2(a, b) {  output("[DC]  ServoRHA::"); output(a); outputln(b); }
    #define DebugSerialSRHALn4(a, b, c, d) {  output("[DC]  ServoRHA::"); output(a); output(b); output(c); outputln(d); }
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error) { printServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error); }
#else
    #define DebugSerialSRHALn(a)
    #define DebugSerialSRHALn2(a, b)
    #define DebugSerialSRHALn4(a, b, c, d)
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error)
#endif

/** DEBUG_JOINT_RHA implements debug macros for joint_rha.h and .cpp files */
#ifdef DEBUG_JOINT_RHA
    #define DebugSerialJRHALn(a) {  output("[DC]  JointRHA::"); outputln(a); }
    #define DebugSerialJRHALn2(a, b) {  output("[DC]  JointRHA::"); output(a); outputln(b); }
    #define DebugSerialJRHALn4(a, b, c, d) {  output("[DC]  JointRHA::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialJRHALn(a)
    #define DebugSerialJRHALn2(a, b)
    #define DebugSerialJRHALn4(a, b, c, d)
#endif

/** DEBUG_JOINT_HANDLER implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_JOINT_HANDLER
    #define DebugSerialJHLn(a) {  output("[DC]  JointHandler::"); outputln(a); }
    #define DebugSerialJHLn2(a, b) {  output("[DC]  JointHandler::"); output(a); outputln(b); }
    #define DebugSerialJHLn4(a, b, c, d) {  output("[DC]  JointHandler::"); output(a); output(b); output(c); outputln(d); }
    // #define DebugSerialJHLn4Error(a, b) { printServoStatusError(a, b); }
#else
    #define DebugSerialJHLn(a)
    #define DebugSerialJHLn2(a, b)
    #define DebugSerialJHLn4(a, b, c, d)
    // #define DebugSerialJHLn4Error(a, b)
#endif


#define DebugSerialJHLn4Error(a, b) { printServoStatusError(a, b); }

/** DEBUG_UTILITIES implements debug macros for utilities.h file */
#ifdef DEBUG_UTILITIES
    #define DebugSerialUtilitiesLn(a) {  output("#[DC]  Utilities::"); outputln(a); }
    #define DebugSerialUtilitiesLn2(a, b) {  output("#[DC]  Utilities::"); output(a); outputln(b); }
    #define DebugSerialUtilities(a) {  output("#[DC]  Utilities::"); output(a); }
    #define DebugSerialUtilitiesLn4(a, b, c, d) {  output("#[DC]  Utilities::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialUtilitiesLn(a)
    #define DebugSerialUtilitiesLn2(a, b)
    #define DebugSerialUtilities(a)
    #define DebugSerialUtilitiesLn4(a, b, c, d)
#endif

/** DEBUG_RHA_TYPES implements debug macros for rha_types.h file */
#ifdef DEBUG_RHA_TYPES
    #define DebugSerialRHATypesLn(a) {  output("[DC]  RHATypes::"); outputln(a); }
    #define DebugSerialRHATypesLn2(a, b) {  output("[DC]  RHATypes::"); output(a); outputln(b); }
    #define DebugSerialRHATypes(a) {  output("[DC]  RHATypes::"); output(a); }
    #define DebugSerialRHATypesLn4(a, b, c, d) {  output("[DC]  RHATypes::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialRHATypesLn(a)
    #define DebugSerialRHATypesLn2(a, b)
    #define DebugSerialRHATypes(a)
    #define DebugSerialRHATypesLn4(a, b, c, d)
#endif

/** DebugSerialSeparation prints a horizontal line to separate different set of debug information */

#define DebugSerialSeparation(a) {outputln("#===============================================================#");}

/** DEBUG_ROBOT_RHA implements debug macros for robot_rha.h and .cpp files */
#ifdef DEBUG_ROBOT_RHA
    #define DebugSerialRRHALn(a) {  output("[DC]  ROBOT_RHA::"); outputln(a); }
    #define DebugSerialRRHALn2(a, b) {  output("[DC]  ROBOT_RHA::"); output(a); outputln(b); }
    #define DebugSerialRRHAn4(a, b, c, d) {  output("[DC]  ROBOT_RHA::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialRRHALn(a)
    #define DebugSerialRRHALn2(a, b)
    #define DebugSerialRRHALn4(a, b, c, d)
#endif

/********************************************************
 *       Debugging macro definition for test files      *
 ********************************************************/

/** DEBUG_TEST_CYTRON_G15_SERVO implements debug macros for test_cytron_g15_servo.cpp file */
#ifdef DEBUG_TEST_CYTRON_G15_SERVO
    #define DebugSerialTG15Ln(a) {  output("[DT]  CYTRON_G15_SERVO::"); outputln(a); }
    #define DebugSerialTG15(a) {  output("[DT]  "); output(a); }
#else
    #define DebugSerialTG15Ln(a)
    #define DebugSerialTG15(a)
#endif

/** DEBUG_TEST_SERVO_RHA_MOCK implements debug macros for test_servo_mock.cpp file */
#ifdef DEBUG_TEST_SERVO_RHA
    #define DebugSerialTSRHALn(a) {  output("[DT]  ServoRHA::"); outputln(a); }
    #define DebugSerialTSRHA(a) {  output("[DT]  ServoRHA::"); output(a); }
#else
    #define DebugSerialTSRHALn(a)
    #define DebugSerialTSRHA(a)
#endif

/** DEBUG_TEST_SERVO_RHA_REAL implements debug macros for test_servo_real.cpp file */
#ifdef DEBUG_TEST_JOINT_RHA
    #define DebugSerialTJRHALn(a) {  output("[DT]  JointRHA::"); outputln(a); }
    #define DebugSerialTJRHALn2(a, b) {  output("[DT]  JointRHA::"); output(a); outputln(b); }
    #define DebugSerialTJRHA(a) {  output("[DT]  JointRHA::"); output(a); }
    #define DebugSerialTJRHALn4(a, b, c, d) {  output("[DT]  JointRHA::"); output(a); output(b); output(c); outputln(d); }
#else
    #define DebugSerialTJRHALn(a)
    #define DebugSerialTJRHALn2(a, b)
    #define DebugSerialTJRHALn4(a, b, c, d)
    #define DebugSerialTJRHA(a)
#endif




#endif  // DEBUG_H

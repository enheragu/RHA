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
// #define DEBUG_UTILITIES
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
    #define DebugSerialG15Ln(a) {  Serial.print(F("[DC]  CYTRON_G15_SERVO::")); Serial.println(a); }
    #define DebugSerialG15Ln2(a, b) {  Serial.print(F("[DC]  CYTRON_G15_SERVO::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialG15Ln4(a, b, c, d) {  Serial.print(F("[DC]  CYTRON_G15_SERVO::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialG15Ln(a)
    #define DebugSerialG15Ln2(a, b)
    #define DebugSerialG15Ln4(a, b, c, d)
#endif

/** DEBUG_SERVO_RHA implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_SERVO_RHA
    #define DebugSerialSRHALn(a) {  Serial.print(F("[DC]  ServoRHA::")); Serial.println(a); }
    #define DebugSerialSRHALn2(a, b) {  Serial.print(F("[DC]  ServoRHA::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialSRHALn4(a, b, c, d) {  Serial.print(F("[DC]  ServoRHA::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error) { printServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error); }
#else
    #define DebugSerialSRHALn(a)
    #define DebugSerialSRHALn2(a, b)
    #define DebugSerialSRHALn4(a, b, c, d)
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error)
#endif

/** DEBUG_JOINT_RHA implements debug macros for joint_rha.h and .cpp files */
#ifdef DEBUG_JOINT_RHA
    #define DebugSerialJRHALn(a) {  Serial.print(F("[DC]  JointRHA::")); Serial.println(a); }
    #define DebugSerialJRHALn2(a, b) {  Serial.print(F("[DC]  JointRHA::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialJRHALn4(a, b, c, d) {  Serial.print(F("[DC]  JointRHA::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialJRHALn(a)
    #define DebugSerialJRHALn2(a, b)
    #define DebugSerialJRHALn4(a, b, c, d)
#endif

/** DEBUG_JOINT_HANDLER implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_JOINT_HANDLER
    #define DebugSerialJHLn(a) {  Serial.print(F("[DC]  JointHandler::")); Serial.println(a); }
    #define DebugSerialJHLn2(a, b) {  Serial.print(F("[DC]  JointHandler::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialJHLn4(a, b, c, d) {  Serial.print(F("[DC]  JointHandler::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
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
    #define DebugSerialUtilitiesLn(a) {  Serial.print("#[DC]  Utilities::")); Serial.println(a); }
    #define DebugSerialUtilitiesLn2(a, b) {  Serial.print("#[DC]  Utilities::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialUtilities(a) {  Serial.print("#[DC]  Utilities::")); Serial.print(a); }
    #define DebugSerialUtilitiesLn4(a, b, c, d) {  Serial.print("#[DC]  Utilities::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialUtilitiesLn(a)
    #define DebugSerialUtilitiesLn2(a, b)
    #define DebugSerialUtilities(a)
    #define DebugSerialUtilitiesLn4(a, b, c, d)
#endif

/** DEBUG_RHA_TYPES implements debug macros for rha_types.h file */
#ifdef DEBUG_RHA_TYPES
    #define DebugSerialRHATypesLn(a) {  Serial.print(F("[DC]  RHATypes::")); Serial.println(a); }
    #define DebugSerialRHATypesLn2(a, b) {  Serial.print(F("[DC]  RHATypes::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialRHATypes(a) {  Serial.print(F("[DC]  RHATypes::")); Serial.print(a); }
    #define DebugSerialRHATypesLn4(a, b, c, d) {  Serial.print(F("[DC]  RHATypes::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialRHATypesLn(a)
    #define DebugSerialRHATypesLn2(a, b)
    #define DebugSerialRHATypes(a)
    #define DebugSerialRHATypesLn4(a, b, c, d)
#endif

/** DebugSerialSeparation prints a horizontal line to separate different set of debug information */
#define DebugSerialSeparation(a) { Serial.println("#===============================================================#"); }

/** DEBUG_ROBOT_RHA implements debug macros for robot_rha.h and .cpp files */
#ifdef DEBUG_ROBOT_RHA
    #define DebugSerialRRHALn(a) {  Serial.print(F("[DC]  ROBOT_RHA::")); Serial.println(a); }
    #define DebugSerialRRHALn2(a, b) {  Serial.print(F("[DC]  ROBOT_RHA::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialRRHAn4(a, b, c, d) {  Serial.print(F("[DC]  ROBOT_RHA::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
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
    #define DebugSerialTG15Ln(a) {  Serial.print(F("[DT]  CYTRON_G15_SERVO::")); Serial.println(a); }
    #define DebugSerialTG15(a) {  Serial.print(F("[DT]  "); Serial.print(a); }
#else
    #define DebugSerialTG15Ln(a)
    #define DebugSerialTG15(a)
#endif

/** DEBUG_TEST_SERVO_RHA_MOCK implements debug macros for test_servo_mock.cpp file */
#ifdef DEBUG_TEST_SERVO_RHA
    #define DebugSerialTSRHALn(a) {  Serial.print(F("[DT]  ServoRHA::")); Serial.println(a); }
    #define DebugSerialTSRHA(a) {  Serial.print(F("[DT]  ServoRHA::")); Serial.print(a); }
#else
    #define DebugSerialTSRHALn(a)
    #define DebugSerialTSRHA(a)
#endif

/** DEBUG_TEST_SERVO_RHA_REAL implements debug macros for test_servo_real.cpp file */
#ifdef DEBUG_TEST_JOINT_RHA
    #define DebugSerialTJRHALn(a) {  Serial.print(F("[DT]  JointRHA::")); Serial.println(a); }
    #define DebugSerialTJRHALn2(a, b) {  Serial.print(F("[DT]  JointRHA::")); Serial.print(a); Serial.println(b); }
    #define DebugSerialTJRHA(a) {  Serial.print(F("[DT]  JointRHA::")); Serial.print(a); }
    #define DebugSerialTJRHALn4(a, b, c, d) {  Serial.print(F("[DT]  JointRHA::")); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialTJRHALn(a)
    #define DebugSerialTJRHALn2(a, b)
    #define DebugSerialTJRHALn4(a, b, c, d)
    #define DebugSerialTJRHA(a)
#endif




#endif  // DEBUG_H

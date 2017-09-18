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
 * @Last modified time: 16-Sep-2017
 */

#ifndef DEBUG_H
#define DEBUG_H

/*********************************
 *       Debugging options       *
 *********************************/

/*
 * To activate debugging in any file uncomment next defines (one for each file)
 */

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA_MOCK
// #define DEBUG_TEST_SERVO_RHA_REAL
// #define DEBUG_JOINT_HANDLER
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO
#define DEBUG_UTILITIES


/******************************************
 *       Debugging macro definition       *
 ******************************************/

/** DEBUG_CYTRON_G15_SERVO implements debug macros for cytron_g15_servo.h and .cpp files */
#ifdef DEBUG_CYTRON_G15_SERVO
    #define DebugSerialG15Ln(a) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.println(a); }
    #define DebugSerialG15Ln2(a, b) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialG15Ln4(a, b, c, d) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialG15Ln(a)
    #define DebugSerialG15Ln2(a, b)
    #define DebugSerialG15Ln4(a, b, c, d)
#endif

/** DEBUG_SERVO_RHA implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_SERVO_RHA
    #define DebugSerialSRHALn(a) {  Serial.print("[DC]  ServoRHA::"); Serial.println(a); }
    #define DebugSerialSRHALn2(a, b) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialSRHALn4(a, b, c, d) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialSRHALn(a)
    #define DebugSerialSRHALn2(a, b)
    #define DebugSerialSRHALn4(a, b, c, d)
#endif

/** DEBUG_JOINT_HANDLER implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_JOINT_HANDLER
    #define DebugSerialJHLn(a) {  Serial.print("[DC]  ServoRHA::"); Serial.println(a); }
    #define DebugSerialJHLn2(a, b) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialJHLn4(a, b, c, d) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialJHLn(a)
    #define DebugSerialJHLn2(a, b)
    #define DebugSerialJHLn4(a, b, c, d)
#endif

/** DEBUG_UTILITIES implements debug macros for utilities.h file */
#ifdef DEBUG_UTILITIES
    #define DebugSerialUtilitiesLn(a) {  Serial.print("[DC]  Utilities::"); Serial.println(a); }
    #define DebugSerialUtilitiesLn2(a, b) {  Serial.print("[DC]  Utilities::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialUtilities(a) {  Serial.print("[DC]  Utilities::"); Serial.print(a); }
    #define DebugSerialUtilitiesLn4(a, b, c, d) {  Serial.print("[DC]  Utilities::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialUtilitiesLn(a)
    #define DebugSerialUtilitiesLn2(a, b)
    #define DebugSerialUtilities(a)
    #define DebugSerialUtilitiesLn4(a, b, c, d)
#endif

/** DebugSerialSeparation prints a horizontal line to separate different set of debug information */
#define DebugSerialSeparation(a) {Serial.println("=========================================================");}


/********************************************************
 *       Debugging macro definition for test files      *
 ********************************************************/

/** DEBUG_TEST_CYTRON_G15_SERVO implements debug macros for test_cytron_g15_servo.cpp file */
#ifdef DEBUG_TEST_CYTRON_G15_SERVO
    #define DebugSerialTG15Ln(a) {  Serial.print("[DT]  CYTRON_G15_SERVO::"); Serial.println(a); }
    #define DebugSerialTG15(a) {  Serial.print("[DT]  "); Serial.print(a); }
#else
    #define DebugSerialTG15Ln(a)
    #define DebugSerialTG15(a)
#endif

/** DEBUG_TEST_SERVO_RHA_MOCK implements debug macros for test_servo_mock.cpp file */
#ifdef DEBUG_TEST_SERVO_RHA_MOCK
    #define DebugSerialTSRHAMockLn(a) {  Serial.print("[DT]  (mock)ServoRHA::"); Serial.println(a); }
    #define DebugSerialTSRHAMock(a) {  Serial.print("[DT]  (mock)ServoRHA::"); Serial.print(a); }
#else
    #define DebugSerialTSRHAMockLn(a)
    #define DebugSerialTSRHAMock(a)
#endif

/** DEBUG_TEST_SERVO_RHA_REAL implements debug macros for test_servo_real.cpp file */
#ifdef DEBUG_TEST_SERVO_RHA_REAL
    #define DebugSerialTSRHARealLn(a) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.println(a); }
    #define DebugSerialTSRHARealLn2(a, b) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialTSRHAReal(a) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); }
    #define DebugSerialTSRHARealLn4(a, b, c, d) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialTSRHARealLn(a)
    #define DebugSerialTSRHARealLn2(a, b)
    #define DebugSerialTSRHARealLn4(a, b, c, d)
    #define DebugSerialTSRHAReal(a)
#endif

#endif  // DEBUG_H

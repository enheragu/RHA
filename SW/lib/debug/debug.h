// #include "debug.h"

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
#define DEBUG_TEST_SERVO_RHA_REAL
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO


/******************************************
 *       Debugging macro definition       *
 ******************************************/

#ifdef DEBUG_CYTRON_G15_SERVO
    #define DebugSerialG15Ln(a) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.println(a); }
    #define DebugSerialG15Ln2(a, b) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialG15Ln4(a, b, c, d) {  Serial.print("[DC]  CYTRON_G15_SERVO::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialG15Ln(a)
    #define DebugSerialG15Ln2(a, b)
    #define DebugSerialG15Ln4(a, b, c, d)
#endif

#ifdef DEBUG_SERVO_RHA
    #define DebugSerialSRHALn(a) {  Serial.print("[DC]  ServoRHA::"); Serial.println(a); }
    #define DebugSerialSRHALn2(a, b) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialSRHALn4(a, b, c, d) {  Serial.print("[DC]  ServoRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialSRHALn(a)
    #define DebugSerialSRHALn2(a, b)
    #define DebugSerialSRHALn4(a, b, c, d)
#endif

/********************************************************
 *       Debugging macro definition for test files      *
 ********************************************************/

#ifdef DEBUG_TEST_CYTRON_G15_SERVO
    #define DebugSerialTG15Ln(a) {  Serial.print("[DT]  CYTRON_G15_SERVO::"); Serial.println(a); }
    #define DebugSerialTG15(a) {  Serial.print("[DT]  "); Serial.print(a); }
#else
    #define DebugSerialTG15Ln(a)
    #define DebugSerialTG15(a)
#endif

#ifdef DEBUG_TEST_SERVO_RHA_MOCK
    #define DebugSerialTSRHAMockLn(a) {  Serial.print("[DT]  (mock)ServoRHA::"); Serial.println(a); }
    #define DebugSerialTSRHAMock(a) {  Serial.print("[DT]  (mock)ServoRHA::"); Serial.print(a); }
#else
    #define DebugSerialTSRHAMockLn(a)
    #define DebugSerialTSRHAMock(a)
#endif

#ifdef DEBUG_TEST_SERVO_RHA_REAL
    #define DebugSerialTSRHARealLn(a) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.println(a); }
    #define DebugSerialTSRHARealLn2(a, b) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialTSRHAReal(a) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); }
    #define DebugSerialTSRHARealLn4(a, b, c, d) {  Serial.print("[DT]  (real)ServoRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialTSRHARealLn(a)
    #define DebugSerialTSRHARealLn4(a, b)
    #define DebugSerialG15Ln4(a, b, c, d)
    #define DebugSerialTSRHAReal(a)
#endif


#endif  // DEBUG_H

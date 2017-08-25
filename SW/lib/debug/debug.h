//#include "debug.h"

#ifndef DEBUG_H
#define DEBUG_H

/*********************************
 *       Debugging options       *
 *********************************/

/*
 * To activate debugging in any file uncomment next defines (one for each file)
 */

//#define DEBUG_SERVO_RHA
//#define DEBUG_TEST_SERVO_RHA_MOCK
//#define DEBUG_TEST_SERVO_RHA_REAL
//#define DEBUG_CYTRON_G15SHIELD
//#define DEBUG_TEST_CYTRON_G15SHIELD


/******************************************
 *       Debugging macro definition       *
 ******************************************/

#ifdef DEBUG_CYTRON_G15SHIELD
  #define DebugSerialG15Ln(a) {Serial.print("[-]  "); Serial.println(a);}
  #define DebugSerialG15Ln2(a,b) {Serial.print("[-]  ");Serial.print(a); Serial.println(b);}
  #define DebugSerialG15Ln4(a,b,c,d) {Serial.print("[-]  ");Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d);}
#else
  #define DebugSerialG15Ln(a)
  #define DebugSerialG15Ln2(a,b)
  #define DebugSerialG15Ln4(a,b,c,d)
#endif

#ifdef DEBUG_SERVO_RHA
  #define DebugSerialSRHALn(a) {Serial.print("[-]  "); Serial.println(a);}
  #define DebugSerialSRHALn2(a,b) {Serial.print("[-]  ");Serial.print(a); Serial.println(b);}
  #define DebugSerialSRHALn4(a,b,c,d) {Serial.print("[-]  ");Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d);}
#else
  #define DebugSerialSRHALn(a)
  #define DebugSerialSRHALn2(a,b)
  #define DebugSerialSRHALn4(a,b,c,d)
#endif

/********************************************************
 *       Debugging macro definition for test files      *
 ********************************************************/

#ifdef DEBUG_TEST_CYTRON_G15SHIELD
 #define DebugSerialTG15Ln(a) {Serial.println(a);}
 #define DebugSerialTG15(a) {Serial.print(a);}
#else
 #define DebugSerialTG15Ln(a)
 #define DebugSerialTG15(a)
#endif

#ifdef DEBUG_TEST_SERVO_RHA_MOCK
 #define DebugSerialTSRHAMockLn(a) {Serial.println(a);}
 #define DebugSerialTSRHAMock(a) {Serial.print(a);}
#else
 #define DebugSerialTSRHAMockLn(a)
 #define DebugSerialTSRHAMockz(a)
#endif

#ifdef DEBUG_TEST_SERVO_RHA_REAL
  #define DebugSerialTSRHARealLn(a) {Serial.println(a);}
  #define DebugSerialTSRHAReal(a) {Serial.print(a);}
#else
  #define DebugSerialTSRHARealLn(a)
  #define DebugSerialTSRHAReal(a)
#endif


#endif //DEBUG_H

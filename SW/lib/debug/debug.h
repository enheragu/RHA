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
 * @Last modified by:   enheragu
 * @Last modified time: 21_Sep_2017
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "joint_handler.h"  // Needs error definition
/*********************************
 *       Debugging options       *
 *********************************/

/*
 * To activate debugging in any file uncomment next defines (one for each file)
 */

 #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA_MOCK
// #define DEBUG_TEST_SERVO_RHA_REAL
// #define DEBUG_JOINT_HANDLER
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO
// #define DEBUG_UTILITIES
// #define DEBUG_RHA_TYPES
#define DEBUG_JOINT_RHA

void printServoStatusError (uint16_t error, uint8_t ID);

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
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error) {
      Serial.println(" ");
      Serial.print("[DC]  ServoRHA::Printing servo stats: ");
      Serial.print("              - Position: "); Serial.println(pos);
      Serial.print("              - Speed: "); Serial.println(speed);
      Serial.print("              - Speed dir (CW = 1; CCW = 0): "); Serial.println(speed_dir);
      Serial.print("              - Load: "); Serial.println(load);
      Serial.print("              - Load dir (CW = 1; CCW = 0): "); Serial.println(load_dir);
      Serial.print("              - Voltage: "); Serial.println(voltage);
      Serial.print("              - Temperature: "); Serial.println(temperature);
      Serial.print("              - Error in comunication: "); Serial.println(error);
      Serial.println(" ");
    }
#else
    #define DebugSerialSRHALn(a)
    #define DebugSerialSRHALn2(a, b)
    #define DebugSerialSRHALn4(a, b, c, d)
    #define DebuSerialRHALnPrintServoStatus(pos, speed, speed_dir, load, load_dir, voltage, temperature, error)
#endif

/** DEBUG_JOINT_RHA implements debug macros for joint_rha.h and .cpp files */
#ifdef DEBUG_JOINT_RHA
    #define DebugSerialJRHALn(a) {  Serial.print("[DC]  JointRHA::"); Serial.println(a); }
    #define DebugSerialJRHALn2(a, b) {  Serial.print("[DC]  JointRHA::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialJRHALn4(a, b, c, d) {  Serial.print("[DC]  JointRHA::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialJRHALn(a)
    #define DebugSerialJRHALn2(a, b)
    #define DebugSerialJRHALn4(a, b, c, d)
#endif

/** DEBUG_JOINT_HANDLER implements debug macros for servo_rha.h and .cpp files */
#ifdef DEBUG_JOINT_HANDLER
    #define DebugSerialJHLn(a) {  Serial.print("[DC]  JointHandler::"); Serial.println(a); }
    #define DebugSerialJHLn2(a, b) {  Serial.print("[DC]  JointHandler::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialJHLn4(a, b, c, d) {  Serial.print("[DC]  JointHandler::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
    #define DebugSerialJHLn4Error(a, b) { printServoStatusError(a, b); }
#else
    #define DebugSerialJHLn(a)
    #define DebugSerialJHLn2(a, b)
    #define DebugSerialJHLn4(a, b, c, d)
    #define DebugSerialJHLn4Error(a, b)
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

/** DEBUG_RHA_TYPES implements debug macros for rha_types.h file */
#ifdef DEBUG_RHA_TYPES
    #define DebugSerialRHATypesLn(a) {  Serial.print("[DC]  RHATypes::"); Serial.println(a); }
    #define DebugSerialRHATypesLn2(a, b) {  Serial.print("[DC]  RHATypes::"); Serial.print(a); Serial.println(b); }
    #define DebugSerialRHATypes(a) {  Serial.print("[DC]  RHATypes::"); Serial.print(a); }
    #define DebugSerialRHATypesLn4(a, b, c, d) {  Serial.print("[DC]  RHATypes::"); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d); }
#else
    #define DebugSerialRHATypesLn(a)
    #define DebugSerialRHATypesLn2(a, b)
    #define DebugSerialRHATypes(a)
    #define DebugSerialRHATypesLn4(a, b, c, d)
#endif

/** DebugSerialSeparation prints a horizontal line to separate different set of debug information */
#define DebugSerialSeparation(a) {Serial.println("#=========================================================#");}


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


/**
  * @brief Analyses error and prints error msgs
  */
void printServoStatusError (uint16_t error, uint8_t ID) {
    // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
    if (error != 0){
        Serial.print("Error in comunication detected in servo: "); Serial.println(ID);
    } else return;
    if (error & SERROR_PING)  {Serial.println("Ping error in servo");}
    if (error & SERROR_INPUTVOLTAGE)  {Serial.println("Input voltage error in servo");}          // bit 0
    if (error & SERROR_ANGLELIMIT)  {Serial.println("Angle limit error in servo");}           // bit 1
    if (error & SERROR_OVERHEATING)  {Serial.println("Overheating error in servo");}          // bit 2
    if (error & SERROR_RANGE)  {Serial.println("Range error in servo");}             // bit 3
    if (error & SERROR_CHECKSUM)  {Serial.println("Checksum error in servo");}             // bit 4
    if (error & SERROR_OVERLOAD)  {Serial.println("Overload error in servo");}             // bit 5
    if (error & SERROR_INSTRUCTION)  {Serial.println("Instruction error in servo");}            // bit 7
    if (error & SERROR_PACKETLOST)  {Serial.println("Packet lost or receive time out in servo");}    // bit 8
    if (error & SERROR_WRONGHEADER)  {Serial.println("Wrong header in servo");}              // bit 9
    if (error & SERROR_IDMISMATCH)  {Serial.println("ID mismatch in servo");}                  // bit 10
    if (error & SERROR_CHECKSUMERROR)  {Serial.println("Checksum error in servo");}               // bit 13
}


}

#endif  // DEBUG_H

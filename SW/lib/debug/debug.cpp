/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   21-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 24-Sep-2017
 */

#include "debug.h"

 /**
   * @brief Analyses error and prints error msgs
   */
 void printServoStatusError (uint16_t error, uint8_t ID) {
     // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
     if (error != 0){
         Serial.print("[!]   Error in comunication detected in servo: "); Serial.println(ID);
     } else return;
     if (error & SERROR_PING)  {Serial.println("[!]   Ping error in servo");}
     if (error & SERROR_INPUTVOLTAGE)  {Serial.println("[!]   Input voltage error in servo");}          // bit 0
     if (error & SERROR_ANGLELIMIT)  {Serial.println("[!]   Angle limit error in servo");}           // bit 1
     if (error & SERROR_OVERHEATING)  {Serial.println("[!]   Overheating error in servo");}          // bit 2
     if (error & SERROR_RANGE)  {Serial.println("[!]   Range error in servo");}             // bit 3
     if (error & SERROR_CHECKSUM)  {Serial.println("[!]   Checksum error in servo");}             // bit 4
     if (error & SERROR_OVERLOAD)  {Serial.println("[!]   Overload error in servo");}             // bit 5
     if (error & SERROR_INSTRUCTION)  {Serial.println("[!]   Instruction error in servo");}            // bit 7
     if (error & SERROR_PACKETLOST)  {Serial.println("[!]   Packet lost or receive time out in servo");}    // bit 8
     if (error & SERROR_WRONGHEADER)  {Serial.println("[!]   Wrong header in servo");}              // bit 9
     if (error & SERROR_IDMISMATCH && ID != 0xFE)  {Serial.println("[!]   ID mismatch in servo");}                  // bit 10 // If id is for all servo don't print ID mismatch
     if (error & SERROR_CHECKSUMERROR)  {Serial.println("[!]   Checksum error in servo");}               // bit 13
 }

 void printServoStatus(uint16_t pos, uint16_t speed, uint8_t speed_dir, uint16_t load, uint8_t load_dir, uint8_t voltage, uint8_t temperature, uint16_t error) {
     Serial.println(" ");
     Serial.println("[DC]  ServoRHA::Printing servo stats: ");
     Serial.println("              - Position: "); Serial.println(pos);
     Serial.println("              - Speed: "); Serial.println(speed);
     Serial.println("              - Speed dir (CW = 1; CCW = 0): "); Serial.println(speed_dir);
     Serial.println("              - Load: "); Serial.println(load);
     Serial.println("              - Load dir (CW = 1; CCW = 0): "); Serial.println(load_dir);
     Serial.println("              - Voltage: "); Serial.println(voltage);
     Serial.println("              - Temperature: "); Serial.println(temperature);
     Serial.println("              - Error in comunication: "); Serial.println(error);
     Serial.println(" ");
 }

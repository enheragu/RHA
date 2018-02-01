/**
 * @Author: Enrique Heredia Aguado <quique>
 * @Date:   21-Sep-2017
 * @Project: RHA
 * @Last modified by:   quique
 * @Last modified time: 28-Sep-2017
 */

#include "debug.h"

 /**
   * @brief Analyses error and prints error msgs
   */
void printServoStatusError(uint16_t error, uint8_t ID) {
     // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
     // NOTE: turn to false to print servo status error msg, to true to avoid them
     if (!PRINT_SERVO_ERROR_MSG) return;
     if (error != 0) {
         Serial.print("#[!]   Error in comunication detected in servo: "); Serial.println(ID);
     } else
        return;
     if (error & SERROR_PING)  { Serial.println(F("#[!]   Ping error in servo")); }
     if (error & SERROR_INPUTVOLTAGE)  { Serial.println(F("#[!]   Input voltage error in servo")); }          // bit 0
     if (error & SERROR_ANGLELIMIT)  { Serial.println(F("#[!]   Angle limit error in servo")); }           // bit 1
     if (error & SERROR_OVERHEATING)  { Serial.println(F("#[!]   Overheating error in servo")); }          // bit 2
     if (error & SERROR_RANGE)  { Serial.println(F("#[!]   Range error in servo")); }             // bit 3
     if (error & SERROR_CHECKSUM)  { Serial.println(F("#[!]   Checksum error in servo")); }             // bit 4
     if (error & SERROR_OVERLOAD)  { Serial.println(F("#[!]   Overload error in servo")); }             // bit 5
     if (error & SERROR_INSTRUCTION)  { Serial.println(F("#[!]   Instruction error in servo")); }            // bit 7
     if (error & SERROR_PACKETLOST)  { Serial.println(F("#[!]   Packet lost or receive time out in servo")); }    // bit 8
     if (error & SERROR_WRONGHEADER)  { Serial.println(F("#[!]   Wrong header in servo")); }              // bit 9
     if (error & SERROR_IDMISMATCH && ID != 0xFE)  { Serial.println(F("#[!]   ID mismatch in servo")); }                  // bit 10 // If id is for all servo don't print ID mismatch
     if (error & SERROR_CHECKSUMERROR)  { Serial.println(F("#[!]   Checksum error in servo")); }               // bit 13
}

void printServoStatus(uint16_t pos, uint16_t speed, uint8_t speed_dir, uint16_t load, uint8_t load_dir, uint8_t voltage, uint8_t temperature, uint16_t error) {
     Serial.println();
     Serial.println(F("[DC]  ServoRHA::Printing servo stats: "));
     Serial.println(F("              - Position: ")); Serial.println(pos);
     Serial.println(F("              - Speed: ")); Serial.println(speed);
     Serial.println(F("              - Speed dir (CW = 1; CCW = 0): ")); Serial.println(speed_dir);
     Serial.println(F("              - Load: ")); Serial.println(load);
     Serial.println(F("              - Load dir (CW = 1; CCW = 0): ")); Serial.println(load_dir);
     Serial.println(F("              - Voltage: ")); Serial.println(voltage);
     Serial.println(F("              - Temperature: ")); Serial.println(temperature);
     Serial.println(F("              - Error in comunication: ")); Serial.println(error);
     Serial.println();
}

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
 void printServoStatusError (uint16_t error, uint8_t ID) {
     // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
     if (error != 0){
         output("#[!]   Error in comunication detected in servo: "); outputln(ID);
     } else return;
     if (error & SERROR_PING)  {outputln("#[!]   Ping error in servo");}
     if (error & SERROR_INPUTVOLTAGE)  {outputln("#[!]   Input voltage error in servo");}          // bit 0
     if (error & SERROR_ANGLELIMIT)  {outputln("#[!]   Angle limit error in servo");}           // bit 1
     if (error & SERROR_OVERHEATING)  {outputln("#[!]   Overheating error in servo");}          // bit 2
     if (error & SERROR_RANGE)  {outputln("#[!]   Range error in servo");}             // bit 3
     if (error & SERROR_CHECKSUM)  {outputln("#[!]   Checksum error in servo");}             // bit 4
     if (error & SERROR_OVERLOAD)  {outputln("#[!]   Overload error in servo");}             // bit 5
     if (error & SERROR_INSTRUCTION)  {outputln("#[!]   Instruction error in servo");}            // bit 7
     if (error & SERROR_PACKETLOST)  {outputln("#[!]   Packet lost or receive time out in servo");}    // bit 8
     if (error & SERROR_WRONGHEADER)  {outputln("#[!]   Wrong header in servo");}              // bit 9
     if (error & SERROR_IDMISMATCH && ID != 0xFE)  {outputln("#[!]   ID mismatch in servo");}                  // bit 10 // If id is for all servo don't print ID mismatch
     if (error & SERROR_CHECKSUMERROR)  {outputln("#[!]   Checksum error in servo");}               // bit 13
 }

 void printServoStatus(uint16_t pos, uint16_t speed, uint8_t speed_dir, uint16_t load, uint8_t load_dir, uint8_t voltage, uint8_t temperature, uint16_t error) {
     outputln(" ");
     outputln("[DC]  ServoRHA::Printing servo stats: ");
     outputln("              - Position: "); outputln(pos);
     outputln("              - Speed: "); outputln(speed);
     outputln("              - Speed dir (CW = 1; CCW = 0): "); outputln(speed_dir);
     outputln("              - Load: "); outputln(load);
     outputln("              - Load dir (CW = 1; CCW = 0): "); outputln(load_dir);
     outputln("              - Voltage: "); outputln(voltage);
     outputln("              - Temperature: "); outputln(temperature);
     outputln("              - Error in comunication: "); outputln(error);
     outputln(" ");
 }

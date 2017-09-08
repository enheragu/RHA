//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include "servo_rha.h"
#include <Arduino.h>
//#include "joint_rha.h"

// #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
// #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

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

 byte data[10], IDcurrent = 254, IDset = 0, IDBroadcast = 0xFE;



 //returns string message of servo errors
 int printServoStatusError (uint16_t error)
 {

     if (error & SERROR_PING)  Serial.println("[!]   Ping error in servo: ");
     if (error & SERROR_INPUTVOLTAGE)  Serial.println("[!]   Input voltage error in servo: ")    ;          // bit 0
     if (error & SERROR_ANGLELIMIT)  Serial.println("[!]   Angle limit error in servo: ")      ;           // bit 1
     if (error & SERROR_OVERHEATING)  Serial.println("[!]   Overheating error in servo: ")     ;          // bit 2
     if (error & SERROR_RANGE)  Serial.println("[!]   Range error in servo: ")               ;             // bit 3
     if (error & SERROR_CHECKSUM)  Serial.println("[!]   Checksum error in servo: ")          ;             // bit 4
     if (error & SERROR_OVERLOAD)  Serial.println("[!]   Overload error in servo: ")          ;             // bit 5
     if (error & SERROR_INSTRUCTION)  Serial.println("[!]   Instruction error in servo: ");            // bit 7
     if (error & SERROR_PACKETLOST)  Serial.println("[!]   Packet lost or receive time out in servo: ");    // bit 8
     if (error & SERROR_WRONGHEADER)  Serial.println("[!]   Wrong header in servo: ")      ;              // bit 9
     if (error & SERROR_IDMISMATCH)  Serial.println("[!]   ID mismatch in servo: ")       ;                  // bit 10
     if (error & SERROR_CHECKSUMERROR)  Serial.println("[!]   Checksum error in servo: ")    ;               // bit 13
     else {
         Serial.print("Default case. Error: ");
         Serial.println(error);
     }
 }

void setup() {
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  // g15.begin(19200);
  //servo_test1.init(1, 2, 3, 8, 9200);
  ServoRHA servo_broadcast;
  servo_broadcast.init(0xFE,2,3,8,19200);         //Broadcast initialize
  servo_broadcast.ping(data);
  IDcurrent = data[0];
  Serial.print("ID now is: "); Serial.println(IDcurrent);
  ServoRHA servo1;
  servo1.init(1,2,3,8,19200);

  delay(25);
  servo1.setAlarmLED(0x7F);
  delay(25);
  Serial.println("Exiting wheel mode");
  word error = servo1.exitWheelMode();
  printServoStatusError(error);
  delay(25);
  Serial.println("Set wheel mode");
  error = servo1.setWheelMode();
  printServoStatusError(error);
  delay(25);
  Serial.println("Set speed");
  error = servo1.setWheelSpeed(1000, CW);
  printServoStatusError(error);
}

void loop(){

}

/*
void loop() {
  Serial.println("estoy en el loop");

  speedSet = 700;
  torqueSet = 1023;
  encoderSmallRotation = 0;
  encoderFullRotation = 1;
  encoderTotal = 0;
  encoderCurrent = 0;

  long initTime = 0;
  long currentTime = 0;

  servo_test1.setTorqueLimit( torqueSet);
  servo_test1.setWheelMode();
  servo_test1.setWheelSpeed( 0, CCW);

  servo_test1.getPos(data);             //get the current position from servo
  pos = data[0];
  pos = pos | ((data[1]) << 8);
  encoderTemp = pos;
  Serial.println("Configuration done.");
  delay(2000);
  Serial.println("Start moving");

  uint16_t error;
  servo_test1.setTorqueOnOff(ON, iREG_WRITE);
  error = servo_test1.setWheelSpeed(speedSet, CCW);
  printServoStatusError(error);

  encoderFlag = 1;
  initTime = millis();
  while (1) {
    Serial.println("Init while loop");
    servo_test1.getPos(data); //get the current position from servo
    pos = data[0];
    pos = pos | ((data[1]) << 8);
    encoderCurrent = pos;
    Serial.print("Current pose: ");   Serial.println(encoderCurrent);

    if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
      encoderTotal++;
      currentTime = millis();
      long speedNow = encoderTotal*1000/(initTime - currentTime);
      Serial.print("Torque set es: "); Serial.println(torqueSet);
      Serial.print("  -  Velocidad calculada es: "); Serial.println(speedNow);
      long speedGet = servo_test1.speedRead();
      Serial.print("  -  Velocidad obtenida es: "); Serial.println(speedGet);
      encoderFlag = 1;
      if (encoderTotal == encoderFullRotation ) {
        servo_test1.setTorqueOnOff(ON, iREG_WRITE);
        servo_test1.setWheelSpeed( 0, CCW);
        break;
      }
    }
    if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
  }

  // g15.setWheelMode();
  // g15.setWheelSpeed(500, CW, iWRITE_DATA);



}*/

//#endif

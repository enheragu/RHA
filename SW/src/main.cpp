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
   switch (error)
   {
     case SERROR_PING: return  Serial.print("[!]   Ping error in servo: ");
     case SERROR_INPUTVOLTAGE: return  Serial.print("[!]   Input voltage error in servo: ")    ;          // bit 0
     case SERROR_ANGLELIMIT: return  Serial.print("[!]   Angle limit error in servo: ")      ;           // bit 1
     case SERROR_OVERHEATING: return  Serial.print("[!]   Overheating error in servo: ")     ;          // bit 2
     case SERROR_RANGE: return  Serial.print("[!]   Range error in servo: ")               ;             // bit 3
     case SERROR_CHECKSUM: return  Serial.print("[!]   Checksum error in servo: ")          ;             // bit 4
     case SERROR_OVERLOAD: return  Serial.print("[!]   Overload error in servo: ")          ;             // bit 5
     case SERROR_INSTRUCTION: return  Serial.print("[!]   Instruction error in servo: ");            // bit 7
     case SERROR_PACKETLOST: return  Serial.print("[!]   Packet lost or receive time out in servo: ");    // bit 8
     case SERROR_WRONGHEADER: return  Serial.print("[!]   Wrong header in servo: ")      ;              // bit 9
     case SERROR_IDMISMATCH: return  Serial.print("[!]   ID mismatch in servo: ")       ;                  // bit 10
     case SERROR_CHECKSUMERROR: return  Serial.print("[!]   Checksum error in servo: ")    ;               // bit 13
     default: Serial.println("default case");
   }
 }

void setup() {
  Serial.begin(9600);
  Serial.println("estoy en el setup");
  // g15.begin(19200);
  //servo_test1.init(1, 2, 3, 8, 9200);
  servo_test1.init();
  servo_test1.exitWheelMode();
}


void loop() {
  Serial.println("estoy en el loop");

  speedSet = 500;
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



}

//#endif

/**
 * @file
 * @brief Implements a set of utilities to measure, experimentally, some interesting parameters.
 *
 * Measures real speed of servo, time spent with packet handling, etc
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: utilities.h
 * @Last modified by:   enheragu
 * @Last modified time: 12-Sep-2017
 */

#include <Arduino.h>
#include "debug.h"
#include "servo_rha.h"
#include "cytron_g15_servo.h"

/**
  * @brief Analyses error and prints error msgs
  */
void printServoStatusError (uint16_t error){
    // NOTE: MACROS NEED {} AS THEY AR SUBSTITUTED BY SOME CODE LINES, NOT JUST ONE!!
    if (error & SERROR_PING)  {DebugSerialUtilitiesLn("Ping error in servo");}
    if (error & SERROR_INPUTVOLTAGE)  {DebugSerialUtilitiesLn("Input voltage error in servo");}          // bit 0
    if (error & SERROR_ANGLELIMIT)  {DebugSerialUtilitiesLn("Angle limit error in servo");}           // bit 1
    if (error & SERROR_OVERHEATING)  {DebugSerialUtilitiesLn("Overheating error in servo");}          // bit 2
    if (error & SERROR_RANGE)  {DebugSerialUtilitiesLn("Range error in servo");}             // bit 3
    if (error & SERROR_CHECKSUM)  {DebugSerialUtilitiesLn("Checksum error in servo");}             // bit 4
    if (error & SERROR_OVERLOAD)  {DebugSerialUtilitiesLn("Overload error in servo");}             // bit 5
    if (error & SERROR_INSTRUCTION)  {DebugSerialUtilitiesLn("Instruction error in servo");}            // bit 7
    if (error & SERROR_PACKETLOST)  {DebugSerialUtilitiesLn("Packet lost or receive time out in servo");}    // bit 8
    if (error & SERROR_WRONGHEADER)  {DebugSerialUtilitiesLn("Wrong header in servo");}              // bit 9
    if (error & SERROR_IDMISMATCH)  {DebugSerialUtilitiesLn("ID mismatch in servo");}                  // bit 10
    if (error & SERROR_CHECKSUMERROR)  {DebugSerialUtilitiesLn("Checksum error in servo");}               // bit 13
}

namespace MeasureUtilities{

    #define SPEED 800  // torque send to check speed
    #define SPEED_TARGET 80  // speed target in rpm
    #define KP_REGULATOR 150  // kp for extractRegulatorData function
    #define LOOP_FREQUENCY 100  // in ms
    #define SAMPLE_REGULATOR 150 // samples taken every LOOP_FREQUENCY
    #define BAUD_RATE_G15 57600

    #define CHAUVENET_REPETITIONS 50  // too many repetitions cause memory overfload

    /**
     * @brief checkSpeed implements an encoder mode to measure real speed in RPM and check agains the measure returned by servo and torque value sent. It can test one servo. Autodetects ID of the one connected
     */
    void checkSpeed(){
        DebugSerialUtilitiesLn("checkSpeed: begin of function");
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        printServoStatusError(error);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkSpeed")
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        uint32_t encoderTemp = 0,
             encoderCurrent = 0,
             encoderFullRotation = 100,
             encoderTotal = 0;

        uint16_t speed_set = SPEED,
            torque_set = 1023;

        uint8_t encoderFlag = 0;

        uint16_t pos = 0;

        uint32_t initTime = 0;
        uint32_t currentTime = 0;

        error = servo_test1.setTorqueLimit( torque_set);
        delay(25);
        printServoStatusError(error);
        servo_test1.setWheelMode();
        delay(25);
        servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);
        delay(25);

        servo_test1.getPos(data);             //get the current position from servo
        pos = data[0];
        pos = pos | ((data[1]) << 8);
        encoderTemp = pos;
        encoderFlag = 1;
        initTime = millis();

        DebugSerialUtilitiesLn("Configuration done.");
        delay(2000);
        DebugSerialUtilitiesLn("Start moving");

        error = servo_test1.setTorqueOnOff(ON, iREG_WRITE);
        delay(25);
        printServoStatusError(error);
        error = servo_test1.setWheelSpeed(speed_set, CW, iWRITE_DATA);
        delay(25);
        printServoStatusError(error);

        while (1) {
            //DebugSerialUtilitiesLn("Init while loop");
            servo_test1.getPos(data); //get the current position from servo
            pos = data[0];
            pos = pos | ((data[1]) << 8);
            encoderCurrent = pos;
            //DebugSerialUtilitiesLn2("Current pose: ", encoderCurrent);

            if (encoderCurrent < (encoderTemp + 5)  && encoderCurrent > (encoderTemp - 5) && encoderFlag == 0) {
                encoderTotal++;
                currentTime = millis();
                float time_whole = ((float)(currentTime - initTime) / 1000);
                float speed_now = ((float)encoderTotal / (float)time_whole)*60;
                float speed_read = servo_test1.speedRead();
                DebugSerialSeparation(1);
                DebugSerialUtilitiesLn2("  -  Torque set is: ", speed_set);
                DebugSerialUtilitiesLn2("  -  Speed calculated is (in rpm): ", speed_now);
                DebugSerialUtilitiesLn2("  -  Speed calculated is (in rad/s): ", speed_now/(2*PI/60));
                DebugSerialUtilitiesLn2("  -  Speed read is (in rpm): ", speed_read*(2*PI/60));
                DebugSerialUtilitiesLn2("  -  Speed read is (in rad/s): ", speed_read);
                DebugSerialUtilitiesLn2("  -  Encoder whole is: ", encoderTotal);
                DebugSerialUtilitiesLn2("  -  Time spent is (in s): ", time_whole);
                DebugSerialSeparation(1);
                encoderFlag = 1;
                if (encoderTotal == encoderFullRotation ) {
                    servo_test1.setTorqueOnOff(ON, iREG_WRITE);
                    servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);
                    break;
                }
            }
            if (encoderCurrent > (encoderTemp + 5) || encoderCurrent < (encoderTemp - 5)) encoderFlag = 0;
        }
    }


    #define KN 1.54 // Chauvenet coeficient for n = 4

    /**
     * @brief Calculates the average applying chauvenets criterion
     * @method averageChauvenet
     * @param  data data to calculate the average
     * @param  n amount of data (max of 255)
     * @return  Returns the average
     */
    void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation){
        DebugSerialUtilitiesLn("averageChauvenet: begin of function");
        float values_kept [n];
        uint8_t n_values = 0;
        for (uint8_t i = 0; i < n; i++) arithmetic_average += data[i];
        arithmetic_average /= (float)n;
        //DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated: ", arithmetic_average);
        for (uint8_t i = 0; i < n; i++) standard_deviation += pow((data[i]-arithmetic_average), 2);
        //DebugSerialUtilitiesLn2("averageChauvenet: standard_deviation al cuadrado: ", standard_deviation);
        standard_deviation = sqrt(standard_deviation / (n-1)); // n-1 is used due to Bessel correction
        //DebugSerialUtilitiesLn2("averageChauvenet: standard deviation calculated: ", standard_deviation);
        // Data with more than 2 times standar deviation from average are discarded
        for (uint8_t i = 0; i < n; i++){
            if(abs(data[i]-arithmetic_average) > KN*standard_deviation){
                //DebugSerialUtilitiesLn4("Value discarded:", data[i], ", in position: ", n);
            }
            else{
                values_kept[n_values] = data[i];
                n_values++;
            }
        }
        for (uint8_t i = 0; i < n_values; i++) arithmetic_average += values_kept[i]; // Once all the values have been discarded the average is made again
        arithmetic_average /= n_values;
        //DebugSerialUtilitiesLn2("averageChauvenet: arithmetic average calculated after discard values: ", arithmetic_average);
    }

    /**
     * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::updateInfo() . It can test one servo. Autodetects ID of the one connected
     * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions). Max of 255 (danger of memory overload)
     * @see checkTimeSpeedRead(). Both are used together to compare speed rate in comunication.
     * @see averageChauvenet()
     */
    void checkTimeGetInfo(uint8_t repetitions){
        DebugSerialSeparation(1);
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            printServoStatusError(error);
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkTimeGetInfo");
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();
        //servo_test1.setBaudRate(BAUD_RATE_G15);
        //servo_test1.begin(BAUD_RATE_G15);

        uint32_t initTime = 0;
        uint32_t timeSpent [repetitions];

        DebugSerialUtilitiesLn("Begin of loop to take data");
        for (uint8_t i = 0; i < repetitions; i++){
            initTime = millis();
            servo_test1.updateInfo();
            timeSpent[i] = millis()-initTime;
            if (servo_test1.getError() != 0){
                DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
                printServoStatusError(servo_test1.getError());
                return;
            }
        }
        DebugSerialUtilitiesLn("Data taken, calling averageChauvenet()");
        float average_time = 0, standard_deviation = 0;
        averageChauvenet(timeSpent,repetitions,average_time, standard_deviation);
        DebugSerialUtilitiesLn2("checkTimeGetInfo: Time spent with ServoRHA::updateInfo: ", average_time);
        DebugSerialUtilitiesLn2("checkTimeGetInfo: number of repetitions: ", repetitions);
        DebugSerialUtilitiesLn("checkTimeSpeedRead: 9 bytes read");
        DebugSerialSeparation(1);
    }

    /**
      * @brief checkTimeInfo checks time spent sending and recieving packet with ServoRHA::SpeedRead(). It can test one servo. Autodetects ID of the one connected.
      * @param {long} repetitions: num of repetitions the test is made (time is the average of this repetitions). Max 255 (danger of memory overload)
      * @see checkTimeGetInfo(). Both are used together to compare speed rate in comunication.
      * @see averageChauvenet()
      */
    void checkTimeSpeedRead(uint8_t repetitions){
        DebugSerialSeparation(1);
        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            printServoStatusError(error);
            DebugSerialUtilitiesLn("Error in servo comunication, end of checkTimeSpeedRead");
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();
        //servo_test1.setBaudRate(BAUD_RATE_G15);
        //servo_test1.begin(BAUD_RATE_G15);

        uint32_t initTime = 0;
        uint32_t timeSpent [repetitions];

        DebugSerialUtilitiesLn("Begin of loop to take data");
        for (uint8_t i = 0; i < repetitions; i++){
            initTime = millis();
            servo_test1.speedRead();
            timeSpent[i] = millis()-initTime;
            if (servo_test1.getError() != 0){
                DebugSerialUtilitiesLn("Error in servo comunication, end of loop to take data");
                printServoStatusError(error);
                return;
            }
        }
        float average_time = 0, standard_deviation = 0;
        averageChauvenet(timeSpent,repetitions,average_time, standard_deviation);
        DebugSerialUtilitiesLn2("checkTimeSpeedRead: Time spent with ServoRHA::speedRead: ", average_time);
        DebugSerialUtilitiesLn2("checkTimeSpeedRead: number of repetitions: ", repetitions);
        DebugSerialUtilitiesLn("checkTimeSpeedRead: 2 bytes read");
        DebugSerialSeparation(1);
    }

    /**
     * @brief extractRegulatorData tests ServoRHA regulator and prints through serial monitor all info (python list style) to make a graphic. It can test one servo. Autodetects ID of the one connected
     * @method extractRegulatorData
     */
    void extractRegulatorData(){
        DebugSerialUtilitiesLn("extractRegulatorData: begin of function");
        DebugSerialSeparation(1);

        ServoRHA servo_broadcast(ALL_SERVO,2,3,8);
        servo_broadcast.init();         //Broadcast initialize
        uint8_t data[10];
        uint16_t error = servo_broadcast.ping(data);
        uint8_t IDcurrent = data[0];
        DebugSerialUtilitiesLn2("ID from servo is: ", IDcurrent);
        if (error != 0 && error != SERROR_IDMISMATCH){  // Ignore ID mismatch as we broadcast to all servo
            printServoStatusError(error);
            DebugSerialUtilitiesLn("Error in servo comunication, end of extractRegulatorData");
            printServoStatusError(error);
            return;
        }
        ServoRHA servo_test1(IDcurrent,2,3,8);
        servo_test1.init();

        servo_test1.setWheelMode();
        servo_test1.setWheelSpeed( 0, CCW, iWRITE_DATA);

        uint16_t torque = 0,
                 speed_current = 0,
                 speed_target = SPEED_TARGET,
                 error_speed = 0;
        uint32_t time_init = 0;
        uint8_t counter = 0;

        time_init = millis();
        Serial.print("n_data = "); Serial.println(SAMPLE_REGULATOR);
        Serial.print("speed_target = "); Serial.println(speed_target);
        Serial.print("regulatorTest = [");
        Serial.print("['"); Serial.print((float)KP_REGULATOR); Serial.print("']");
        while(true){
            if (millis() - time_init > LOOP_FREQUENCY){
                speed_current = servo_test1.speedRead();
                error_speed = speed_target - speed_current;
                torque = servo_test1.regulatorServo(error_speed, KP_REGULATOR);
                servo_test1.setWheelSpeed(torque, CW, iWRITE_DATA);
                time_init = millis();

                Serial.print(",['"); Serial.print(speed_current); Serial.print("','");
                Serial.print(torque); Serial.print("','");
                Serial.print(time_init); Serial.println("']\\");
                counter++;
                if(counter > SAMPLE_REGULATOR){
                    Serial.println("]");
                    DebugSerialSeparation(1);
                    DebugSerialUtilitiesLn("extractRegulatorData: end of function");
                    break;
                }
            }
            else delay(5);
        }
        Serial.print("]");
    }

} //end of MeasureUtilities namespace

/***************************************************************
 *        This code is derived from Cytron G15 examples        *
 ***************************************************************/
namespace ServoUtilities{
    #define LED       13

    /**
     * Function to change the ID of the servo connected to the bus (only one servo can be connected)
     * @method setServoId
     * @param  {uint8_t} new_id ID for the servo
     */
    void setServoId(uint8_t new_id){
        Cytron_G15_Servo g15(ALL_SERVO, 2, 3, 8); // SoftwareSerial: Rx, Tx and Control pin
        //Cytron_G15Shield g15(8); // HardwareSerial: Control pin

        word error = 0;
        byte data[10];

        g15.begin(BAUD_RATE_G15);

        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);
        delay(1000);
        g15.setID(new_id);

        error = g15.ping(data);

        if(error == 0 || error == 0x0400) {// Ignore ID mistmatch since broadcast ID is used to ping the servo
            if(data[0] == new_id) { // Succeed change to new ID!
                while(1) {
                    g15.setLED(new_id, ON);
                    delay(500);
                    g15.setLED(new_id, OFF);
                    delay(500);
                }
            } else {  // Fail, new ID is different
                while(1) {
                    digitalWrite(LED, LOW);
                    delay(500);
                    digitalWrite(LED, HIGH);
                    delay(500);
                }
            }
        } else {  // Fail, other error occur
            while(1) {
                digitalWrite(LED, LOW);
                delay(100);
                digitalWrite(LED, HIGH);
                delay(100);
            }
        }
    }


    #define DEFAULT_ID  0x01
    #define BROADCAST   0xFE
    #define LED 13

    /**
     * Makes a full factory reset and then sets ne baudrate
     * @method fullFactoryResetBR
     * @see #define BAUD_RATE_G15
     */


    word error = 0;
    byte data[10];
    int baudrateMode = 0;

    void fullFactoryResetBR(){

        ServoRHA g15(BROADCAST,2, 3, 8);

        ServoRHA g15_restored(DEFAULT_ID,2, 3, 8);

        pinMode(LED, OUTPUT);

        switch(baudrateMode)
        {
            case 0:
              g15.begin(1200); g15_restored.begin(1200); break;
            case 1:
              g15.begin(2400); g15_restored.begin(2400); break;
            case 2:
              g15.begin(4800); g15_restored.begin(4800); break;
            case 3:
              g15.begin(9600); g15_restored.begin(9600); break;
            case 4:
              g15.begin(19200); g15_restored.begin(19200); break;
            case 5:
              g15.begin(38400); g15_restored.begin(38400); break;
            case 6:
              g15.begin(57600); g15_restored.begin(57600); break;
            case 7:
              g15.begin(115200); g15_restored.begin(115200); break;
            default: break;
        }

        g15.factoryReset();
        delay(100);
        error = g15_restored.ping(data);

        if(error == 0 || error == 0x0400) // Ignore ID mistmatch since broadcast ID is used to ping the servo
        {
            if(data[0] == DEFAULT_ID) // Success
            {
                DebugSerialUtilitiesLn("Servo restored succesfully");
                DebugSerialUtilitiesLn2("Baudrate mode is: ",baudrateMode);
                g15_restored.setBaudRate(BAUD_RATE_G15); // Change to default baudrate
                delay(100);
                g15.end();
                delay(100);
                g15.begin(BAUD_RATE_G15);
                delay(100);

                while(1)
                {
                    g15_restored.setLED(DEFAULT_ID, ON);
                    delay(500);
                    g15_restored.setLED(DEFAULT_ID, OFF);
                    delay(500);
                }
            }
            else // Fail, new ID is different
            {
                DebugSerialUtilitiesLn("ID not expected");
                g15_restored.end();
                if(baudrateMode < 8) {
                      baudrateMode++;
                }
                else {
                    while(1)
                    {
                        digitalWrite(LED, LOW);
                        delay(1000);
                        digitalWrite(LED, HIGH);
                        delay(1000);
                    }
                }
            }
          }
        else // Fail, other error occur
        {
            DebugSerialUtilitiesLn("Other error ocurred");
            g15_restored.end();
            if(baudrateMode < 8) {
                baudrateMode++;
            }
            else {
                while(1)
                {
                    digitalWrite(LED, LOW);
                    delay(200);
                    digitalWrite(LED, HIGH);
                    delay(200);
                }
            }
        }
        delay(100);
    }
} // End of ServoUtilities namespace

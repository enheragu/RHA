/**
 * @file
 * @brief Implements a set of utilities to measure, experimentally, some interesting parameters.
 *
 * Measures real speed of servo, time spent with packet handling, etc
 *
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_13
 * @Project: RHA
 * @Filename: utilities.h
 * @Last modified by:   quique
 * @Last modified time: 21-Sep-2017
 */

#include <Arduino.h>
#include "debug.h"
#include "servo_rha.h"
// #include "cytron_g15_servo.h"
#include "utilities.h"
#include "joint_handler.h"


    #define SPEED 800  // torque send to check speed
    #define SPEED_TARGET 80  // speed target in rpm
    #define KP_REGULATOR 150  // kp for extractRegulatorData function
    #define LOOP_FREQUENCY 100  // in ms
    #define SAMPLE_REGULATOR 150 // samples taken every LOOP_FREQUENCY
    #define BAUD_RATE_G15 57600

    #define CHAUVENET_REPETITIONS 50  // too many repetitions cause memory overfload
    #define KN 1.54 // Chauvenet coeficient for n = 4

namespace MeasureUtilities {

    /**
     * @brief Calculates the average applying chauvenets criterion
     * @method averageChauvenet
     * @param  data data to calculate the average
     * @param  n amount of data (max of 255)
     * @return  Returns the average
     */
    void averageChauvenet(uint32_t *data, uint8_t n, float &arithmetic_average, float &standard_deviation) {
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

        Cytron_G15_Servo g15(BROADCAST,2, 3, 8);

        Cytron_G15_Servo g15_restored(DEFAULT_ID,2, 3, 8);

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

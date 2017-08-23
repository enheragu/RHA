//
// Created by Richard on 10/01/2017.
// Modified by enheragu on 23/08/2017
//

#include <iostream>

#ifndef TRANSITION_ARDUINO_H
#define TRANSITION_ARDUINO_H

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define SERIAL  0x0
#define DISPLAY 0x1

typedef unsigned int word;
typedef bool boolean;
typedef uint8_t byte;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t millis(void);
void delay(uint32_t ms_delay);
void pinMode(int, int);
void analogWrite(int, int);
void digitalWrite(int, int);
int digitalRead(int);
int analogRead(int);

#endif //TRANSITION_ARDUINO_H

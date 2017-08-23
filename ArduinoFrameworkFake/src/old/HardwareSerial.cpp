

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

#include "HardwareSerial.h"
//#include "HardwareSerial_private.h"


#if defined(HAVE_HWSERIAL0) || defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3)


#if defined(HAVE_HWSERIAL0)
  void serialEvent() __attribute__((weak));
  bool Serial0_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL1)
  void serialEvent1() __attribute__((weak));
  bool Serial1_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL2)
  void serialEvent2() __attribute__((weak));
  bool Serial2_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL3)
  void serialEvent3() __attribute__((weak));
  bool Serial3_available() __attribute__((weak));
#endif

void serialEventRun(void)
{
#if defined(HAVE_HWSERIAL0)
  if (Serial0_available && serialEvent && Serial0_available()) serialEvent();
#endif
#if defined(HAVE_HWSERIAL1)
  if (Serial1_available && serialEvent1 && Serial1_available()) serialEvent1();
#endif
#if defined(HAVE_HWSERIAL2)
  if (Serial2_available && serialEvent2 && Serial2_available()) serialEvent2();
#endif
#if defined(HAVE_HWSERIAL3)
  if (Serial3_available && serialEvent3 && Serial3_available()) serialEvent3();
#endif
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerial::_tx_udr_empty_irq(void)
{
  return;
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(unsigned long baud, byte config)
{
  return 0;
}

void HardwareSerial::end()
{
  return 0;
}

int HardwareSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerial::peek(void)
{
  return 1;
}

int HardwareSerial::read(void)
{
  return 50;
}

int HardwareSerial::availableForWrite(void)
{
  return 1;
}

void HardwareSerial::flush()
{
  return;
}

size_t HardwareSerial::write(uint8_t c)
{
  return 1;
}

#endif // whole file
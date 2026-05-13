// Utils.h

#ifndef _UTILS_h
#define _UTILS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include "Arduino.h"

#ifndef DEBUG
#define DEBUG
#endif

#ifdef DEBUG
#define SerialDebugPrintln(a) (Serial.println(a))
#define SerialDebugPrint(a) (Serial.print(a))
#else   //Does nothing if DEBUG NOT defined
#define SerialDebugPrintln(a)
#define SerialDebugPrint(a)
#endif

unsigned long GetEllapsedTimeInSeconds(unsigned long lastTimestamp);
unsigned long GetEllapsedTimeInMillis(unsigned long lastTimestamp);
unsigned long GetEllapsedTimeInMicros(unsigned long lastTimestamp);
#endif //end of #ifndef _UTILS_h


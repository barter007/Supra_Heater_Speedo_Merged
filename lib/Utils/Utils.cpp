//Utils.cpp

#include "Utils.h"

unsigned long GetEllapsedTimeInSeconds(unsigned long lastTimestamp)
{
    return GetEllapsedTimeInMillis(lastTimestamp) / 1000;
}

unsigned long GetEllapsedTimeInMillis(unsigned long lastTimestamp)
{
    unsigned long currTimestamp = millis();
    unsigned long ellapsedTimeInMs;
    if (lastTimestamp > currTimestamp)
    {  //clock has overflowed (reset to 0 passed max value)
        ellapsedTimeInMs = (4294967295 - lastTimestamp + currTimestamp);
    }
    else
    {
        ellapsedTimeInMs = (currTimestamp - lastTimestamp);
    }
    return ellapsedTimeInMs;
}

// Calculates the elapsed microseconds since the last check.
//Automatically handles overflow due to unsigned subtraction properties. 
unsigned long GetEllapsedTimeInMicros(unsigned long lastTimestamp) {
    unsigned long elapsedTimeInMicros = micros() - lastTimestamp;
    return elapsedTimeInMicros;
}


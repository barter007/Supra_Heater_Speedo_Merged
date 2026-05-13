// HeaterValveHelper.h

#ifndef _HEATERVALVEHELPER_h
#define _HEATERVALVEHELPER_h

#include <ESP32Servo.h>
#include <CircularBuffer.hpp>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class HeaterValveHelper
{
public:
	HeaterValveHelper();
	void Begin(int servoPin, int thermistorPin);
	void Loop();

private:
    const int SERVO_MOVE_ANGLE_THRESHOLD = 3;
    const long SERVO_ACTIVATION_DURATION_IN_MS = 500;	//Duration to keep the servo activated (in milliseconds). This number must be high enough to enable full sweep or else the servo will not reach the target angle before deactivation.

    const int MIN_THERMISTOR_VALUE = 300;    // Minimum expected temperature (°C)    //ds le DASH, ça varie de 19 à 31C, mais c'est pas linéaire, la 1ière coche à gauche avait un gap de 10C, vs les autres genre 3C par coche...(de mémoire)
    const int MAX_THERMISTOR_VALUE = 2400;   // Maximum expected temperature (°C)
   
    const int MIN_ANGLE = 70;     // Min Servo angle for minimum temperature
    const int MAX_ANGLE = 140;    // Max Servo angle for maximum temperature

    const int THERMISTOR_READ_INTERVAL_IN_MS = 50;    // How often to read the thermistor value

    #define THERMISTOR_BUFFER_SIZE 10
    CircularBuffer<unsigned long, THERMISTOR_BUFFER_SIZE> _circBufferThermistorValues;	// Circular buffer to store the last thermistor reading

    int _servoPin;
	int _thermistorPin;
	bool _servoIsActivated = false;
    int _previousServoAngle = -1;
    int _targetServoAngle = MIN_ANGLE;
    unsigned long _lastServoActivationTime = 0;
    unsigned long _lastThermistorReadTime = 0;
    int _thermistorAvgValue = 0;

    Servo _myServo;

    int CalculateTargetServoAngleFromThermistorValue(int thermistorValue);
};

#endif	//_HEATERVALVEHELPER_h
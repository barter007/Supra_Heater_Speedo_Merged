// SpeedoHelper.h

#ifndef _SPEEDOHELPER_h
#define _SPEEDOHELPER_h

#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <driver/pcnt.h>
#include "TestHelper.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class SpeedoHelper
{
public:
	SpeedoHelper();
	 void Begin(int stepMotorPulsePin, int stepMotorDirectionPin, int stepMotorResetPin, int wheelSensorInputPin, bool isTestMode);
	 void Loop();

private:
    enum StepMotorDirection
    {
        DIRECTION_CLOCKWISE,
        DIRECTION_ANTI_CLOCKWISE,
    };

    const int STEP_MOTOR_PULSE_DURATION_IN_MICROS = 20; // Duration of each pulse sent to the step motor (in microseconds). Also used for direction change
    const int STEP_MOTOR_STEPS_PER_DEGREE = 12;   // Number of steps the step motor needs to take to move the speedo needle by 1 degree.
    const int NEEDLE_SPEED_MIN_DEGREES_PER_SECOND = 30;   // Minimum speed at which the speedo needle can move in degrees per second.
    const int NEEDLE_SPEED_MAX_DEGREES_PER_SECOND = 60;   // Maximum speed at which the speedo needle can move in degrees per second.
    const int ABS_RING_TEETH_COUNT = 44;    // Number of teeth on the ABS ring that the wheel sensor detects. This is used to calculate the speed from the duration between wheel sensor triggers.
    const int STEP_TARGET_JITTER_PROTECTION_IN_STEPS = 4; // If the target step motor position is within this number of steps from the current position, we won't move the step motor. This is to avoid jitter when the calculated speed is fluctuating around a value that would require the needle to be in a certain position, we can adjust this value based on how much jitter we want to allow vs how responsive we want the speedo to be.
    const unsigned long MIN_DELAY_IN_MICROS_BETWEEN_WHEEL_SENSOR_TRIGGERS = 520; // 520Us is expected delay at 320km/h.Minimum delay in microseconds between wheel sensor triggers to consider them valid. This is used to filter out noise and avoid calculating unrealistically high speeds when the wheel sensor is triggered multiple times in a very short period of time due to noise or other factors.
    const int TARGET_POS_UPDATE_FREQUENCY_IN_MS = 100;    // How often to interpret wheel sensor data and set the new step motor target position (not actually moving the step motor).
    const float WHEEL_CIRCUMFERENCE_IN_METERS = 2.025; // Example: 65.3cm is 0.653m
    const unsigned long FALLBACK_TO_0KMH_AFTER_NO_WHEEL_SENSOR_TRIGGER_FOR_IN_MS = 500; // If no wheel sensor trigger is detected for this duration, we consider the vehicle to be stopped and set the speedo to 0km/h.
    const int PHYSICAL_SPEEDO_ANGLE_DISPLAYED_SPEED_AT_180_DEGREES= 192; // At 180 degrees from 0km/h the physical cluster is displaying what speed in km/h? This is used to calculate the target degree for the speedo needle based on the calculated speed.

    #define WHEEL_SENSOR_BUFFER_SIZE 10	// Number of last durations between wheel sensor triggers to keep track of for speed calculation. This is a trade-off between having a responsive speedo (lower number) vs a more stable speedo (higher number).

    // Pins Definitions
    int _stepMotorPulsePin;
	int _stepMotorDirectionPin;
    int _stepMotorResetPin;
    int _wheelSensorInputPin;

    // Calibration related
    bool _isTestMode = false;   // If true, Test mode activated and takes commands from serial input. WheelSensorInput will be ignored in this mode.

    #define CALIBRATION_DATA_POINTS_COUNT 40
    float _calibrationDataPointsInputKmPerHour[CALIBRATION_DATA_POINTS_COUNT] =         {0, 10, 15, 20, 25, 30.0, 35, 40.0, 45, 50, 55, 60, 65, 70.0, 75, 80.0, 85, 90.0, 95.0, 100.0, 105, 110, 115, 120, 125.0, 130.0, 135.0, 140.0, 145.0, 150, 155, 160, 165, 170.0, 175.0, 180.0, 185.0, 190.0, 195.0, 200};
    float _calibrationDataPointsTargetAngleInDegrees[CALIBRATION_DATA_POINTS_COUNT] =   {0,  4,  6, 10, 15, 19.5, 24, 28.8, 34, 39, 44, 49, 54, 58.8, 64, 69.5, 75, 79.5, 84.5,  89.5,  95, 100, 105, 110, 114.5, 119.5, 124.5, 130.5, 135.5, 140, 145, 150, 155, 160.5, 165.5, 170.5, 175.5, 180.5, 185.5, 191};

    // Other variables
    TestHelper _testHelper;  // Instance of TestHelper to use its functions in test mode
    bool _stepMotorMoveInProgress = false;   // Flag to indicate if a step motor move is currently in progress. This is used to avoid starting a new move before the previous one has finished.
    bool _stepMotorDirectionChangeInProgress = false; // Flag to indicate the step motor is currently changning direction.
    int _currentStepMotorPos = 0;   //This is NOT degrees, it is the steps count.
    int _targetStepMotorPos = 0;    //This is NOT degrees, it is the steps count.
    StepMotorDirection _currentStepMotorDirection = DIRECTION_CLOCKWISE;
    pcnt_unit_t _pcntUnit = PCNT_UNIT_0;
    unsigned long _lastSpeedoNeedleUpdateTimeInMs = 0;
    unsigned long _lastStepMotorPulseStartTimeInMicros = 0;
    unsigned long _lastTargetStepMotorUpdateTimeInMs = 0;
    
    CircularBuffer<unsigned long, WHEEL_SENSOR_BUFFER_SIZE> _bufferDurationsInMicrosBetweenWheelSensorTriggers;	// Circular buffer to store the last durations between wheel sensor triggers for speed calculation

    void ResetStepMotor();
    void PrintCircularBuffer();
    bool IsOutlier(unsigned long x, float mean, float stddev, float k);
    void UpdateSpeedCalculationBufferWithLatestWheelSensorData();
    float GetAverageDurationInMicrosBetweenWheelSensorTriggers();
    float CalculateSpeedInKmPerHourFromAverageDurationInMicros(float averageDurationInMicrosBetweenWheelSensorTriggers);
    bool ChangeStepMotorDirectionIfRequired(StepMotorDirection newDirection);
    unsigned long CalculateStepDeltaTimeInMicros(int needleSpeedInDegreesPerSecond);
    //bool ReadCalibrationCmdFromSerial();
    int GetTargetStepMotorPosFromKmPerHour(float speedInKmPerHour);
    void SetupPcnt();

};

#endif	//_SPEEDOHELPER_h
// TestHelper.h

#ifndef _TESTHELPER_h
#define _TESTHELPER_h

#include "Arduino.h"

class TestHelper
{
public:
    enum TestMode
    {
        TEST_MODE_ANGLE,
        TEST_MODE_KMH,
        TEST_MODE_CALIBRATION_POINTS,   //
        TEST_MODE_JITTER
    };

    TestHelper();
	void Begin(TestMode initialTestMode, int stepMotorStepsPerDegree, float calibrationDataPointsInputKmPerHour[], float calibrationDataPointsTargetAngleInDegrees[], int calibrationPointsCount);
	bool HasNewTestData();
    TestMode GetCurrentTestMode();
    int GetNeedleSpeedDegreesPerSecond();
    float GetTargetStepMotorAngleInDegrees();
    float GetTargetStepMotorKmPerHour();


private:
    enum TestAction
    {
        TEST_ACTION_STEP_UP,
        TEST_ACTION_STEP_DOWN,
        TEST_ACTION_NEED_PARAMS,
        TEST_ACTION_MOVING_TO_STARTUP_POS
    };


    const unsigned int DEFAULT_TIME_IN_MS_BEFORE_NEXT_CALIBRATION_POINT = 1000;
    const unsigned int TEST_FIRST_POS_TIMER = 4000;                     // Time allowed to get to StartupPos before start of next test steps
    #define MAX_BUFFER_SIZE_FOR_CALIBRATION_MODE 30
    char _calibrationCmdBuffer[MAX_BUFFER_SIZE_FOR_CALIBRATION_MODE];
    int _calibrationCmdBufferIndex = 0;
    int _needleSpeedDegreesPerSecond = 0;
    int _stepMotorStepsPerDegree = 0;
    float _targetStepMotorAngleInDegrees = 0.0;
    float _targetStepMotorKmPerHour = 0.0;
    unsigned long _lastStepMoveTimeInMs = 0;
    unsigned long _nextStepMoveDelayInMs = 100;                         //Delay in ms between each step
    int _calibrationPointIndex = 0;
    int _calibrationPointsCount = 0;
    float* _calibrationDataPointsInputKmPerHour;
    float* _calibrationDataPointsTargetAngleInDegrees;
    
    //Jitter test related variables
    unsigned long _jitterStepPosUpCount = 1;                            //Number of step motor to go up from the current position during the jitter test
    unsigned long _jitterStepPosDownCount = 1;                          //Number of step motor to go down from the current position
    
    TestAction _currentTestAction = TEST_ACTION_NEED_PARAMS;            //Used in some test modesto keep track of the next action
    TestMode _currentTestMode = TEST_MODE_ANGLE;

    bool ReadCmdFromSerial();
    bool ParseCommand();
    bool ParseParamsForAngleMode();
    bool ParseParamsForKmhMode();
    bool ParseParamsForCalibrationPointsMode();
    bool ParseParamsForJitterMode();
    void SetCurrentTestMode(TestMode testMode); 
};

#endif // _TESTHELPER_h
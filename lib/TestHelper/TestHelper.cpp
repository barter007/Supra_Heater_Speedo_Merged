// TestHelper.cpp
#include "TestHelper.h"
#include "Utils.h"

// CTOR
TestHelper::TestHelper()
{

}

// Public Functions
void TestHelper::Begin(TestMode initialTestMode, int stepMotorStepsPerDegree, float calibrationDataPointsInputKmPerHour[], float calibrationDataPointsTargetAngleInDegrees[], int calibrationPointsCount)
{
    _calibrationDataPointsInputKmPerHour = calibrationDataPointsInputKmPerHour;
    _calibrationDataPointsTargetAngleInDegrees = calibrationDataPointsTargetAngleInDegrees;
    _calibrationPointsCount = calibrationPointsCount;
    _stepMotorStepsPerDegree = stepMotorStepsPerDegree;
    Serial.println("Available Test modes are : [A]ngle, [K]mh, [C]alibration and [J]itter");
    SetCurrentTestMode(initialTestMode);
}

bool TestHelper::HasNewTestData()
{
    bool hasNewCmd = ReadCmdFromSerial();

    if(!hasNewCmd){
        if(_currentTestMode == TEST_MODE_JITTER) {
            if(_currentTestAction == TEST_ACTION_MOVING_TO_STARTUP_POS && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= TEST_FIRST_POS_TIMER) {
                _currentTestAction = TEST_ACTION_STEP_UP;
                _lastStepMoveTimeInMs = millis();
                hasNewCmd = true;
                Serial.println("CurrentTestAction from TEST_MODE_CALIBRATION_POINTS to TEST_ACTION_STEP_UP");
            }
            else if(_currentTestAction == TEST_ACTION_STEP_UP && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= _nextStepMoveDelayInMs) {
                _currentTestAction = TEST_ACTION_STEP_DOWN;
                _lastStepMoveTimeInMs = millis();
                _targetStepMotorAngleInDegrees = _targetStepMotorAngleInDegrees + ((float)_jitterStepPosUpCount / 12.0); //each step is 1/12 of a degree
                hasNewCmd = true;
            }
            else if(_currentTestAction == TEST_ACTION_STEP_DOWN && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= _nextStepMoveDelayInMs) {
                _currentTestAction = TEST_ACTION_STEP_UP;
                _lastStepMoveTimeInMs = millis();
                _targetStepMotorAngleInDegrees = _targetStepMotorAngleInDegrees - ((float)_jitterStepPosDownCount / 12.0); //each step is 1/12 of a degree
                hasNewCmd = true;
            }
        }
        else if(_currentTestMode == TEST_MODE_CALIBRATION_POINTS) {
            if(_currentTestAction == TEST_ACTION_MOVING_TO_STARTUP_POS && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= TEST_FIRST_POS_TIMER) {
                _currentTestAction = TEST_ACTION_STEP_UP;
                _lastStepMoveTimeInMs = millis();
                hasNewCmd = true;
                Serial.println("CurrentTestAction from TEST_MODE_CALIBRATION_POINTS to TEST_ACTION_STEP_UP");
            }
            else if(_currentTestAction == TEST_ACTION_STEP_UP && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= _nextStepMoveDelayInMs) {

                // Move to the next calibration point
                _targetStepMotorKmPerHour = _calibrationDataPointsInputKmPerHour[_calibrationPointIndex];
                _lastStepMoveTimeInMs = millis(); // Reset the timer for the next calibration point

                Serial.print("Calibration Index: "); Serial.print(_calibrationPointIndex);
                Serial.print("Calibration Point - Target Km/h: "); Serial.print(_targetStepMotorKmPerHour);
                Serial.print(" | Target Angle: "); Serial.print(_calibrationDataPointsTargetAngleInDegrees[_calibrationPointIndex]);

                if(_calibrationPointIndex == _calibrationPointsCount -1) {
                    _currentTestAction = TEST_ACTION_STEP_DOWN;
                    _calibrationPointIndex--;
                    Serial.print(" | Switching calibration direction to DOWN for next calibration point.");
                }
                Serial.println();

                hasNewCmd = true;
            }
            else if(_currentTestAction == TEST_ACTION_STEP_DOWN && GetEllapsedTimeInMillis(_lastStepMoveTimeInMs) >= _nextStepMoveDelayInMs) {

                // Move to the next calibration point
                _targetStepMotorKmPerHour = _calibrationDataPointsInputKmPerHour[_calibrationPointIndex];
                _lastStepMoveTimeInMs = millis(); // Reset the timer for the next calibration point

                Serial.print("Calibration Index: "); Serial.print(_calibrationPointIndex);
                Serial.print("Calibration Point - Target Km/h: "); Serial.print(_targetStepMotorKmPerHour);
                Serial.print(" | Target Angle: "); Serial.print(_calibrationDataPointsTargetAngleInDegrees[_calibrationPointIndex]);

                if (_calibrationPointIndex == 0) {
                    _currentTestAction = TEST_ACTION_STEP_UP;
                    _calibrationPointIndex++;
                    Serial.print(" | Switching calibration direction to UP for next calibration point.");
                }
                Serial.println();

                hasNewCmd = true;
            }            
        }
    }

    return hasNewCmd;
}

TestHelper::TestMode TestHelper::GetCurrentTestMode(){
    return _currentTestMode;
}

int TestHelper::GetNeedleSpeedDegreesPerSecond()
{
    return _needleSpeedDegreesPerSecond;
}

float TestHelper::GetTargetStepMotorAngleInDegrees()
{
    return _targetStepMotorAngleInDegrees;
}
float TestHelper::GetTargetStepMotorKmPerHour()
{
    return _targetStepMotorKmPerHour;
}

// Private Functions
void TestHelper::SetCurrentTestMode(TestMode testMode){
    _currentTestMode = testMode;
    
    // Reset all test data when switching to a new test mode 
    _needleSpeedDegreesPerSecond = 0;
    _targetStepMotorAngleInDegrees = 0.0;
    _targetStepMotorKmPerHour = 0.0;

    _lastStepMoveTimeInMs = millis();

    _currentTestAction = TEST_ACTION_NEED_PARAMS;   //By default, override for specific test if needed below

    if(_currentTestMode == TEST_MODE_ANGLE) {
        Serial.println("Test Mode: Angle - Please input [targetAngle] or [targetAngle];[needleSpeed] without brackets Ex.: 100.5 or 100.5;50");
        Serial.println("[needleSpeed] is optional and the unit is degrees/second. If omited, will use default speed based on the distanct to the target.");
    }
    else if(_currentTestMode == TEST_MODE_KMH) {
        Serial.println("Test Mode: Speed - Please input [targetKMH] or [targetKMH];[needleSpeed] without the brackets Ex.: 100.5 or 105.5;50");
        Serial.println("[needleSpeed] is optional and the unit is degrees/second. If omited, will use default speed based on the distanct to the target.");
    }
    else if(_currentTestMode == TEST_MODE_CALIBRATION_POINTS) {
        _currentTestAction = TEST_ACTION_MOVING_TO_STARTUP_POS; // In calibration mode, we first want to move to the startup position before going through the calibration points, so we set the action accordingly.
        _nextStepMoveDelayInMs = TEST_FIRST_POS_TIMER;
        _calibrationPointIndex = 0;
        Serial.println("Test Mode: Calibration Points - Optional, you can input the [timeInMsBetweenNewCalibrationPoint] Ex.: 1000 or 500 for 1000ms and 500ms respectively.");
    }
    else if(_currentTestMode == TEST_MODE_JITTER) {
        _nextStepMoveDelayInMs = 100;  
        _jitterStepPosUpCount = 1;                            
        _jitterStepPosDownCount = 1;

        Serial.println("Test Mode: Jitter - Please input [StartupAnglePos];[StepPosUpCount];[StepPosDownCount];[StepDelayInMs] without the brackets Ex.: 100;1;1;100");
        Serial.println("Only [StartupAnglePos] is mandatory, the other parameters are optionals.");
        Serial.println("The unit of [StepPosUpCount] and [StepPosDownCount] is in step count (1 step = 1/12 degree).");
        Serial.println("The unit of [StepDelayInMs] is in milliseconds and it represents the delay between each step during the jitter test.");
    }
}

bool TestHelper::ReadCmdFromSerial(){
    bool hasReceivedCmd = false;
    while(Serial.available() > 0) {
        char incomingByte = Serial.read();
        //todo: flèche haut/bas monte de 0.1, pageUp/pageDown de 1
        if(incomingByte == '\n' || incomingByte == '\r') {
            Serial.println(); // Echo enter to console
            hasReceivedCmd = true;
            _calibrationCmdBuffer[_calibrationCmdBufferIndex] = '\0'; // Null-terminate the string

            while (Serial.available()) Serial.read(); // Clear any remaining input in the serial buffer

            hasReceivedCmd = ParseCommand();

            _calibrationCmdBufferIndex = 0; // Reset buffer index for next command
        }
        else if(incomingByte == '\b' || incomingByte == 127) { // Handle backspace (ASCII 8 or 127)
            if(_calibrationCmdBufferIndex > 0) {
                _calibrationCmdBufferIndex--; // Move back the index
                Serial.print("\b \b"); // Erase the last character in the serial monitor (on recule, on met l espace pour effacer le caractère, puis on recule à nouveau)
            }
        }
        else {
            if(_calibrationCmdBufferIndex < MAX_BUFFER_SIZE_FOR_CALIBRATION_MODE - 1) { // Ensure we don't overflow the buffer
                Serial.print(incomingByte); // Echo the incoming byte back to the serial monitor
                _calibrationCmdBuffer[_calibrationCmdBufferIndex++] = incomingByte;
            }
        }
    }
    return hasReceivedCmd;
}

// Called when an Enter is detected.
// Returns True if valid command, Otherwise False
bool TestHelper::ParseCommand() {

    // 1- TestMode Detection
    if(strlen(_calibrationCmdBuffer) == 1 && (_calibrationCmdBuffer[0] == 'a' || _calibrationCmdBuffer[0] == 'A')) {
        SetCurrentTestMode(TestHelper::TEST_MODE_ANGLE);
        return true;
    }
    else if(strlen(_calibrationCmdBuffer) == 1 && (_calibrationCmdBuffer[0] == 'k' || _calibrationCmdBuffer[0] == 'K')) {
        SetCurrentTestMode(TestHelper::TEST_MODE_KMH);
        return true;
    }
    else if(strlen(_calibrationCmdBuffer) == 1 && (_calibrationCmdBuffer[0] == 'c' || _calibrationCmdBuffer[0] == 'C')) {
        SetCurrentTestMode(TestHelper::TEST_MODE_CALIBRATION_POINTS);
        return true;
    }
    else if(strlen(_calibrationCmdBuffer) == 1 && (_calibrationCmdBuffer[0] == 'j' || _calibrationCmdBuffer[0] == 'J')) {
        SetCurrentTestMode(TestHelper::TEST_MODE_JITTER);
        return true;
    }

    // 2- If not TestMode command, we consider it is the parameters of a command
    if(_currentTestMode == TEST_MODE_ANGLE) {
        // Expected format: "targetAngle" or "targetAngle;needleSpeed"
        // Example: "100" or "100.5" or "100;50" or "100.5;50"
        return ParseParamsForAngleMode();
    }
    else if(_currentTestMode == TEST_MODE_KMH) {
        // Expected format: "targetSpeedInKmPerHour"
        // Example: "50" or "50.5"
        return ParseParamsForKmhMode();
    }
    else if(_currentTestMode == TEST_MODE_CALIBRATION_POINTS) {
        // Expected format: "targetAngle" or "targetAngle;needleSpeed"
        // Example: "100" or "100.5" or "100;50" or "100.5;50"
        return ParseParamsForCalibrationPointsMode();
    }
    else if(_currentTestMode == TEST_MODE_JITTER) {
        return ParseParamsForJitterMode();
    }

    return false; //unknown Test Mode
}

bool TestHelper::ParseParamsForAngleMode() {
    // Parse the command, expected format: "targetAngle;needleSpeed" or "targetAngle"
    char* token = strtok(_calibrationCmdBuffer, ";");
    if(token != nullptr) {
        _targetStepMotorAngleInDegrees = atof(token);
        Serial.print("Target Angle in Degrees: "); Serial.print(_targetStepMotorAngleInDegrees);
    }
    token = strtok(nullptr, ";");
    if(token != nullptr) {
        _needleSpeedDegreesPerSecond = atoi(token);
        Serial.print(" | Needle Speed (Degrees/Sec): "); Serial.println(_needleSpeedDegreesPerSecond);
    }
    else
    {
        _needleSpeedDegreesPerSecond = 0;
        Serial.println(" | Needle Speed (Degrees/Sec): DEFAULT");
    }

    return true;
}

bool TestHelper::ParseParamsForKmhMode() {
    // Parse the command, expected format: "targetKMH;needleSpeed" or "targetKMH"
    char* token = strtok(_calibrationCmdBuffer, ";");
    if(token != nullptr) {
        _targetStepMotorKmPerHour = atof(token);
        Serial.print("Target KM/H: "); Serial.print(_targetStepMotorKmPerHour);
    }
    token = strtok(nullptr, ";");
    if(token != nullptr) {
        _needleSpeedDegreesPerSecond = atoi(token);
        Serial.print(" | Needle Speed (Degrees/Sec): "); Serial.println(_needleSpeedDegreesPerSecond);
    }
    else
    {
        _needleSpeedDegreesPerSecond = 0;
        Serial.println(" | Needle Speed (Degrees/Sec): DEFAULT");
    }

    return true;
}

bool TestHelper::ParseParamsForCalibrationPointsMode() {
    _targetStepMotorAngleInDegrees = 0.0;
    _currentTestAction = TEST_ACTION_MOVING_TO_STARTUP_POS;
    
    // Parse the command, expected format: "timeInMsBetweenNewCalibrationPoint"
    int timeInMsBetweenNewCalibrationPoint = atoi(_calibrationCmdBuffer);
    _nextStepMoveDelayInMs = timeInMsBetweenNewCalibrationPoint;
    Serial.print("Time in ms before next calibration point: "); Serial.println(_nextStepMoveDelayInMs);

    return true;
}

bool TestHelper::ParseParamsForJitterMode() {
    //Up to 5 params can be provided for jitter mode, in the following order and separated by ';':
    //StartupAnglePos;StepPosUpCount;StepPosDownCount;StepDelayInMs

    // Param 1: Startup Angle Position
    char* token = strtok(_calibrationCmdBuffer, ";");
    if(token != nullptr) {
        _targetStepMotorAngleInDegrees = atof(token);
        Serial.print("Startup Target Angle in Degrees: "); Serial.print(_targetStepMotorAngleInDegrees);
        _currentTestAction = TEST_ACTION_MOVING_TO_STARTUP_POS;
        _lastStepMoveTimeInMs = millis();
    }
    else
    {
        _currentTestAction = TEST_ACTION_NEED_PARAMS;
        return false;
    }

    // Param 2: Step Position Up Count
    token = strtok(nullptr, ";");
    if(token != nullptr) {
        _jitterStepPosUpCount = atol(token);
    }
    else
    {
        _jitterStepPosUpCount = 1;
    }
    Serial.print(" | Step Position Up Count: "); Serial.print(_jitterStepPosUpCount);

    // Param 3: Step Position Down Count
    token = strtok(nullptr, ";");
    if(token != nullptr) {
        _jitterStepPosDownCount = atol(token);
    }
    else
    {
        _jitterStepPosDownCount = 1;
    }
    Serial.print(" | Step Position Down Count: "); Serial.print(_jitterStepPosDownCount);

    // Param 4: Step Delay In Ms
    token = strtok(nullptr, ";");
    if(token != nullptr) {
        _nextStepMoveDelayInMs = atol(token);
    }
    else
    {
        _nextStepMoveDelayInMs = 100;
    }
    Serial.print(" | Step Delay In Ms: "); Serial.println(_nextStepMoveDelayInMs);

    return true;
}
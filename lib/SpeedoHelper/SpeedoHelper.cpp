#include "SpeedoHelper.h"
#include "Utils.h"
#include "StatsWelford.h"

// CTOR
SpeedoHelper::SpeedoHelper()
{

}

// Public Functions
void SpeedoHelper::Begin(int stepMotorPulsePin, int stepMotorDirectionPin, int stepMotorResetPin, int wheelSensorInputPin, bool isTestMode)
{
    _isTestMode = isTestMode;
    _stepMotorPulsePin = stepMotorPulsePin;
    _stepMotorDirectionPin = stepMotorDirectionPin;
    _stepMotorResetPin = stepMotorResetPin;
    _wheelSensorInputPin = wheelSensorInputPin;

    Serial.println("SpeedoHelper Begin()");
    Serial.print("..stepMotorPulsePin: "); Serial.println(_stepMotorPulsePin);
    Serial.print("..stepMotorDirectionPin: "); Serial.println(_stepMotorDirectionPin);
    Serial.print("..stepMotorResetPin: "); Serial.println(_stepMotorResetPin);
    Serial.print("..wheelSensorInputPin: "); Serial.println(_wheelSensorInputPin);

    pinMode(_stepMotorPulsePin, OUTPUT);
    pinMode(_stepMotorDirectionPin, OUTPUT);
    pinMode(_stepMotorResetPin, OUTPUT);

    ResetStepMotor();

    if(_isTestMode) {
        Serial.println("Test mode ACTIVATED, no data will be read from SpeedSensor");
        _testHelper.Begin(TestHelper::TEST_MODE_ANGLE, STEP_MOTOR_STEPS_PER_DEGREE, _calibrationDataPointsInputKmPerHour, _calibrationDataPointsTargetAngleInDegrees, CALIBRATION_DATA_POINTS_COUNT);
    }
    else {
        Serial.println("SpeedSensor ACTIVATED");
        SetupPcnt();
    }
}

void SpeedoHelper::Loop()
{
    // 1- Compute target step motor pos if needed
    if(_isTestMode){
        if(_testHelper.HasNewTestData())
        {
            if( _testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_ANGLE || 
                _testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_JITTER ||
                _testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_CALIBRATION_POINTS)
            {
                float targetDegree = _testHelper.GetTargetStepMotorAngleInDegrees();
                _targetStepMotorPos = targetDegree * (float)STEP_MOTOR_STEPS_PER_DEGREE; // Convert the target degree to steps for the step motor
            }
            else if(_testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_KMH || _testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_CALIBRATION_POINTS)
            {
                float targetDegree = GetTargetStepMotorPosFromKmPerHour(_testHelper.GetTargetStepMotorKmPerHour());
                _targetStepMotorPos = targetDegree * (float)STEP_MOTOR_STEPS_PER_DEGREE; // Convert the target degree to steps for the step motor
            }
        }
    }
    else if (GetEllapsedTimeInMillis(_lastTargetStepMotorUpdateTimeInMs) >= TARGET_POS_UPDATE_FREQUENCY_IN_MS) {
        UpdateSpeedCalculationBufferWithLatestWheelSensorData();
        _lastTargetStepMotorUpdateTimeInMs = millis();

        float averageDurationInMicrosBetweenWheelSensorTriggers = GetAverageDurationInMicrosBetweenWheelSensorTriggers();
        float speedInKmPerHour = CalculateSpeedInKmPerHourFromAverageDurationInMicros(averageDurationInMicrosBetweenWheelSensorTriggers);
        
        float targetDegree = GetTargetStepMotorPosFromKmPerHour(speedInKmPerHour);
        _targetStepMotorPos = targetDegree * (float)STEP_MOTOR_STEPS_PER_DEGREE; // Convert the target degree to steps for the step motor

        Serial.print("SpeedoHelper.Loop() => averageDurationInMicrosBetweenWheelSensorTriggers: "); Serial.print(averageDurationInMicrosBetweenWheelSensorTriggers);
        Serial.print(" | speedInKmPerHour: "); Serial.print(speedInKmPerHour);
        Serial.print(" | targetDegree: "); Serial.print(targetDegree);
        Serial.print(" | currentStepMotorPos: "); Serial.print(_currentStepMotorPos);
        Serial.print(" | targetStepMotorPos: "); Serial.println(_targetStepMotorPos);
        PrintCircularBuffer();
    }

    // 2- Change Step motor direction if required
    long deltaSteps = _targetStepMotorPos - _currentStepMotorPos;
    StepMotorDirection stepMotorDirection = (deltaSteps > 0) ? DIRECTION_CLOCKWISE : DIRECTION_ANTI_CLOCKWISE;

    if(ChangeStepMotorDirectionIfRequired(stepMotorDirection))
    {
        return; 
    }

    // 3- Move step motor if needed (Start/Stop pulse if needed)
    if(_stepMotorMoveInProgress && GetEllapsedTimeInMicros(_lastStepMotorPulseStartTimeInMicros) >= STEP_MOTOR_PULSE_DURATION_IN_MICROS) {
        // If a step motor move is in progress and the pulse duration has elapsed, end the current pulse, this is to replace the delayMicrosecond() between HIGH/LOW to produce pulse required by step motor
        _stepMotorMoveInProgress = false;
        digitalWrite(_stepMotorPulsePin, LOW);
    }
    else if(_stepMotorDirectionChangeInProgress && GetEllapsedTimeInMicros(_lastStepMotorPulseStartTimeInMicros) >= STEP_MOTOR_PULSE_DURATION_IN_MICROS)
    {
        _stepMotorDirectionChangeInProgress = false;
    }
    else if(!_stepMotorMoveInProgress && !_stepMotorDirectionChangeInProgress && deltaSteps != 0) {
        if(abs(deltaSteps) > (_isTestMode ? 0 : STEP_TARGET_JITTER_PROTECTION_IN_STEPS) || _targetStepMotorPos == 0) {

            int minDelayInMicrosBetweenSteps = CalculateStepDeltaTimeInMicros(NEEDLE_SPEED_MIN_DEGREES_PER_SECOND);
            int maxDelayInMicrosBetweenSteps = CalculateStepDeltaTimeInMicros(NEEDLE_SPEED_MAX_DEGREES_PER_SECOND);

            unsigned long intervalBeforeNextStepMotorMoveInMicros = map(constrain(abs(deltaSteps), 0, 100), 0, 100, minDelayInMicrosBetweenSteps, maxDelayInMicrosBetweenSteps);

            if( _isTestMode && 
                (_testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_ANGLE || _testHelper.GetCurrentTestMode() == TestHelper::TEST_MODE_KMH) &&
                _testHelper.GetNeedleSpeedDegreesPerSecond() > 0) {
                    intervalBeforeNextStepMotorMoveInMicros = CalculateStepDeltaTimeInMicros(_testHelper.GetNeedleSpeedDegreesPerSecond()); 
            }
           
            if(GetEllapsedTimeInMicros(_lastStepMotorPulseStartTimeInMicros) >= intervalBeforeNextStepMotorMoveInMicros) {
                // If no step motor move is in progress and we need to move the step motor, and had high enough delay since last pulse, start a new pulse
                
                digitalWrite(_stepMotorPulsePin, HIGH);
                _lastStepMotorPulseStartTimeInMicros = micros();
                _stepMotorMoveInProgress = true;
                _currentStepMotorPos += (deltaSteps > 0 ? 1 : -1); // Update the current step motor position by one step in the direction of the target
            }
        }
    }
}

// Private Functions
int SpeedoHelper::GetTargetStepMotorPosFromKmPerHour(float speedInKmPerHour) {
    // 1. Handle Out-of-Bounds (Extrapolation)
    if (speedInKmPerHour <= _calibrationDataPointsInputKmPerHour[0]) return _calibrationDataPointsTargetAngleInDegrees[0];
    if (speedInKmPerHour >= _calibrationDataPointsInputKmPerHour[CALIBRATION_DATA_POINTS_COUNT - 1]) return _calibrationDataPointsTargetAngleInDegrees[CALIBRATION_DATA_POINTS_COUNT - 1];

    // 2. Find the segment
    for (int i = 0; i < CALIBRATION_DATA_POINTS_COUNT - 1; i++) {
    if (speedInKmPerHour >= _calibrationDataPointsInputKmPerHour[i] && speedInKmPerHour <= _calibrationDataPointsInputKmPerHour[i+1]) {
        // 3. Apply the Linear Interpolation Formula
        // Equivalent to: map(val, rawValues[i], rawValues[i+1], refValues[i], refValues[i+1])
        return (speedInKmPerHour - _calibrationDataPointsInputKmPerHour[i]) * (_calibrationDataPointsTargetAngleInDegrees[i+1] - _calibrationDataPointsTargetAngleInDegrees[i]) / (_calibrationDataPointsInputKmPerHour[i+1] - _calibrationDataPointsInputKmPerHour[i]) + _calibrationDataPointsTargetAngleInDegrees[i];
    }
    }
    return 0; // Should never reach here
}

bool SpeedoHelper::ChangeStepMotorDirectionIfRequired(StepMotorDirection newDirection)
{
    if(newDirection != _currentStepMotorDirection)
    {
        digitalWrite(_stepMotorDirectionPin, newDirection); // Set direction based on whether we need to move forward or backward
        _stepMotorDirectionChangeInProgress = true;
        _lastStepMotorPulseStartTimeInMicros = micros();
        _currentStepMotorDirection = newDirection;
        return true;
    }

    return false;
}

void SpeedoHelper::SetupPcnt()
{
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = _wheelSensorInputPin;
    pcnt_config.unit = _pcntUnit;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32767;
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(_pcntUnit, 1000); 
    pcnt_filter_enable(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);
}

void SpeedoHelper::UpdateSpeedCalculationBufferWithLatestWheelSensorData() {
    int16_t count = 0;
    pcnt_get_counter_value(_pcntUnit, &count);
    pcnt_counter_clear(_pcntUnit);

    if(count == 0)
    {
        _bufferDurationsInMicrosBetweenWheelSensorTriggers.push(0);
    }
    else
    {
        unsigned long deltaTimePerToothInMicros = GetEllapsedTimeInMillis(_lastTargetStepMotorUpdateTimeInMs) / count * 1000.0; // Convert from ms to micros
        _bufferDurationsInMicrosBetweenWheelSensorTriggers.push(deltaTimePerToothInMicros);
    }

}

unsigned long SpeedoHelper::CalculateStepDeltaTimeInMicros(int needleSpeedInDegreesPerSecond){
    int stepPerSecond = needleSpeedInDegreesPerSecond * STEP_MOTOR_STEPS_PER_DEGREE;
    unsigned long deltaTimeInMicros = 1.0 / (float)stepPerSecond * 1000000.0;   // Time in microseconds between step motor steps based on the desired speed of the needle
    return deltaTimeInMicros;
}

void SpeedoHelper::PrintCircularBuffer(){
    Serial.print("WheelBuffer[");
    Serial.print(_bufferDurationsInMicrosBetweenWheelSensorTriggers.size());
    Serial.print("]: {");
    
    for (int i = 0; i < _bufferDurationsInMicrosBetweenWheelSensorTriggers.size(); i++) {
        Serial.print(_bufferDurationsInMicrosBetweenWheelSensorTriggers[i]);
        if(i < _bufferDurationsInMicrosBetweenWheelSensorTriggers.size() - 1) {
            Serial.print(", ");
        }
    }
    Serial.println("}");
}

void SpeedoHelper::ResetStepMotor()
{
    Serial.print("SpeedoHelper Resetting Step Motor...");

    // To set Step Motor in small increment mode (vs full step mode) and to reset any previous state of the step motor driver
    digitalWrite(_stepMotorResetPin, LOW);
    delay(10);
    digitalWrite(_stepMotorResetPin, HIGH);

    // Move back need to 0km/h
    ChangeStepMotorDirectionIfRequired(DIRECTION_ANTI_CLOCKWISE);
    delayMicroseconds(20);
    for(int i = 0; i < 400; i++) {
        digitalWrite(_stepMotorPulsePin, HIGH); delayMicroseconds(40);
        digitalWrite(_stepMotorPulsePin, LOW);  delay(2);
    }
    _currentStepMotorPos = 0;

    ChangeStepMotorDirectionIfRequired(DIRECTION_CLOCKWISE);

    Serial.println("Done.");
}

bool SpeedoHelper::IsOutlier(unsigned long x, float mean, float stddev, float k) {
    if (stddev == 0.0f) return false; // avoid divide-by-zero / startup issues

    float value = (float)x;
    return fabs(value - mean) > k * stddev;
}

float SpeedoHelper::GetAverageDurationInMicrosBetweenWheelSensorTriggers() {
    const float k = 3.0;  // Threshold for outlier detection, can be adjusted based on how aggressive you want to be in filtering out outliers

    if(_bufferDurationsInMicrosBetweenWheelSensorTriggers.size() == 0) {
        return 0;   // Avoid division by zero, this case can happen when we fallback to 0km/h after no wheel sensor trigger for a while, we clear the buffer in this case to avoid calculating speed based on old data when we start receiving wheel sensor triggers again
    }

    StatsWelford stats;
    // First pass to calculate mean and standard deviation
    for (int i = 0; i < _bufferDurationsInMicrosBetweenWheelSensorTriggers.size(); i++) {
        stats.Add(_bufferDurationsInMicrosBetweenWheelSensorTriggers[i]);
    }
    float average = stats.Mean();
    float stdDev = stats.StdDev();

    stats.Reset();  // Reset stats to calculate the mean again without outliers
    for (int i = 0; i < _bufferDurationsInMicrosBetweenWheelSensorTriggers.size(); i++) {
        if(!IsOutlier(_bufferDurationsInMicrosBetweenWheelSensorTriggers[i], average, stdDev, k)) {
            stats.Add(_bufferDurationsInMicrosBetweenWheelSensorTriggers[i]);
        }
    }    
    
    return stats.Mean();
}

float SpeedoHelper::CalculateSpeedInKmPerHourFromAverageDurationInMicros(float averageDurationInMicrosBetweenWheelSensorTriggers) {
    if(averageDurationInMicrosBetweenWheelSensorTriggers == 0) {
        return 0;   // Avoid division by zero, this case can happen when we fallback to 0km/h after no wheel sensor trigger for a while, we clear the buffer in this case to avoid calculating speed based on old data when we start receiving wheel sensor triggers again
    }
    float durationInSeconds = averageDurationInMicrosBetweenWheelSensorTriggers / 1000000.0; // Convert duration from microseconds to seconds
    float speedInMetersPerSeconds = WHEEL_CIRCUMFERENCE_IN_METERS / (durationInSeconds * (float)ABS_RING_TEETH_COUNT); // Convert duration from microseconds to seconds
    return speedInMetersPerSeconds * 3.6; // Convert from m/s to km/h       
}
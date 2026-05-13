#include "HeaterValveHelper.h"
#include "Utils.h"

// CTOR
HeaterValveHelper::HeaterValveHelper()
{

}

// Public Functions
void HeaterValveHelper::Begin(int servoPin, int thermistorPin)
{
	_servoPin = servoPin;
    _thermistorPin = thermistorPin;
	_servoIsActivated = false;

    Serial.println("HeaterValveHelper Begin()");
    Serial.print("..servoPin: "); Serial.println(_servoPin);
    Serial.print("..thermistorPin: "); Serial.println(_thermistorPin);

      // ESP32 ADC setup
    analogReadResolution(12); // 12-bit
    analogSetPinAttenuation(_thermistorPin, ADC_11db); // adjust for 0-3.3V input
}

// Manage heater valve activation
void HeaterValveHelper::Loop()
{
    if (GetEllapsedTimeInMillis(_lastThermistorReadTime) >= THERMISTOR_READ_INTERVAL_IN_MS)
    {
        _lastThermistorReadTime = millis();
        int thermistorValue = analogRead(_thermistorPin);
        _circBufferThermistorValues.push(thermistorValue);	// Add the latest thermistor reading to the circular buffer for any future use (e.g. smoothing)

        unsigned long sum = 0;
        for (int i = 0; i < _circBufferThermistorValues.size(); i++) {
            sum += _circBufferThermistorValues[i];
        }
        _thermistorAvgValue = sum / _circBufferThermistorValues.size();	// Calculate the average of the thermistor readings in the circular buffer to have a more stable control (e.g. avoid sudden changes in servo angle due to noise in the thermistor readings)
        _targetServoAngle = CalculateTargetServoAngleFromThermistorValue(_thermistorAvgValue);	// Calculate the target servo angle based on the average of the thermistor readings in the circular buffer to have a more stable control (e.g. avoid sudden changes in servo angle due to noise in the thermistor readings)
    }

    // Only move the servo if the target angle is sufficiently different from the previous angle to avoid unnecessary movements
    if (abs(_targetServoAngle - _previousServoAngle) >= SERVO_MOVE_ANGLE_THRESHOLD)
    {
        Serial.print("Thermistor value: ");
        Serial.print(_thermistorAvgValue);
        Serial.print(", Target Servo Angle: ");
        Serial.println(_targetServoAngle);

        _previousServoAngle = _targetServoAngle;
        _servoIsActivated = true;
        _lastServoActivationTime = millis();
        
        _myServo.attach(_servoPin);
        _myServo.write(_targetServoAngle);
    }

    // Deactivate servo after a certain duration to avoid keeping it powered unnecessarily and to allow it to reach the target angle before deactivation
    if(_servoIsActivated && GetEllapsedTimeInMillis(_lastServoActivationTime) >= SERVO_ACTIVATION_DURATION_IN_MS)
    {
        _myServo.detach();
        _servoIsActivated = false;
        Serial.println("Servo motor deactivated");
    }
}

// Private Functions
int HeaterValveHelper::CalculateTargetServoAngleFromThermistorValue(int thermistorValue)
{
    return map(constrain(thermistorValue, MIN_THERMISTOR_VALUE, MAX_THERMISTOR_VALUE), MIN_THERMISTOR_VALUE, MAX_THERMISTOR_VALUE, MIN_ANGLE, MAX_ANGLE);
}

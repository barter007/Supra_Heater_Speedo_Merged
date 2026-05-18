#include <Arduino.h>
#include "Utils.h"
#include "HeaterValveHelper.h"
#include "SpeedoHelper.h"

//Board: ESP32-S2 Thing Plus wrl-17743

// Heater Valve control pins
const int THERMISTOR_PIN = 4;   // GPIO pin connected to the thermistor voltage divider (ADC input) used for Heater Valve control
const int SERVO_PIN = 2;        // GPIO pin connected to the servo signal of the Heater Valve

// Speedo control pins
const int STEP_MOTOR_PULSE_PIN = 12;        // We pulse this pin to produce step motor movement
const int STEP_MOTOR_DIRECTION_PIN = 13;    // We set this pin to determine the direction of step motor movement
const int STEP_MOTOR_RESET_PIN = 33;        // Used to set step sensitivity of the step motor
const int WHEEL_SENSOR_PIN = 1;             // Hall effect sensor reading is on this pin

const bool IS_TEST_MODE = false;   // If true, the speedo needle can move to target angle inputed from serial console. WheelSensorInput will be ignored in this mode.

HeaterValveHelper _heaterValveHelper;
SpeedoHelper _speedoHelper;

void setup() {
  Serial.begin(115200);
  _heaterValveHelper.Begin(SERVO_PIN, THERMISTOR_PIN);
  _speedoHelper.Begin(STEP_MOTOR_PULSE_PIN, STEP_MOTOR_DIRECTION_PIN, STEP_MOTOR_RESET_PIN, WHEEL_SENSOR_PIN, IS_TEST_MODE);
}

void loop() {
  _heaterValveHelper.Loop();
  _speedoHelper.Loop();
}

//todo: francois, jitter, quand le stepposup/down est trop petit, il fait juste monter....pourquoi y descend pas ???
//si on fait juste descendre ex; 0 UP et 1 down, il va aller par en bas...

//calibration il reste tjrs à 0, prob que l'array est mal passé en paramètre ???

//de plus vu j ai juste une var pour le timer, et que je met 4000 au début, il reste à 4000 tout le temps on dirait...

//p-e un prob de délai entre le changement de DIR ET le prochain pulse...

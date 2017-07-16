/**
 * Cuenta los pulsos de un sensor optico de velocidad.
 * 
 * Calcula la velocidad y la cantidad de pasos 
 * transcurridos desde la última medición.
 * 
 * Created by Miguel Braidot, April 13, 2017.
 * Released into the public domain.
 * 
 * TimerOne: https://code.google.com/archive/p/arduino-timerone/downloads
 */
 
#include "SpeedSensor.h"


SpeedSensor::SpeedSensor(int leftPin, int rightPin, unsigned int encoderHoles, int surveyInterval)
{

  _leftPin          = leftPin;
  _rightPin         = rightPin;
  _encoderHoles     = encoderHoles;
  _surveyInterval   = surveyInterval;
  _rmpInterval      = 4000;

  _leftCounterRPM = _leftCounter = _rightCounterRPM = _rightCounter  = 0;
  _leftSteps = _rightSteps      = 0;
}

void SpeedSensor::clear(bool leftMotor){
  if(leftMotor){
    _leftSteps = 0;
    _leftCounter = 0;
  }else{ 
    _rightSteps = 0;
    _rightCounter = 0; 
  }
}


int SpeedSensor::getRPM(bool left)
{
  int rpm = 0;
  if(left){
    rpm = (60 * 1000 / _encoderHoles ) / _surveyInterval * _leftCounterRPM;
    _leftCounterRPM = 0; 
  }else{
    rpm = (60 * 1000 / _encoderHoles ) / _surveyInterval * _rightCounterRPM;
    _rightCounterRPM = 0; 
  }
  
  return rpm;
}


unsigned long SpeedSensor::getSteps(bool left)
{
  unsigned long steps = 0;
  if(left){
    steps = _leftSteps;
  }else{
    steps = _rightSteps;
  }
  return steps;
}


bool SpeedSensor::_pinState(bool left){
  if(left){
    return digitalRead(_leftPin);
  }else{
    return digitalRead(_rightPin);
  }
}


void SpeedSensor::timerInterrupt()
{
  
  /*static unsigned long rpm_timeout = millis() + _rmpInterval;
  if(rpm_timeout < millis()){
    _leftCounter = 0;
    _rightCounter = 0;
    rpm_timeout = millis() + _rmpInterval;
  }*/
  
  // Left side
  static uint16_t stateL = 0; // current debounce status
  stateL = (stateL << 1) | !_pinState(1) | 0xe0000;
  if(stateL == 0xf000){
    _leftCounterRPM++;  // increase +1 the rpm counter value
    _leftCounter++;  // increase +1 the counter value
    _leftSteps++;   // increase steps for shaft position
  }
  
  // Right side
  static uint16_t stateR = 0;
  stateR = (stateR << 1) | !_pinState(0) | 0xe0000;
  if(stateR == 0xf000){
    _rightCounterRPM++;  // increase +1 the rpm counter value
    _rightCounter++;
    _rightSteps++;
  }
  
}


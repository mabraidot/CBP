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

#ifndef SpeedSensor_h
#define SpeedSensor_h

#include "Arduino.h"
#include <TimerOne.h>



class SpeedSensor
{
  public:
    SpeedSensor(int leftPin, int rightPin, unsigned int encoderHoles, int surveyInterval);
    int getRPM(bool left);
    unsigned long getSteps(bool left);
    void timerInterrupt();
    void SpeedSensor::clear(bool leftMotor);
  //private:
    bool _pinState(bool left);
    int _leftPin;
    unsigned int _leftCounter;
    unsigned long _leftSteps;
    int _rightPin;
    unsigned int _rightCounter;
    unsigned long _rightSteps;
    unsigned int _encoderHoles;
    int _surveyInterval;
    
};

extern SpeedSensor Speed;


#endif

#include "DCMotorServo.h";


DCMotorServo::DCMotorServo(SpeedSensor &Speed, uint8_t pin_dir_1, uint8_t pin_dir_2, uint8_t pin_PWM_output, bool leftMotor) : _position(Speed)
{
  _pin_PWM_output = pin_PWM_output;
  _pin_dir_1 = pin_dir_1;
  _pin_dir_2 = pin_dir_2;
  
  //Direction and PWM output
  pinMode(_pin_dir_1, OUTPUT);
  pinMode(_pin_dir_2, OUTPUT);
  pinMode(_pin_PWM_output, OUTPUT);

  _leftMotor = leftMotor;
  
  _PWM_output = 0;  
  _pwm_skip = 30;
  _position_accuracy = 0;
  _position_direction = 1;
  
  _PID_input = 0; //_position.getSteps(_leftMotor);
  _PID_output = 0;
  _PID_setpoint = _PID_input;
  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint,.1,.2,.1, DIRECT);

  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip-255, 255-_pwm_skip);
  //turn the PID on
  myPID->SetMode(AUTOMATIC);



  /*_acceleration = 20.0;
  _PID_speed_setpoint = 70;
  _PID_speed_input = 0;
  _PID_speed_output = 0;
  speedPID = new PID(&_PID_speed_input, &_PID_speed_output, &_PID_speed_setpoint,1,1,0, DIRECT);

  speedPID->SetSampleTime(50);
  speedPID->SetOutputLimits(-255, 255);
  //turn the PID on
  speedPID->SetMode(AUTOMATIC);
  */
}

/**
 * SPEED
 */
void DCMotorServo::setSpeed(int new_speed)
{
  _PID_speed_setpoint = new_speed;
}



/**
 * PWM
 */

void DCMotorServo::setCurrentPosition(int new_position)
{
  _PID_input = new_position;
  _position_direction = new_position/abs(new_position);
}

void DCMotorServo::setAccuracy(unsigned int range)
{
  _position_accuracy = range;
}

bool DCMotorServo::setPWMSkip(uint8_t range)
{
  if ( 0 <= range && range < 255) {
    _pwm_skip = range;
    return 1;
  }
  else
    return 0;
}

bool DCMotorServo::finished()
{
  if (abs(_PID_setpoint - _PID_input) <= _position_accuracy && _PWM_output == 0)
    return 1;
  return 0;
 
}

void DCMotorServo::move(int new_rela_position)
{
  //use _PID_setpoint so that we don't introduce errors of _position_accuracy
  _PID_setpoint = _PID_setpoint + new_rela_position;
  _position_direction = _PID_setpoint/abs(_PID_setpoint);
}

void DCMotorServo::moveTo(int new_position)
{
  _PID_setpoint = new_position;
  _position_direction = _PID_setpoint/abs(_PID_setpoint);
}

int DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

int DCMotorServo::getActualPosition()
{
  return _position.getSteps(_leftMotor) * _position_direction;
}

void DCMotorServo::run() {

  _PID_input = _position.getSteps(_leftMotor) * (double) _position_direction;
  myPID->Compute();
  _PWM_output = abs(_PID_output) + _pwm_skip;
  if (abs(_PID_setpoint - _PID_input) <= _position_accuracy)
  {
    myPID->SetMode(MANUAL);
    _PID_output = 0;
    _PWM_output = 0;
    _position_direction = 1;
  }
  else
  {
    myPID->SetMode(AUTOMATIC);
  }

  _pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
  
}

void DCMotorServo::stop() {
  
  myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  _position_direction = 1;
  
  analogWrite(_pin_PWM_output, _PWM_output);
  digitalWrite(_pin_dir_1, LOW);
  digitalWrite(_pin_dir_2, LOW);
  
}

void DCMotorServo::_pick_direction() {
  
  if (_PID_output < 0)
  {
    digitalWrite(_pin_dir_1, LOW);
    digitalWrite(_pin_dir_2, HIGH);
  }
  else
  {

    digitalWrite(_pin_dir_1, HIGH);
    digitalWrite(_pin_dir_2, LOW);
  }
  
}



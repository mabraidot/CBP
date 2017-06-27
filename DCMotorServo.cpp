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
  
  _PID_input = 0;
  _PID_output = 0;
  _PID_setpoint = 0;
  posPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint,1,1,0, DIRECT);
  posPID->SetSampleTime(50);

  // Max PWM value
  if(leftMotor){
    posPID->SetOutputLimits((-1 * LEFT_DCMOTOR_MAX_PWM), LEFT_DCMOTOR_MAX_PWM);
  }else{
    posPID->SetOutputLimits((-1 * RIGHT_DCMOTOR_MAX_PWM), RIGHT_DCMOTOR_MAX_PWM);
  }
  //turn the PID on
  posPID->SetMode(MANUAL);
  
  /*
  _PID_speed_setpoint = 500; //RPM
  _PID_speed_input = 0;
  _PID_speed_output = 0;
  speedPID = new PID(&_PID_speed_input, &_PID_speed_output, &_PID_speed_setpoint,1,1,0, DIRECT);

  speedPID->SetSampleTime(50);
  speedPID->SetOutputLimits(0, 255); // Max RPM
  //turn the PID on
  speedPID->SetMode(MANUAL);
  */
  
}

/**
 * SPEED
 */
/*void DCMotorServo::setSpeed(int new_speed)
{
  _PID_speed_setpoint = new_speed;
}*/



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

  new_rela_position = new_rela_position * ENCODER_RATIO;
  
  //use _PID_setpoint so that we don't introduce errors of _position_accuracy
  _PID_setpoint = _PID_setpoint + new_rela_position;
  _position_direction = _PID_setpoint/abs(_PID_setpoint);
}


void DCMotorServo::moveTo(int new_position)
{
  new_position = (double) new_position * ENCODER_RATIO;
  
  if(_PID_setpoint != new_position){
    _position.clear(_leftMotor);
    _PID_setpoint = new_position;
    _position_direction = _PID_setpoint/abs(_PID_setpoint);
  }
}

int DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

int DCMotorServo::getActualPosition()
{
  return _position.getSteps(_leftMotor) * _position_direction;
}

int DCMotorServo::getActualRPM()
{
  return _position.getRPM(_leftMotor);
}

void DCMotorServo::clearEncoder(void)
{
  _position.clear(_leftMotor);
}


void DCMotorServo::freeRun(int speedPWM) {
  _pick_direction();
  if(speedPWM > 0){
    analogWrite(_pin_PWM_output, speedPWM);
    _PWM_output = speedPWM;
  }else{
    if(_leftMotor){
      analogWrite(_pin_PWM_output, LEFT_DCMOTOR_MAX_PWM);
      _PWM_output = LEFT_DCMOTOR_MAX_PWM;
    }else{
      analogWrite(_pin_PWM_output, RIGHT_DCMOTOR_MAX_PWM);
      _PWM_output = RIGHT_DCMOTOR_MAX_PWM;
    }
  }
}

void DCMotorServo::run() {

  _PID_input = (double) _position.getSteps(_leftMotor) * _position_direction;
  //_PID_speed_input = (double) _position.getRPM(_leftMotor);

  //speedPID->Compute();
  posPID->Compute();
  //_PWM_output = (int) abs(_PID_speed_output);
  _PWM_output = (int) abs(_PID_output);

  if (abs(_PID_setpoint - _PID_input) <= _position_accuracy)
  {
    stop();
  }
  else
  {
    //speedPID->SetMode(AUTOMATIC);
    posPID->SetMode(AUTOMATIC);
  }

  _pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
  
}

void DCMotorServo::stop() {

  /*
  if(_PWM_output > 0){
    analogWrite(_pin_PWM_output, 255);
    if (_position_direction < 0){
      digitalWrite(_pin_dir_1, HIGH);
      digitalWrite(_pin_dir_2, LOW);
    }else{
      digitalWrite(_pin_dir_1, LOW);
      digitalWrite(_pin_dir_2, HIGH);
    }
    delay(50);
  }
  */
  
  //speedPID->SetMode(MANUAL);
  posPID->SetMode(MANUAL);

  _PID_input = 0;
  _PID_output = 0;
  _PWM_output = 0;
  _PID_setpoint = 0;
  _position_direction = 1;

  //_PID_speed_input = 0;
  //_PID_speed_output = 0;

  analogWrite(_pin_PWM_output, 0);
  digitalWrite(_pin_dir_1, LOW);
  digitalWrite(_pin_dir_2, LOW);

  _position.clear(_leftMotor);
  
}

void DCMotorServo::_pick_direction() {
  
  if (_position_direction < 0)
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



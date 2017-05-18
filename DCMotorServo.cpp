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
  
  _feed = 200;
  //_acceleration = 20.0;
  _PID_speed_setpoint = 0;
  _PID_speed_input = 0;
  _PID_speed_output = 0;
  speedPID = new PID(&_PID_speed_input, &_PID_speed_output, &_PID_speed_setpoint,1,1,0, DIRECT);

  speedPID->SetSampleTime(50);
  speedPID->SetOutputLimits(-255, 255);
  //turn the PID on
  speedPID->SetMode(MANUAL);
  _running = false;
  
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
  //if (abs(_PID_setpoint - _PID_input) <= _position_accuracy && _PWM_output == 0)
  if (abs(_PID_setpoint - _PID_input) <= _position_accuracy && !_running)
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

void DCMotorServo::run() {

  _PID_input = _position.getSteps(_leftMotor) * (double) _position_direction;
  
  _PID_speed_setpoint = _feed;
  _PID_speed_input = _position.getRPM(_leftMotor);
  speedPID->Compute();
  _PWM_output = abs(_PID_speed_output);

  if (abs(_PID_setpoint - _PID_input) <= _position_accuracy)
  {
    stop();
  }
  else
  {
    _running = true;
    speedPID->SetMode(AUTOMATIC);
  }

  _pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
  
}

/*void DCMotorServo::runTrapezoidal(void) {
  
  long a1 = millis();
  _PID_input = _position.getSteps(_leftMotor) * (double) _position_direction;
  int distance = _PID_setpoint - _PID_input;
  boolean dirPos = _position_direction;
  
  float xm = _feed * _feed / _acceleration;
  float t1, t2;
  if (distance <= xm) t1 = t2 = sqrt(distance / _acceleration); // triangular
  else { // trapezoidal
    t1 = sqrt(xm / _acceleration); // t1 = end of accel
    t2 = (distance - xm) / _feed + t1; // t2 = end of coasting
  }
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  float da = _acceleration * dt;
  float covered = _PID_setpoint;
  float maxt = t1 + t2;
  while (t < maxt) {
    t += dt;
    if (t < t1) spd += da; else if (t >= t2) spd -= da;
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    
    _PID_input = _position.getSteps(_leftMotor) * (double) _position_direction;
    _PID_setpoint = covered;
    while(!myPID->Compute()); // espero a que termine el cálculo
    
    _PID_speed_setpoint = abs(_PID_output) + _pwm_skip;
    speedPID->Compute();
    _PWM_output = abs(_PID_speed_output);
  
    _pick_direction();
    analogWrite(_pin_PWM_output, _PWM_output); 
  }
}
*/

void DCMotorServo::stop() {

  speedPID->SetMode(MANUAL);
  _running = false;
  
  _PID_input = 0;
  _PID_output = 0;
  _PWM_output = 0;
  _PID_setpoint = 0;
  _position_direction = 1;

  _PID_speed_setpoint = 0;
  _PID_speed_input = 0;
  _PID_speed_output = 0;

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



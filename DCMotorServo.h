#include <PID_v1.h>
#include "SpeedSensor.h"


class DCMotorServo {
public:
  //DCMotorServo(SpeedSensor& Speed, uint8_t pin_dir_1 = 6, uint8_t pin_dir_2 = 7, uint8_t pin_pwm_output = 3, bool leftMotor = true): _position(Speed) {};
  DCMotorServo(SpeedSensor &Speed, uint8_t pin_dir_1 = 6, uint8_t pin_dir_2 = 7, uint8_t pin_pwm_output = 3, bool leftMotor = true);
  PID * myPID;
  PID * speedPID;
  void run();
  void stop();
  void move(int new_rela_position);
  void moveTo(int new_position);
  int getRequestedPosition();
  int getActualPosition();
  bool finished();
  bool setPWMSkip(uint8_t range);
  void setAccuracy(unsigned int range);
  void setCurrentPosition(int new_position);
  SpeedSensor & _position;
  
  void setSpeed(int new_speed);
  void runTrapezoidal(int destination);
  
  
private:
  uint8_t _pin_PWM_output, _pin_dir_1, _pin_dir_2;
  double _PID_setpoint, _PID_input, _PID_output;
  double _PID_speed_setpoint, _PID_speed_input, _PID_speed_output;
  uint8_t _PWM_output;
  
  bool _leftMotor;
  uint8_t _pwm_skip;            // The range of PWM to skip (for me, I set it to 50 because duty-cycles under 50/255 are not enough to surpass motor and gearing frictions)
  uint8_t _position_accuracy;   // Set to the highest tolerable inaccuracy (units are encoder counts)
  int _position_direction;
  void _pick_direction();

  float _acceleration;         // Desired acceleration in mm/s^2
};


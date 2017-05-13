#include "Configuration.h"
#include "DCMotorServo.h"
#include "SpeedSensor.h"

SpeedSensor _position(PIN_ENCODER1, PIN_ENCODER2, ENCODER_HOLES, ENCODER_QUERY_INTERVAL);

DCMotorServo leftMotor = DCMotorServo(_position, PIN_LEFT_DCMOTOR_DIR1, PIN_LEFT_DCMOTOR_DIR2, PIN_LEFT_DCMOTOR_PWM, true);
DCMotorServo rightMotor = DCMotorServo(_position, PIN_RIGHT_DCMOTOR_DIR1, PIN_RIGHT_DCMOTOR_DIR2, PIN_RIGHT_DCMOTOR_PWM, false);

int dcmoto_move_cm = (int) 100 * ENCODER_RATIO;

void setup() {
  Serial.begin(115200);
  Timer1.initialize(50);
  Timer1.attachInterrupt( timerInterrupt );

  //Tune the servo feedback
  //Determined by trial and error
  //leftMotor.myPID->SetTunings(0.1,0.15,0.05);
  leftMotor.myPID->SetTunings(4.01,5.99,0.01);
  rightMotor.myPID->SetTunings(4.01,5.99,0.01);
}

void loop() {

  debug();
  
  forward(dcmoto_move_cm);
  //backward(dcmoto_move_cm);
  //rotateRight();
  //rotateLeft();
  
}

void timerInterrupt(){
  _position.timerInterrupt();
}

void forward(int movement_distance_cm){

  leftMotor.moveTo(movement_distance_cm);
  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }

  rightMotor.moveTo(movement_distance_cm);
  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
  }
  
}

void backward(int movement_distance_cm){
  
  movement_distance_cm = -1 * movement_distance_cm;
  forward(movement_distance_cm);
  
}

void rotateRight(void){

  leftMotor.moveTo(15);
  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }

  rightMotor.moveTo(-15);
  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
  }
  
}

void rotateLeft(void){

  leftMotor.moveTo(-15);
  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }

  rightMotor.moveTo(15);
  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
  }
  
}


void debug(){
  
  static unsigned long serial_timeout = millis() + 1000;
  if(serial_timeout < millis()){
    Serial.print("Left Destino:    ");
    Serial.println(leftMotor.getRequestedPosition());
    Serial.print("Right Destino:    ");
    Serial.println(rightMotor.getRequestedPosition());
    Serial.print("Left Act.Pos.:   ");
    Serial.println(leftMotor.getActualPosition());
    Serial.print("Right Act.Pos.:   ");
    Serial.println(rightMotor.getActualPosition());
    
    serial_timeout = millis() + 1000;
  }
}


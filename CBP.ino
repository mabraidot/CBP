#include "Configuration.h"
#include "DCMotorServo.h"
#include "SpeedSensor.h"
#include "Planner.h"

SpeedSensor _position(PIN_ENCODER1, PIN_ENCODER2, ENCODER_HOLES, ENCODER_QUERY_INTERVAL);

DCMotorServo leftMotor = DCMotorServo(_position, PIN_LEFT_DCMOTOR_DIR1, PIN_LEFT_DCMOTOR_DIR2, PIN_LEFT_DCMOTOR_PWM, true);
DCMotorServo rightMotor = DCMotorServo(_position, PIN_RIGHT_DCMOTOR_DIR1, PIN_RIGHT_DCMOTOR_DIR2, PIN_RIGHT_DCMOTOR_PWM, false);
Planner plan = Planner();

int dcmoto_move_cm = (int) 10 * ENCODER_RATIO;
char action = "";

void setup() {
  Serial.begin(115200);
  Timer1.initialize(50);
  Timer1.attachInterrupt( timerInterrupt );

  //Tune the servo feedback
  //Determined by trial and error
  //leftMotor.myPID->SetTunings(0.1,0.15,0.05);
  
  //leftMotor.myPID->SetTunings(4.01,5.99,0.01);
  //rightMotor.myPID->SetTunings(4.01,5.99,0.01);
  
}

void loop() {

  switch(action){
    case 'D':
      debug();
      break;
  }
  
  if (Serial.available()) process_serial();

  /** 
   * si hay comandos en la queue (plan.count > 0) 
   * leer el comando de la tail
   * si no esta ocupado (plan.busy = true)
   * setear el moveto del motor
   * fin si
   * resto del bloque de control del motor stop/run
   * si el motor termino
   * mover la tail del buffer al siguiente comando
   * fin si
   * fin si hay comandos en la queue
   */
  if(!plan.isEmpty()){
    Planner::bufferRing bufferRing = plan.get();
    
    // Si devolvió un comando válido y lo puso en ocupado
    if(bufferRing.busy){
      leftMotor.moveTo(bufferRing.leftPosition);
      rightMotor.moveTo(bufferRing.rightPosition);
    }
    
    // Si ambos motores terminaron el comando, liberar el planner 
    // para poder tomar otro comando
    if (leftMotor.finished() && rightMotor.finished()) {
      plan.next();
    }
  }
  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }
  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
  }
  
}

void process_serial(){
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  action = cmd;
  switch (cmd) {
    case 'H': help(); break;
    case 'D': break;
    case 'F': dcmoto_move_cm = Serial.parseInt() * ENCODER_RATIO; plan.put(dcmoto_move_cm, dcmoto_move_cm); break;
    case 'B': dcmoto_move_cm = Serial.parseInt() * ENCODER_RATIO * -1; plan.put(dcmoto_move_cm, dcmoto_move_cm); break;
    case 'L': plan.put(-15, 15); break;
    case 'R': plan.put(15, -15); break;
    case 'S': stopMotors(); break;
  }
  
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)

  
}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F(""));
  Serial.println(F("H: will print this help message again"));
  Serial.println(F("D: will print debug information"));
  Serial.println(F("F123: sets the target destination for the motor to 123 cm, Forward"));
  Serial.println(F("B123: sets the target destination for the motor to 123 cm, Backward"));
  Serial.println(F("L: sets the target destination for the motor to 90 degrees, Left"));
  Serial.println(F("R: sets the target destination for the motor to 90 degrees, Right"));
  Serial.println(F("S: stop the motor"));
}



void timerInterrupt(){
  _position.timerInterrupt();
}


void stopMotors(){
  leftMotor.stop();
  rightMotor.stop();
}

void forward(int movement_distance_cm){

  leftMotor.moveTo(movement_distance_cm);
  rightMotor.moveTo(movement_distance_cm);

  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
    //leftMotor.runTrapezoidal();
  }

  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
    //rightMotor.runTrapezoidal();
  }
  
}

void backward(int movement_distance_cm){
  
  movement_distance_cm = -1 * movement_distance_cm;
  forward(movement_distance_cm);
  
}

void rotateRight(void){

  leftMotor.moveTo(15);
  rightMotor.moveTo(-15);

  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }

  if (rightMotor.finished()) {
    rightMotor.stop();
  }else{
    rightMotor.run();
  }
  
}

void rotateLeft(void){

  leftMotor.moveTo(-15);
  rightMotor.moveTo(15);

  if (leftMotor.finished()) {
    leftMotor.stop();
  }else{
    leftMotor.run();
  }

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

    Serial.print("Left RPM:   ");
    Serial.println(leftMotor.getActualRPM());
    Serial.print("Right RPM:   ");
    Serial.println(rightMotor.getActualRPM());

    Serial.print("count: ");
    Serial.println(plan.count);
    Serial.print("head: ");
    Serial.println(plan.head);
    Serial.print("tail: ");
    Serial.println(plan.tail);

    
    serial_timeout = millis() + 1000;
  }
}


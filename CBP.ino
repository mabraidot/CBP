#include "Configuration.h"
#include "DCMotorServo.h"
#include "SpeedSensor.h"
#include "Planner.h"

SpeedSensor _position(PIN_ENCODER1, PIN_ENCODER2, ENCODER_HOLES, ENCODER_QUERY_INTERVAL);

DCMotorServo leftMotor = DCMotorServo(_position, PIN_LEFT_DCMOTOR_DIR1, PIN_LEFT_DCMOTOR_DIR2, PIN_LEFT_DCMOTOR_PWM, true);
DCMotorServo rightMotor = DCMotorServo(_position, PIN_RIGHT_DCMOTOR_DIR1, PIN_RIGHT_DCMOTOR_DIR2, PIN_RIGHT_DCMOTOR_PWM, false);
Planner plan = Planner();
Planner::bufferRing bufferRing;

int dcmoto_move_cm = 0;
char action;

void setup() {
  Serial.begin(115200);
  Timer1.initialize(50);
  Timer1.attachInterrupt( timerInterrupt );

  int sampleTime = 50;
  leftMotor.posPID->SetSampleTime(sampleTime);
  rightMotor.posPID->SetSampleTime(sampleTime);
  
  float kP = 4;
  float kI = 70;
  float kD = 1;
  
  /*float kP = 100;
  float kI = 10;
  float kD = 1;
  */
  leftMotor.posPID->SetTunings(kP,kI,kD);
  rightMotor.posPID->SetTunings(kP,kI,kD);
  
  // TEST: a little choreography at the beginning
  plan.put(120,120);
  plan.put(ENCODER_TURN_CM,-ENCODER_TURN_CM);
  plan.put(30,30);
  plan.put(-ENCODER_TURN_CM,ENCODER_TURN_CM);
  plan.put(-20,-20);
  
}

void loop() {
  
  switch(action){
    case 'D':
      debug();
      break;
  }
  
  if (Serial.available()) process_serial();
  process_plan();
  
}

void process_plan(){

  /*leftMotor.freeRun(247);
  rightMotor.freeRun(255);
  return;
  */
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
	  bufferRing = plan.get();
    
    // Si devolvió un comando válido y lo puso en ocupado
    if(bufferRing.busy){
      leftMotor.moveTo(bufferRing.leftPosition);
      rightMotor.moveTo(bufferRing.rightPosition);
    }
    
    // Si ambos motores terminaron el comando, liberar el planner 
    // para poder tomar otro comando
    if (leftMotor.finished() && rightMotor.finished()) {
      delay(1000);
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
    case 'F': dcmoto_move_cm = Serial.parseInt(); plan.put(dcmoto_move_cm, dcmoto_move_cm); break;
    case 'B': dcmoto_move_cm = Serial.parseInt() * -1; plan.put(dcmoto_move_cm, dcmoto_move_cm); break;
    case 'L': plan.put(-ENCODER_TURN_CM, ENCODER_TURN_CM); break;
    case 'R': plan.put(ENCODER_TURN_CM, -ENCODER_TURN_CM); break;
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


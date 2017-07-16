#include "Configuration.h"
#include "DCMotorServo.h"
#include "SpeedSensor.h"
#include "Planner.h"
#include <NewPing.h>

SpeedSensor _position(PIN_ENCODER1, PIN_ENCODER2, ENCODER_HOLES, ENCODER_QUERY_INTERVAL);

DCMotorServo leftMotor = DCMotorServo(_position, PIN_LEFT_DCMOTOR_DIR1, PIN_LEFT_DCMOTOR_DIR2, PIN_LEFT_DCMOTOR_PWM, true);
DCMotorServo rightMotor = DCMotorServo(_position, PIN_RIGHT_DCMOTOR_DIR1, PIN_RIGHT_DCMOTOR_DIR2, PIN_RIGHT_DCMOTOR_PWM, false);
Planner plan = Planner();
Planner::bufferRing bufferRing;

int dcmoto_move_cm = 0;
char action;
char free_run = 0;

NewPing sonar(PIN_SONAR_TRIGGER, PIN_SONAR_ECHO, SONAR_MAX_DISTANCE);
char sonar_distance = 0;
char sonar_busy = 0;
char sonar_query_interval = 200;

void setup() {
  Serial.begin(115200);
  Timer1.initialize(50);
  Timer1.attachInterrupt( timerInterrupt );

  int sampleTime = 50;
  leftMotor.posPID->SetSampleTime(sampleTime);
  rightMotor.posPID->SetSampleTime(sampleTime);
  
  float kP = 0.04;
  float kI = 10;
  float kD = 0.01;
  /*
  float kP = 4;
  float kI = 70;
  float kD = 1;
  */
  leftMotor.posPID->SetTunings(kP,kI,kD);
  rightMotor.posPID->SetTunings(kP,kI,kD);
  
  // TEST: a little choreography at the beginning
  /*plan.put(120,120);
  plan.put(ENCODER_TURN_CM,-ENCODER_TURN_CM);
  plan.put(30,30);
  plan.put(-ENCODER_TURN_CM,ENCODER_TURN_CM);
  plan.put(-20,-20);
  */
}

void loop() {
  
  switch(action){
    case 'D':
      debug();
      break;
  }
  
  if (Serial.available()) process_serial();

  free_run = 1;
  process_obstacles();
  process_plan();
  
}

void process_obstacles(){

  static unsigned long sonar_timeout = millis() + sonar_query_interval;
  if(sonar_timeout < millis()){
    sonar_distance = sonar.ping_cm();
    
    if(sonar_distance > 0 && sonar_distance <= SONAR_MIN_DISTANCE && !sonar_busy){
      sonar_busy = 1;
      stopMotors();
      plan.init(true);
      plan.put(ENCODER_TURN_CM,-ENCODER_TURN_CM);
      delay(200);
      
    }else if(sonar_distance == 0 || sonar_distance > SONAR_MIN_DISTANCE || plan.isEmpty()){
      sonar_busy = 0;
    }
    
    sonar_timeout = millis() + sonar_query_interval;
  }
}


void process_plan(){

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
  }else if(free_run){
    leftMotor.freeRun(0);
    rightMotor.freeRun(0);
    return;
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

  static int serial_interval = 1000;
  static double sensorLeft = 0;
  static double sensorRight = 0;
  static unsigned long serial_timeout = millis() + serial_interval;
  if(serial_timeout < millis()){
    
    /*
    Serial.println("------------------------------------------------");
    Serial.print("Left Current sensing:    ");
    sensorLeft = leftMotor.getCurrentSenseValue();
    Serial.println(sensorLeft);
    
    Serial.print("Right Current sensing:    ");
    sensorRight = rightMotor.getCurrentSenseValue();
    Serial.println(sensorRight);
    */
    
    /*Serial.println("------------------------------------------------");
    Serial.print("Left Destino:    ");
    Serial.println(leftMotor.getRequestedPosition());
    Serial.print("Right Destino:    ");
    Serial.println(rightMotor.getRequestedPosition());
    Serial.print("Left Act.Pos.:   ");
    Serial.println(leftMotor.getActualPosition());
    Serial.print("Right Act.Pos.:   ");
    Serial.println(rightMotor.getActualPosition());

    Serial.println("------------------------------------------------");
    Serial.print("count: ");
    Serial.println(plan.count);
    Serial.print("head: ");
    Serial.println(plan.head);
    Serial.print("tail: ");
    Serial.println(plan.tail);

    Serial.println("------------------------------------------------");
    Serial.print("Sonar: ");
    Serial.print(sonar.ping_cm());
    Serial.println(" cm");
    */

    
    Serial.println("------------------------------------------------");
    Serial.print("Left RPM:   ");
    Serial.println(leftMotor.getActualRPM());
    Serial.print("Right RPM:   ");
    Serial.println(rightMotor.getActualRPM());
    
    
    serial_timeout = millis() + serial_interval;
  }
}


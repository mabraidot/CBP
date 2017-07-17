// Encoders
#define PIN_ENCODER1              2         // Left
#define PIN_ENCODER2              13        // Right
// Encoder resolution
#define ENCODER_HOLES             30 * 5    // Motor gear ratio = 1:5
#define ENCODER_WHEEL_CIRCLE      25        // Wheel circumference
#define ENCODER_RATIO             ENCODER_HOLES / ENCODER_WHEEL_CIRCLE // , wheel circumference = 25 cm
#define ENCODER_TURN_CM           12        // set xx cms and -xx cms on both wheels to turn 90 degrees
#define ENCODER_QUERY_INTERVAL    200       // Interval in milliseconds for RPM calculations

#define CURRENT_LIMIT             1

// Left DC Motor
#define PIN_LEFT_CURRENT_SENSE    A0
#define PIN_LEFT_DCMOTOR_DIR1     4
#define PIN_LEFT_DCMOTOR_DIR2     3
#define PIN_LEFT_DCMOTOR_PWM      5
#define LEFT_DCMOTOR_MAX_PWM      255

// Right DC Motor
#define PIN_RIGHT_CURRENT_SENSE   A1
#define PIN_RIGHT_DCMOTOR_DIR1    7
#define PIN_RIGHT_DCMOTOR_DIR2    8
#define PIN_RIGHT_DCMOTOR_PWM     6
#define RIGHT_DCMOTOR_MAX_PWM     252

// Planner - Ring Buffer Pattern
#define RING_BUFFER_SIZE          256

// Ultrasonic Sensor
#define PIN_SONAR_TRIGGER         11
#define PIN_SONAR_ECHO            12
#define SONAR_MAX_DISTANCE        40
#define SONAR_MIN_DISTANCE        20


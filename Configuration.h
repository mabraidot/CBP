// Encoders
#define PIN_ENCODER1              2   // Left
#define PIN_ENCODER2              13  // Right
// Encoder resolution
#define ENCODER_HOLES             14
#define ENCODER_RATIO             ENCODER_HOLES / 26
#define ENCODER_TURN_CM           15  // set xx cms and -xx cms on both wheels to turn 90 degrees
#define ENCODER_QUERY_INTERVAL    500 // milliseconds

// Left DC Motor
#define PIN_LEFT_DCMOTOR_DIR1     4
#define PIN_LEFT_DCMOTOR_DIR2     3
#define PIN_LEFT_DCMOTOR_PWM      5
#define LEFT_DCMOTOR_MAX_PWM      252

// Right DC Motor
#define PIN_RIGHT_DCMOTOR_DIR1    7
#define PIN_RIGHT_DCMOTOR_DIR2    8
#define PIN_RIGHT_DCMOTOR_PWM     6
#define RIGHT_DCMOTOR_MAX_PWM     255

// Planner - Ring Buffer Pattern
#define RING_BUFFER_SIZE          256

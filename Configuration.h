// Encoders
#define PIN_ENCODER1              2   // Left
#define PIN_ENCODER2              13  // Right
// Encoder resolution
#define ENCODER_HOLES             14
#define ENCODER_RATIO             ENCODER_HOLES / 26
#define ENCODER_QUERY_INTERVAL    500 // milliseconds

// Left DC Motor
#define PIN_LEFT_DCMOTOR_DIR1     4
#define PIN_LEFT_DCMOTOR_DIR2     5
#define PIN_LEFT_DCMOTOR_PWM      3

// Right DC Motor
#define PIN_RIGHT_DCMOTOR_DIR1    7
#define PIN_RIGHT_DCMOTOR_DIR2    8
#define PIN_RIGHT_DCMOTOR_PWM     6

// Planner - Ring Buffer Pattern
#define RING_BUFFER_SIZE          256

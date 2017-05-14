#include "Arduino.h"

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


#define  FORCE_INLINE __attribute__((always_inline)) inline
// The number of linear motions that can be in the plan at any give time.
#define BLOCK_BUFFER_SIZE 16
#define NUM_AXIS 2            // The axis order in all axis related arrays is X, Y
#define DROP_SEGMENTS 2       // 5; Everything with less than this number of steps will be ignored as move and joined with the next movement
// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)


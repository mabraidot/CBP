
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y;                    // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block
  
  // Fields used by the motion planner to manage acceleration
  // float speed_x, speed_y;                         // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  volatile char busy;
  
} block_t;

extern float mintravelfeedrate;
extern unsigned long minsegmenttime;
extern float axis_steps_per_unit[NUM_AXIS];
extern float max_feedrate[NUM_AXIS];                // set the max speeds
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];
extern float acceleration;                          // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves.
extern float max_xy_jerk;                           //speed than can be stopped at once

// Initialize the motion plan subsystem      
void plan_init();

// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index);
// Returns the index of the next block in the ring buffer
static int8_t next_block_index(int8_t block_index);

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next);
// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass();

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next);
// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass();

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration);

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance);

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor);

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids();

void planner_recalculate(void);

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
float max_allowable_speed(float acceleration, float target_velocity, float distance);

// Add a new linear movement to the buffer. x and y are the signed, absolute target position in millimiters. 
// Feed rate specifies the speed of the motion.
void plan_buffer_line(const float &x, const float &y, float feed_rate);



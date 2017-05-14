
/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. 

  Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
  
  s == speed, a == acceleration, t == time, d == distance
  Basic definitions:
    Speed[s_, a_, t_] := s + (a*t) 
    Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
  Distance to reach a specific speed with a constant acceleration:
    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
      d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
  Speed after a given distance of travel with constant acceleration:
    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
      m -> Sqrt[2 a d + s^2]    
    DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
  When to start braking (di) to reach a specified destionation speed (s2) after accelerating
  from initial speed s1 without ever stopping at a plateau:
    Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
      di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
    IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
    
*/
#include "Configuration.h"
#include "Planner.h"

//===========================================================================
//============================  public variables  ===========================
//===========================================================================
float acceleration;                     // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves.
float axis_steps_per_unit[NUM_AXIS];
float max_feedrate[NUM_AXIS];           // set the max speeds
unsigned long axis_steps_per_sqr_second[NUM_AXIS];
float mintravelfeedrate;
unsigned long minsegmenttime;
float max_xy_jerk;                      //speed than can be stopped at once

// The current position of the tool in absolute steps
long position[NUM_AXIS];                //rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_AXIS];  // Speed of previous path line segment
static float previous_nominal_speed;    // Nominal speed of previous path line segment


//===========================================================================
//=================  semi-private variables, used in inline  functions  =====
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now



void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  
  //memset(position, 0, sizeof(position)); // clear position
  for( int i = 0; i < sizeof(position);  ++i )
   position[i] = (char) 0;
   
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }
  
    if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {
    
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
          max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;
    
    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  if(((block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = { NULL, NULL, NULL };
    while(block_index != block_buffer_tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { return; }
  
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
        max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { NULL, NULL, NULL };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
  return((target_rate*target_rate-initial_rate*initial_rate)/
         (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}


// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
 if (acceleration!=0) {
  return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
         (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) {initial_rate=120; }
  if(final_rate < 120) {final_rate=120;  }
  
  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));
    
  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }
  
  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  // CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if(block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  // CRITICAL_SECTION_END;
}



// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
          next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
      MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}



// Returns the index of the next block in the ring buffer
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}








// Add a new linear movement to the buffer. steps_x, and _y are the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(const float &x, const float &y, float feed_rate)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head);

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps.
  long target[2];
  target[0] = lround(x*axis_steps_per_unit[0]);
  target[1] = lround(y*axis_steps_per_unit[1]);
  
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy
  block->busy = false;

  // Number of steps for each axis
  block->steps_x = labs(target[0] - position[0]);
  block->steps_y = labs(target[1] - position[1]);
  block->step_event_count = max(block->steps_x, block->steps_y);

  // Bail if this is a zero-length block
  if (block->step_event_count <= DROP_SEGMENTS) { return; };

  // Compute direction bits for this block 
  block->direction_bits = 0;
  if (target[0] < position[0]) { block->direction_bits |= (1 << 0); }
  if (target[1] < position[1]) { block->direction_bits |= (1 << 1); }

  if(feed_rate < mintravelfeedrate) feed_rate = mintravelfeedrate;

  float delta_mm[2];
  delta_mm[0] = (target[0]-position[0])/axis_steps_per_unit[0];
  delta_mm[1] = (target[1]-position[1])/axis_steps_per_unit[1];

  block->millimeters = sqrt(square(delta_mm[0]) + square(delta_mm[1]));
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 

  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;

  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
  
  #ifdef SLOWDOWN
  //  segment time im micro seconds
  unsigned long segment_time = lround(1000000.0/inverse_second);
  if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5))) {
    if (segment_time < minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
    }
  }
  #endif

  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[2];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for(int i=0; i < 2; i++) {
    current_speed[i] = delta_mm[i] * inverse_second;
    if(fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed  
  if( speed_factor < 1.0) {
    for(unsigned char i=0; i < 4; i++) {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }


  // Compute and limit the acceleration rate for the trapezoid generator.  
  float steps_per_mm = block->step_event_count/block->millimeters;

    block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  // Limit acceleration per axis
  if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[0])
    block->acceleration_st = axis_steps_per_sqr_second[0];
  if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[1])
    block->acceleration_st = axis_steps_per_sqr_second[1];
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);

  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2;  
  vmax_junction = min(vmax_junction, block->nominal_speed);

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[0]-previous_speed[0]), 2)+pow((current_speed[1]-previous_speed[1]), 2));
    if((fabs(previous_speed[0]) > 0.0001) || (fabs(previous_speed[1]) > 0.0001)) {
      vmax_junction = block->nominal_speed;
    }
    if (jerk > max_xy_jerk) {
      vmax_junction *= (max_xy_jerk/jerk);
    }
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { block->nominal_length_flag = true; }
  else { block->nominal_length_flag = false; }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed,
    MINIMUM_PLANNER_SPEED/block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;
  
  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();
  
}



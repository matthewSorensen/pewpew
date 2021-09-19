#ifndef motion_buffer_h
#define motion_buffer_h
#include <stdint.h>
#include "pin_maps.h"

typedef struct motion_segment_t {
  uint32_t move_id; // Whatever the sender tells us - just an opaque ID with no expected ordering.
  uint32_t move_flag;   // If this is non-zero, it's a special event
  // Both of these are in step counts per microsecond
  double start_velocity;
  double end_velocity;
  // These are all in raw step counts
  double coords[NUM_AXIS];
} motion_segment_t;

// Event segments are exactly the same size and layout as motion segments, but have
// some fields renamed for sanity.
typedef struct event_segment_t {
  uint32_t move_id;
  uint32_t move_flag;
  double args[NUM_AXIS + 2];
} event_segment_t;

// We can go up to 4x faster
#define MIN_OVERRIDE 0.25
// ...and 32x slower. Anything more, and it's interpreted as a halt.
#define MAX_OVERRIDE 32.0

typedef struct motion_state_t {
  // Where are we?
  int32_t position[NUM_AXIS];
  // Track the state of the motion ring buffer
  uint32_t current_move;
  uint32_t buffer_size;
  motion_segment_t* move;
  // State of the current move:
  uint32_t move_id;    // What's the current move id/number, directly taken from the move
  uint32_t move_flag;  // Move flags from the current move - if non-zero, it's actually a special event
  double velocity;     // What's the velocity at the end of the last step?
  double acceleration; // Acceleration over this segment?
  double end[NUM_AXIS];// What's the destination of this move?
 
  uint32_t step_bitmask; // What bits did we just set in the last step?
  int32_t step_update[NUM_AXIS];
  uint32_t dir_bitmask;  // What's the current state of the direction bits?
  uint32_t delay; // How long should we delay?

  // State of feed overrides:
  double override_current; // What's our current feed rate override?
  uint32_t override_changing; // Is it currently changing?
  double override_target;   // What are we changing it to?
  double override_velocity; // How fast is it changing per microsecond?
  
  
} motion_state_t;

// Must be a power of two
#define MOTION_BUFFER_SIZE 128 
#define MOTION_BUFFER_MASK (MOTION_BUFFER_SIZE - 1)

extern motion_segment_t motion_buffer[MOTION_BUFFER_SIZE];
extern volatile motion_state_t mstate;

void initialize_motion_state(void);
uint32_t free_buffer_spaces(void);
motion_segment_t* next_free_segment(void);
void start_motion(void);
void finish_motion(void);
void stepper_isr(void);
void set_override(double,double,uint32_t);
void finish_motion(uint32_t);

#endif

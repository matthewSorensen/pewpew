#ifndef motion_buffer_h
#define motion_buffer_h
#include <stdint.h>

#define INVERT 1
#define Y_AXIS 2
#define DWELL  4


typedef struct motion_segment_t {
  uint32_t steps;
  uint32_t direction;
  double start_velocity;
  double end_velocity; 
} motion_segment_t;

typedef struct motion_state_t {
  // Track the state of the motion ring buffer
  uint32_t current_move;
  uint32_t buffer_size;
  motion_segment_t* move;
  // State of the current move:
  uint32_t steps; // How many steps left in the segment?
  double velocity; // What's the velocity at the end of the last step?
  double acceleration; // Acceleration over this segment?
  uint32_t direction; // Direction field direct from the move?

  uint32_t step_bitmask; // What bits did we just set in the last step?
  uint32_t dir_bitmask;  // What's the current state of the direction bits?
  uint32_t delay; // How long should we delay?
} motion_state_t;

// Must be a power of two
#define MOTION_BUFFER_SIZE 128 
#define MOTION_BUFFER_MASK (MOTION_BUFFER_SIZE - 1)

extern motion_segment_t motion_buffer[MOTION_BUFFER_SIZE];
extern volatile motion_state_t mstate;

void initialize_motion_state(void);
uint32_t free_buffer_spaces(void);
uint32_t add_move_to_buffer(uint32_t steps, uint32_t direction, double start_velocity, double end_velocity);
void start_motion(void);
void stepper_isr(void);

#endif

#ifndef homing_h
#define homing_h

#include "pin_maps.h"
#include "protocol_constants.h"

typedef struct homing_state_t {
  homing_phase_t current_phase[NUM_AXIS]; 
  uint32_t unhomed_axes;
} homing_state_t;

extern volatile homing_state_t homing_state;

void homing_isr(void);


typedef struct homing_message_t {
  // A bitmask specifying which axes should be homed in this cycle - 2^0 is axis[0], 2^1 is axis[1] etc...
  uint32_t axes;
  // Either HOMING_APPROACH, or HOMING_BACKOFF - approach means we respect the direction and switch polarity
  // defined in homing_flags, backoff means we invert both.
  uint32_t direction;
  // Speed in steps / us - constant for all axes in a given cycle
  double speed;
} homing_message_t;

void start_homing(void* message);

#endif

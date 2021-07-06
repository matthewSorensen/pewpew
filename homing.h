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
void start_homing(void* message);

#endif

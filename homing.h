#ifndef homing_h
#define homing_h

#include "pin_maps.h"

// A fully general homing implementation this isn't... Maybe generalize this later?
typedef enum {
  HOMING_APPROACH,
  HOMING_BACKOFF,
  HOMING_DONE
} homing_phase_t;

typedef struct homing_state_t {
  homing_phase_t current_phase[NUM_AXIS]; 
  uint32_t unhomed_axes;
} homing_state_t;

extern volatile homing_state_t homing_state;


void homing_isr(void);
void start_homing(uint32_t axes, homing_phase_t direction, double speed);
uint32_t check_homing_status(void);


#endif

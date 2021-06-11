#ifndef dda_h
#define dda_h
#include "pin_maps.h"

typedef struct dda_state_h {

  double start[NUM_AXIS];
  double end[NUM_AXIS];
  double length;

  double increment_vector[NUM_AXIS];
  double increment_length;
  
  double length_acc;
  double error_acc[NUM_AXIS];
  
} dda_state_h;

extern dda_state_h dda;

uint32_t initialize_dda(volatile double* start, double* end);
uint32_t compute_step(double* length_dest);

#endif



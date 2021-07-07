#ifndef dda_h
#define dda_h
#include "pin_maps.h"

typedef struct dda_state_h {

  double increment_vector[NUM_AXIS];
  double error_acc[NUM_AXIS];

  uint32_t done;
  uint32_t step_count[NUM_AXIS]; // At least 10k/day, right?

  int32_t step_sign[NUM_AXIS];
  
  double qunit[NUM_AXIS]; // Quantized unit vector - unit vector in step space
  double qlength; // Lenght of the move in step space
  double prev_length;
  
} dda_state_h;

extern dda_state_h dda;

uint32_t initialize_dda(volatile double* start, double* end);
uint32_t compute_step(double* length_dest, volatile int32_t* step_count_dest);

#endif



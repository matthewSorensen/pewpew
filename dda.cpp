#include <stdint.h>
#include <math.h>
#include "dda.h"


#define OVERSAMPLE 4

dda_state_h dda;


uint32_t initialize_dda(volatile double* start, double* end){

  double qlength = 0;
  double max = 0;
  uint32_t dir_mask = 0;
  int i;

  dda.done = 0;

  for(i = 0; i < NUM_AXIS; i++){
    double s = start[i], qs = round(s), d = end[i] - s;
    int32_t target = (int32_t) round(end[i]) - qs;
    
    dda.step_count[i] = target < 0 ? 0 - target : target;
    
    if(d < 0){
      d = 0 - d;
      dir_mask |= axis_to_dir[i];
      dda.error_acc[i] = -1 * (s - qs);
    } else {
      dda.error_acc[i] = s - qs;
    }

    dda.increment_vector[i] = d;
    
    if(d > max) max = d;
  }

  max = 1 / max;
  // Rescale the increment vector so that the largest components are 1/(oversampling factor),
  // and compute the total length of the quantized move.
  for(i = 0; i < NUM_AXIS; i++){
    double qcomponent = dda.step_count[i];
    qlength += qcomponent * qcomponent;
    dda.increment_vector[i] *= max / OVERSAMPLE;
  }

  dda.qlength = qlength = sqrt(qlength);
  dda.prev_length = qlength;
  
  qlength = 1 / qlength;
  for(i = 0; i < NUM_AXIS; i++){
    dda.qunit[i] = (double) dda.step_count[i] * qlength;
  }
  
  
  return dir_mask;
}


uint32_t compute_step(double* length_dest){
  uint32_t steps = 0;
  double length = 0;
  uint32_t i = 0;
  if(dda.done)
    return 0;

  // Compute the main step - keep adding copies of the increment vector
  // to the error vector, and if it gets large enough, wrap it and record the step.
  while(1){
    uint32_t done = 1;
    for(i = 0; i<NUM_AXIS; i++){
      double error = dda.error_acc[i] + dda.increment_vector[i];
      uint32_t count = dda.step_count[i];    
    
      if(error > 0.5){
	error -= 1.0;
	if(count > 0){
	  steps |= axis_to_step[i];
	  count -= 1;
	  dda.step_count[i] = count;
	}
      }

      dda.error_acc[i] = error;
      
      if(count > 0)
	done = 0;   
    }
    
    if(steps || done)
      break;
  }
  
  if(steps == 0){
    dda.done = 1;
  }

  // If any steps are about to happen and would result in a super short interval between
  // pulses, take them
  for(i = 0; i<NUM_AXIS; i++){
    double error = dda.error_acc[i] + dda.increment_vector[i];
    uint32_t count = dda.step_count[i];    
    
    if(error > 0.5 - 1.0 / OVERSAMPLE){
      error -= 1.0;
      if(count > 0){
	steps |= axis_to_step[i];
	  count -= 1;
	  dda.step_count[i] = count;
      }
    }
    dda.error_acc[i] = error;
  }
  // Compute the new length along the move
  for(i = 0; i < NUM_AXIS; i++){
    length += dda.qunit[i] * dda.step_count[i];
  }
  
  *length_dest = dda.prev_length - length;
  dda.prev_length = length;
  return steps;
}


#include <stdint.h>
#include <math.h>
#include "dda.h"


dda_state_h dda;


uint32_t initialize_dda(volatile double* start, double* end){

  double length = 0;
  double max = 0;
  uint32_t dir_mask = 0;
  
  for(int i = 0; i < NUM_AXIS; i++){
    double d = end[i] - start[i];
    dda.start[i] = start[i];
    dda.end[i] = end[i];
    length += d * d;
    if(d < 0){
      d = 0 - d;
      dir_mask |= axis_to_dir[i];      
    }
    if(d > max)
      max = d;
    dda.increment_vector[i] = d;
  }

  dda.length = length = sqrt(length);
  max = 1 / max;
  for(int i = 0; i < NUM_AXIS; i++){
    dda.increment_vector[i] *= max;
    dda.error_acc[i] = 0;
  }
  dda.increment_length = length * max;
  dda.length_acc = 0;
  
  return dir_mask;
}


uint32_t compute_step(double* length_dest){
  uint32_t steps = 0;
  double length = dda.length;
  double length_acc = dda.length_acc;
  double new_length = length_acc + dda.increment_length;
  double scale = 1.0;
  // Weird error state - dont' generate any steps
  if(length <= length_acc) return 0;
  // Otherwise, we might have just a little bit left in the segment,
  // and have to take a partial step
  if(new_length > length){
    double delta = length - length_acc;
    *length_dest = delta;
    scale = delta / dda.increment_length;
  }else{
    *length_dest = dda.increment_length;
  }

  for(int i = 0; i<NUM_AXIS; i++){
    double error = dda.error_acc[i] + scale * dda.increment_vector[i];
    if(error > 0.5){
      steps |= axis_to_step[i];
      error -= 1.0;
    }
    dda.error_acc[i] = error;
  }
  
  dda.length_acc = new_length;
  return steps;
}


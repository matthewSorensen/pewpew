#include "pin_maps.h"
#include "core_pins.h"


const uint32_t axis_to_step[NUM_AXIS] = {PIN_BITMASK(X_STEP), PIN_BITMASK(Y_STEP)};
const uint32_t axis_to_dir[NUM_AXIS]  = {PIN_BITMASK(X_DIR), PIN_BITMASK(Y_DIR)};
const uint32_t axis_to_limit[NUM_AXIS]  = {PIN_BITMASK(X_LIMIT), PIN_BITMASK(Y_LIMIT)};
const uint32_t homing_flags[NUM_AXIS] = {REVERSE_HOME,REVERSE_HOME};
const uint32_t home_positions[NUM_AXIS] = {0,0};


void initialize_gpio(void){

  pinMode(X_STEP, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(LASER, OUTPUT);

  pinMode(X_LIMIT, INPUT_PULLDOWN);
  pinMode(Y_LIMIT, INPUT_PULLDOWN);
 
}

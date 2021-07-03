#include "pin_maps.h"
#include "core_pins.h"



const uint32_t axis_to_step[NUM_AXIS] = {PIN_BITMASK(X_STEP), PIN_BITMASK(Y_STEP)};
const uint32_t axis_to_dir[NUM_AXIS]  = {PIN_BITMASK(X_DIR), PIN_BITMASK(Y_DIR)};
const uint32_t axis_to_limit[NUM_AXIS]  = {PIN_BITMASK(X_LIMIT), PIN_BITMASK(Y_LIMIT)};
const double steps_per_mm[NUM_AXIS]   = {2 / 0.00125, 2 / 0.00125};
const uint32_t homing_flags[NUM_AXIS] = {REVERSE_HOME,REVERSE_HOME};


void initialize_gpio(void){

  pinMode(X_STEP, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(LASER, OUTPUT);

  pinMode(X_LIMIT, INPUT_PULLDOWN);
  pinMode(Y_LIMIT, INPUT_PULLDOWN);
 
}

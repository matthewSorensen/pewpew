#include "pin_maps.h"
#include "core_pins.h"



const uint32_t axis_to_step[NUM_AXIS] = {PIN_BITMASK(X_STEP), PIN_BITMASK(Y_STEP)};
const uint32_t axis_to_dir[NUM_AXIS]  = {PIN_BITMASK(X_DIR), PIN_BITMASK(Y_DIR)};
const double steps_per_mm[NUM_AXIS]   = {2 / 0.00125, 2 / 0.00125};

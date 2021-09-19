#include "core_pins.h"
#include "pin_maps.h"


const motor_pins_t motor_pins[NUM_AXIS] = {MOTOR_PINS(23, 22),
					   MOTOR_PINS(19, 18)};


const homing_pins_t home_pins[NUM_AXIS] = {HOMING_PIN(0,0,HOME_REVERSE),
					   HOMING_PIN(1,0,HOME_REVERSE)};


void initialize_gpio(void){

  for(int i = 0; i < NUM_AXIS; i++){
    pinMode(motor_pins[i].step_pin_number, OUTPUT);
    
    if(motor_pins[i].dir_pin_bitmask)
      pinMode(motor_pins[i].dir_pin_number, OUTPUT);
    
    if(!(home_pins[i].flags & HOME_NONE)){
      pinMode(home_pins[i].limit_pin_number, INPUT_PULLDOWN);
    }
  }
 
}

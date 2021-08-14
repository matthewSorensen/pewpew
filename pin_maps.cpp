#include "core_pins.h"
#include "pin_maps.h"


const motor_pins_t motor_pins[NUM_AXIS] = {MOTOR_PINS(19, 20),
					   MOTOR_PINS(17, 18)};


const homing_pins_t home_pins[NUM_AXIS] = {
  {.limit_pin_number = 0, .limit_pin_bitmask = PIN_BITMASK(0), .home_position = 0 , .flags = REVERSE_HOME},
  {.limit_pin_number = 1, .limit_pin_bitmask = PIN_BITMASK(1), .home_position = 0 , .flags = REVERSE_HOME}};



void initialize_gpio(void){

  for(int i = 0; i < NUM_AXIS; i++){
    pinMode(motor_pins[i].step_pin_number, OUTPUT);
    pinMode(motor_pins[i].dir_pin_number, OUTPUT);
    pinMode(home_pins[i].limit_pin_number, INPUT_PULLDOWN);
  }
 
}

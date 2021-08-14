#ifndef pin_maps_h
#define pin_maps_h

#include <stdint.h>

#define NUM_AXIS 2

// These definitions aren't entirely orthagonal from the
// pin defs in pin_maps.h, alas...
#define STEP_SET   GPIO6_DR_SET
#define STEP_CLEAR GPIO6_DR_CLEAR
#define DIR_REG GPIO6_DR
#define LIMIT_REG CORE_PIN0_PINREG
// Same here - changing NUM_AXIS requires updating this too...
#define DIR_BITMASK (motor_pins[0].dir_pin_bitmask | motor_pins[1].dir_pin_bitmask)
#define STEP_BITMASK  (motor_pins[0].step_pin_bitmask | motor_pins[1].step_pin_bitmask)

#define PIN_BITMASK_(pin) (CORE_PIN##pin##_BITMASK)
#define PIN_BITMASK(pin) PIN_BITMASK_(pin)


typedef struct motor_pins_t {
  uint32_t step_pin_number;
  uint32_t step_pin_bitmask;
  
  uint32_t dir_pin_number;
  uint32_t dir_pin_bitmask;
} motor_pins_t;


#define MOTOR_PINS(step, dir) {.step_pin_number = step, .step_pin_bitmask = PIN_BITMASK(step), .dir_pin_number = dir, .dir_pin_bitmask = PIN_BITMASK(dir)}

extern const motor_pins_t motor_pins[NUM_AXIS];


#define INVERT_HOME 1
#define REVERSE_HOME 2

typedef struct homing_pins_t {
  uint32_t limit_pin_number;
  uint32_t limit_pin_bitmask;
  
  uint32_t home_position;
  uint32_t flags;
} homing_pins_t;


extern const homing_pins_t home_pins[NUM_AXIS];


void initialize_gpio(void);

#endif

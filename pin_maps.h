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
#define STEP_ONLY_MOTOR(step) {.step_pin_number = step, .step_pin_bitmask = PIN_BITMASK(step), .dir_pin_number = 0, .dir_pin_bitmask = 0}

extern const motor_pins_t motor_pins[NUM_AXIS];

typedef enum homing_flag_t {
  HOME_NONE = 1,
  HOME_INVERT = 2,
  HOME_REVERSE = 4
} homing_flag_t;

typedef struct homing_pins_t {
  uint32_t limit_pin_number;
  uint32_t limit_pin_bitmask;
  
  uint32_t home_position;
  homing_flag_t flags;
} homing_pins_t;

#define NO_HOME {.limit_pin_number = 0, .limit_pin_bitmask = 0, .home_position = 0, .flags = HOME_NONE}
#define HOMING_PIN(pin,home_pos,flag) {.limit_pin_number = pin, .limit_pin_bitmask = PIN_BITMASK(pin), .home_position = home_pos, .flags = flag}

extern const homing_pins_t home_pins[NUM_AXIS];


void initialize_gpio(void);

#endif

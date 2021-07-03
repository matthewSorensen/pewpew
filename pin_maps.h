#ifndef pin_maps_h
#define pin_maps_h

#include <stdint.h>

#define NUM_AXIS 2

#define X_STEP 19
#define X_DIR  20
#define Y_STEP 17
#define Y_DIR  18

#define LASER 13

#define X_LIMIT 0
#define Y_LIMIT 1

#define STEP_SET   PIN_PORTSET(X_STEP)
#define STEP_CLEAR PIN_PORTCLEAR(X_STEP)

#define DIR_REG PIN_PORTREG(X_DIR)

#define LIMIT_REG CORE_PIN0_PINREG

#define DIR_BITMASK (PIN_BITMASK(X_DIR) | PIN_BITMASK(Y_DIR))
#define STEP_BITMASK (PIN_BITMASK(X_STEP) | PIN_BITMASK(Y_STEP))

#define PIN_BITMASK_(pin) (CORE_PIN##pin##_BITMASK)
#define PIN_BITMASK(pin) PIN_BITMASK_(pin)

#define PIN_PORTCLEAR_(pin) (CORE_PIN##pin##_PORTCLEAR)
#define PIN_PORTCLEAR(pin)  PIN_PORTCLEAR_(pin)

#define PIN_PORTSET_(pin) (CORE_PIN##pin##_PORTSET)
#define PIN_PORTSET(pin)  PIN_PORTSET_(pin)

#define PIN_PORTREG_(pin) (CORE_PIN##pin##_PORTREG)
#define PIN_PORTREG(pin)  PIN_PORTREG_(pin)

#define CLEAR_PIN_(pin) {CORE_PIN##pin##_PORTCLEAR = CORE_PIN##pin##_BITMASK;}
#define CLEAR_PIN(pin)  CLEAR_PIN_(pin)

#define SET_PIN_(pin) {CORE_PIN##pin##_PORTSET = CORE_PIN##pin##_BITMASK;}
#define SET_PIN(pin)  SET_PIN_(pin)

#define READ_PIN_(pin) (!!(CORE_PIN##pin##_PINREG & CORE_PIN##pin##_BITMASK))
#define READ_PIN(pin)  READ_PIN_(pin)



extern const uint32_t axis_to_step[NUM_AXIS];
extern const uint32_t axis_to_dir[NUM_AXIS];
extern const uint32_t axis_to_limit[NUM_AXIS];

extern const double steps_per_mm[NUM_AXIS];

#define INVERT_HOME 1
#define REVERSE_HOME 2

extern const uint32_t homing_flags[NUM_AXIS];


void initialize_gpio(void);

#endif

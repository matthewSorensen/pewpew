// Auto-generated file containing enum definitions shared with python client. Do not edit directly!
#ifndef protocol_constants_h 
#define protocol_constants_h

#include <stdint.h>
#include "pin_maps.h"

#define MAX_MESSAGE 10

typedef enum message_type_t {
    MESSAGE_INQUIRE = 1,
    MESSAGE_DESCRIBE = 2,
    MESSAGE_ASK = 3,
    MESSAGE_STATUS = 4,
    MESSAGE_BUFFER = 5,
    MESSAGE_EXPECT = 6,
    MESSAGE_SEGMENT = 7,
    MESSAGE_HOME = 8,
    MESSAGE_START = 9,
    MESSAGE_ERROR = 10
} message_type_t;

typedef enum homing_phase_t {
    HOMING_APPROACH = 1,
    HOMING_BACKOFF = 2,
    HOMING_DONE = 3
} homing_phase_t;

typedef enum status_flag_t {
    STATUS_IDLE = 1,
    STATUS_BUSY = 2,
    STATUS_HOMING = 3,
    STATUS_DEAD = 4
} status_flag_t;

extern const uint32_t message_sizes[10];
extern uint8_t message_buffer[24 + 8 * NUM_AXIS];
#endif

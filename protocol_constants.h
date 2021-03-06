// Auto-generated file containing enum definitions shared with python client. Do not edit directly!
// Regenerate by running host/pewpew/codegen.py from the project home directory.
#ifndef protocol_constants_h 
#define protocol_constants_h

#include <stdint.h>
#include "pin_maps.h"

#define MAX_MESSAGE 15

typedef enum message_type_t {
    MESSAGE_INQUIRE = 1,
    MESSAGE_DESCRIBE = 2,
    MESSAGE_ASK = 3,
    MESSAGE_STATUS = 4,
    MESSAGE_BUFFER = 5,
    MESSAGE_DONE = 6,
    MESSAGE_SEGMENT = 7,
    MESSAGE_SPECIAL = 8,
    MESSAGE_IMMEDIATE = 9,
    MESSAGE_HOME = 10,
    MESSAGE_START = 11,
    MESSAGE_OVERRIDE = 12,
    MESSAGE_ERROR = 13,
    MESSAGE_QUIZ = 14,
    MESSAGE_PERIPHERAL = 15
} message_type_t;

typedef enum homing_phase_t {
    HOMING_APPROACH = 1,
    HOMING_BACKOFF = 2,
    HOMING_DONE = 3
} homing_phase_t;

typedef enum status_flag_t {
    STATUS_IDLE = 1,
    STATUS_BUSY = 2,
    STATUS_HALT = 3,
    STATUS_HOMING = 4,
    STATUS_DEAD = 5,
    STATUS_BUFFER_UNDERFLOW = 6
} status_flag_t;

typedef union {char field0[8*NUM_AXIS+24]; char field1[8*SPECIAL_EVENT_SIZE+8]; char field2[PERIPHERAL_STATUS];} message_buffer_size;

#define MESSAGE_BUFFER_SIZE sizeof(message_buffer_size)

extern const uint32_t message_sizes[15];
extern uint8_t message_buffer[MESSAGE_BUFFER_SIZE];
#endif


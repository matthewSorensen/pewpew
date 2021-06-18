#ifndef special_events_h
#define special_events_h
#include <stdint.h>
#include "pin_maps.h"

typedef enum event_type_t {
  DWELL = 1,
  FIRE_LASER = 2
} event_type_t;

// This must be something that can type pun with motion segments - as long as it is smaller, and
// and the first 8 bytes look like a NaN when cast to a double, it'll work.
typedef struct special_segment_t {
  uint32_t event_flags; // What sort of event is this? Takes values in event_type_t
  uint16_t padding;   // (manually pad this to keep the rest aligned - free for use)
  uint16_t tag_bytes; // Always 0xFFFF! If it isn't 0xFFFF bad things happen!
  double  arguments[NUM_AXIS + 1]; // Some free space for parameters! That's a lot of bytes!
} special_segment_t;

// When the stepper ISR finds an event in the queue, it initially calls execute_event to
// begin the event. As events may not block, execute_event returns a delay length in
// microseconds. If this is non-zero, the ISR will set a timer for that delay, and then call
// execute_events again when the timer fires - the process repeats until execute_event returns
// a zero-length delay.
uint32_t execute_event(special_segment_t*);

#endif

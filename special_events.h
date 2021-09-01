#ifndef special_events_h
#define special_events_h
#include <stdint.h>
#include "pin_maps.h"

#include "motion_buffer.h"
// These must be non-zero, as 0 signals a motion segment
typedef enum event_type_t {
  NOTHING = 1
} event_type_t;


// When the stepper ISR finds an event in the queue, it initially calls execute_event to
// begin the event. As events may not block, execute_event returns a delay length in
// microseconds. If this is non-zero, the ISR will set a timer for that delay, and then call
// execute_events again when the timer fires - the process repeats until execute_event returns
// a zero-length delay.
uint32_t execute_event(event_segment_t*);

#endif

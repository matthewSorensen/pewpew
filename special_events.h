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

// Immediate events must not return a delay, or access any event state - the delay will be
// ignored, and the immediate event may interfere with a sleeping event.
uint32_t execute_event(event_segment_t* event,uint32_t is_immediate,uint32_t is_initial);


#endif

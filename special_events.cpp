#include <stdint.h>
#include "core_pins.h"

#include "pin_maps.h"
#include "special_events.h"

uint32_t execute_event(special_segment_t* event){

  if(event->event_flags & FIRE_LASER){
    SET_PIN(LASER);
  }else{
    CLEAR_PIN(LASER);
  }
  
  
  return 0;
}

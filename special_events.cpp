#include <stdint.h>
#include "core_pins.h"

#include "pin_maps.h"
#include "special_events.h"

uint32_t execute_event(motion_segment_t* event){

  if(event->move_flag & FIRE_LASER){
    SET_PIN(LASER);
  }else{
    CLEAR_PIN(LASER);
  }
  
  
  return 0;
}

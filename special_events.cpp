#include <stdint.h>
#include "core_pins.h"

#include "pin_maps.h"
#include "special_events.h"

int32_t led_enabled = 0;
int32_t led_on = 0;

int32_t execute_event(event_segment_t* event, uint32_t is_immediate, uint32_t is_initial){
  if(!led_enabled){
    pinMode(13,OUTPUT);
    led_enabled = 1;
  }
  if(led_on){
    digitalWrite(13,LOW);
    led_on = 0;
  }else{
    digitalWrite(13,HIGH);
    led_on = 1;
  }
  return 0;
}


void shutdown_events(){
  if(led_enabled && led_on){
      digitalWrite(13, LOW);
      led_on = 0;
      led_enabled = 0;
  }
}

void build_peripheral_status(uint8_t*buff){
  buff[0] = led_enabled + 2 * led_on;
}

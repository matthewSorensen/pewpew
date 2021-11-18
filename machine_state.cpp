#include "machine_state.h"
#include "motion_buffer.h"
#include "special_events.h"
#include <Arduino.h>

volatile comm_state_t cs;
volatile status_message_t sm;

void set_status(status_flag_t status){
  cs.status = status;
}


void send_status_message(uint32_t request_id){
  
  sm.request_id = request_id;
  sm.flag = cs.status;
  sm.buffer_slots = free_buffer_spaces();
  sm.flag = cs.status;
  
  if(cs.status == STATUS_BUSY || cs.status == STATUS_HALT){
    sm.move_id = mstate.move_id;
  }else{
    sm.move_id = 0;
  }

  sm.override = fstate.current;
  
  for(int i = 0; i < NUM_AXIS; i++){
    sm.pos[i] = mstate.position[i];
  }
  
  Serial.write((char) MESSAGE_STATUS);
  Serial.write((uint8_t*) &sm, message_sizes[MESSAGE_STATUS - 1]);
  Serial.send_now();

  cs.last_status_time = millis();
  
}


void check_status_interval(void){
  if(!cs.have_handshook)
    return;
  
  uint32_t now = millis();
  uint32_t then = cs.last_status_time;
  uint32_t elapsed = 0;

  if(now < then){
    elapsed = now + (0xFFFFFFFF - then);
  }else{
    elapsed = now - then;
  }

  if(elapsed > 250)
    send_status_message(0);

}

void error_and_die(const char* message){
  shutdown_motion();
  shutdown_events();
  Serial.write(MESSAGE_ERROR);
  Serial.write(message);
  Serial.write('\n');
  while(1) delay(1000);
}

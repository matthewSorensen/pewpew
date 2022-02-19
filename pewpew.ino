#include "core_pins.h"

#include "protocol_constants.h"
#include "machine_state.h"
#include "motion_buffer.h"
#include "pin_maps.h"
#include "dda.h"
#include "special_events.h"
#include "homing.h"

void send_message(message_type_t message, uint8_t* body){
  Serial.write((char) message);
  Serial.write(body, message_sizes[message - 1]);
  Serial.send_now();
}

void handle_message(message_type_t mess){

  switch(mess){

  case MESSAGE_INQUIRE:{
    uint32_t* params = (uint32_t*) message_buffer;
    params[0] = 2; // Protocol version - v2 supports peripheral status updates
    params[1] = NUM_AXIS; // The all-important number of axes
    params[2] = 1337; // Device number? IDK. I like inventing random undescribed fields in new protocols.
    params[3] = MOTION_BUFFER_SIZE; // Also important for the sender to know, but not critical.
    params[4] = PERIPHERAL_STATUS; // Peripheral status message byte count
    send_message(MESSAGE_DESCRIBE, message_buffer);
    cs.have_handshook = 1;
  } break;

  case MESSAGE_ASK:{
    send_status_message(*(uint32_t*) message_buffer);
    break;
  }
    
  case MESSAGE_BUFFER:{
    uint32_t* message = (uint32_t*) message_buffer;
    cs.expect_request_id = message[0];
    cs.suppress_buffer_count += message[1];
  }break;

  case MESSAGE_DONE:
    cs.buffer_done = 1;
    break;

    
  case MESSAGE_SEGMENT:{
    segment_t* dest = next_free_segment();
    if(!dest){
      error_and_die("Motion buffer overflow");
    }
    memcpy(dest, message_buffer, sizeof(segment_t));
    mstate.buffer_size++;
    
    if(cs.suppress_buffer_count <= 1){
      uint32_t* message = (uint32_t*) message_buffer;
      message[0] = cs.expect_request_id;
      message[1] = free_buffer_spaces();
      send_message(MESSAGE_BUFFER, message_buffer);
      cs.expect_request_id = 0;
    }
    if(cs.suppress_buffer_count > 0)
      cs.suppress_buffer_count--;
    cs.buffer_done = 0;
  } break;

  case MESSAGE_IMMEDIATE: {
    event_segment_t* event = &(((segment_t*) message_buffer)->event);
    
    if(!event->move_flag)
      error_and_die("Immediate events must not be motion segments\n");
    
    execute_event(event,1,1);
  } break;

  case MESSAGE_HOME:
    if(!(cs.status == STATUS_IDLE || cs.status == STATUS_HALT))
      error_and_die("Homing cycle must start from idle state");
    start_homing(message_buffer);
    break;
  case MESSAGE_START:
    // Start is idempotent
    if(!(cs.status == STATUS_IDLE || cs.status == STATUS_BUSY || cs.status == STATUS_HALT))
      error_and_die("Cycle must start from idle state");
    start_motion();
    break;

  case MESSAGE_OVERRIDE:{
    double* message = (double*) message_buffer;
    set_override(message[0], message[1], cs.status == STATUS_BUSY);
    break;
  };

  case MESSAGE_QUIZ: {
    build_peripheral_status(message_buffer);
    send_message(MESSAGE_PERIPHERAL, message_buffer);
    break;
  };
    
  case MESSAGE_DESCRIBE:
  case MESSAGE_STATUS:
  case MESSAGE_ERROR:
  default:
    error_and_die("Received message in wrong direction\n");
  }
}


int main(void){
  // Message "parser" state
  uint32_t message_started = 0;
  uint32_t message_type = 0;
  uint32_t remaining_chars = 0;
  uint8_t* dest = message_buffer;

  initialize_gpio();
  
  // Initialize the communication state
  cs.serial_active = 0;
  cs.have_handshook = 0;
  cs.suppress_buffer_count = 0;
  cs.status = STATUS_IDLE;
  cs.expect_request_id = 0;
  cs.buffer_done = 1;
   
  while(1){
    // Track the falling and rising edges of the serial connection    
    if(!Serial){
      if(cs.serial_active){
	shutdown_motion();
	shutdown_events();
	cs.serial_active = 0;
      }
      delay(100);
      continue;
    }else if(!cs.serial_active){
      // Serial connection just turned on!
      cs.serial_active = 1;
      cs.have_handshook = 0;
      cs.suppress_buffer_count = 0;
      cs.expect_request_id = 0;
      cs.buffer_done = 1;
      cs.status = STATUS_IDLE;
      cs.last_status_time = 0;
      
      initialize_motion_state();
    }
    // Check for serial input
    if(Serial.available()){
      uint8_t byte = Serial.read(); 

      if(message_started){
	*dest++ = byte;
	remaining_chars -= 1;
      }else{
	// Check that the byte is a valid message type, of course
	if(byte < 1 || byte > MAX_MESSAGE)
	  error_and_die("Invalid message header");
	
	message_started = 1;
	message_type = byte;
	remaining_chars = message_sizes[message_type - 1];
	dest = message_buffer;
      }
      
      if(remaining_chars == 0){
	handle_message((message_type_t) message_type);
	message_started = 0;
      }
    }else{
      check_status_interval();
    }
  }
}

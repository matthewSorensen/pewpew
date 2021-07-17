#include "core_pins.h"
#include "protocol_constants.h"
#include "motion_buffer.h"
#include "pin_maps.h"
#include "dda.h"
#include "special_events.h"
#include "homing.h"

typedef struct comm_state_t {
  // Is the serial port currently active?
  uint32_t serial_active;
  // Have we shook hands with the device, to the point that we can sent messages beyond DESCRIBE?
  uint32_t have_handshook;
  // How many more segments/events should we wait for before we send a buffer count size?
  uint32_t suppress_buffer_count;
  // What are we currently doing?
  status_flag_t status;

  uint32_t expect_request_id;
  
} comm_state_t;

typedef struct status_message_t {
  uint32_t request_id; 
  uint32_t flag;
  uint32_t buffer_slots;
  uint32_t move_id;
  int32_t pos[NUM_AXIS];
} status_message_t;

static comm_state_t cs;

void build_status_message(status_message_t* sm){
  // Yeah, we don't overwrite the request id, because it's the same as the incoming
  // ask request
  sm->flag = cs.status;
  sm->buffer_slots = free_buffer_spaces();

  if(cs.status == STATUS_BUSY){
    sm->move_id = mstate.move_id;
  }else{
    sm->move_id = 0;
  }
  
  for(int i = 0; i < NUM_AXIS; i++){
    sm->pos[i] = mstate.position[i];
  }
  
}

void error_and_die(const char* message){
  Serial.write(MESSAGE_ERROR);
  Serial.write(message);
  Serial.write('\n');
  while(1) delay(1000);
}

void send_message(message_type_t message, uint8_t* body){
  Serial.write((char) message);
  Serial.write(body, message_sizes[message - 1]);
  Serial.send_now();
}

void handle_message(message_type_t mess){

  switch(mess){

  case MESSAGE_INQUIRE:{
    uint32_t* params = (uint32_t*) message_buffer;
    params[0] = 1; // Protocol version
    params[1] = NUM_AXIS; // The all-important number of axes
    params[2] = 1337; // Device number? IDK. I like inventing random undescribed fields in new protocols.
    params[3] = MOTION_BUFFER_SIZE; // Also important for the sender to know, but not critical.
    send_message(MESSAGE_DESCRIBE, message_buffer);
    cs.have_handshook = 1;
  } break;

  case MESSAGE_ASK:
    build_status_message((status_message_t*) message_buffer);
    send_message(MESSAGE_STATUS, message_buffer);
    break;

  case MESSAGE_EXPECT:{
    uint32_t* message = (uint32_t*) message_buffer;
    cs.expect_request_id = message[0];
    cs.suppress_buffer_count += message[1];
  }break;

  case MESSAGE_SEGMENT:{
    motion_segment_t* dest = next_free_segment();
    if(!dest){
      error_and_die("Motion buffer overflow");
    }
    memcpy(dest, message_buffer, sizeof(motion_segment_t));
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
  } break;
  case MESSAGE_HOME:
    if(cs.status != STATUS_IDLE)
      error_and_die("Homing cycle must start from idle state");
    start_homing(message_buffer);
    cs.status = STATUS_HOMING;
    break;
  case MESSAGE_START:
    // Start is idempotent
    if(cs.status != STATUS_IDLE || cs.status == STATUS_BUSY)
      error_and_die("Cycle must start from idle state");
    cs.status = STATUS_BUSY;
    start_motion();
    break;
  case MESSAGE_DESCRIBE:
  case MESSAGE_STATUS:
  case MESSAGE_BUFFER:
  case MESSAGE_ERROR:
  default:
    error_and_die("Recived message in wrong direction\n");
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
   
  while(1){
    // Track the falling and rising edges of the serial connection    
    if(!Serial){
      if(cs.serial_active){
	// Probably should stop motion here too
	cs.serial_active = 0;
      }
      delay(100);
      continue;
    }else if(!cs.serial_active){
      // Serial connection just turned on!
      cs.serial_active = 1;
      cs.have_handshook = 0;
      cs.suppress_buffer_count = 0;
      cs.status = STATUS_IDLE;
      cs.expect_request_id = 0;
      initialize_motion_state();
    }
    // Update the current status
    switch(cs.status){
    case STATUS_HOMING:
      if(homing_state.unhomed_axes == 0)
	cs.status = STATUS_IDLE;
      break;
    case STATUS_BUSY:
      if(mstate.move == NULL)
	cs.status = STATUS_IDLE;
      break;
    case STATUS_IDLE:
    default:
      break;
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
    }
  }
}

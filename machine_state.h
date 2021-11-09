#ifndef machine_state_h
#define machine_state_h


#include <stdint.h>
#include "protocol_constants.h"

typedef struct comm_state_t {
  // Is the serial port currently active?
  uint32_t serial_active;
  // Have we shook hands with the device, to the point that we can sent messages beyond DESCRIBE?
  uint32_t have_handshook;
  // How many more segments/events should we wait for before we send a buffer count size?
  uint32_t suppress_buffer_count;
  // What are we currently doing?
  status_flag_t status;

  // Have we gotten a 'done' message, indicating that finishing the
  // buffer isn't an underflow state? Resets every time a move is put into the
  // motion buffer
  uint32_t buffer_done;
  
  uint32_t expect_request_id;

  uint32_t last_status_time;
  
} comm_state_t;

typedef struct status_message_t {
  uint32_t request_id; 
  uint32_t flag;
  uint32_t buffer_slots;
  uint32_t move_id;
  double override;
  int32_t pos[NUM_AXIS];
} status_message_t;

extern volatile comm_state_t cs;
extern volatile status_message_t sm;

// Compile the current machine status and send it to the host
void send_status_message(uint32_t request_id);
// Set the current status flag
void set_status(status_flag_t);

void check_status_interval(void);
#endif



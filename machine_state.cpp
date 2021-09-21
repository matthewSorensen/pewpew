#include "machine_state.h"

volatile comm_state_t cs;


void set_status(status_flag_t status){
  cs.status = status;
}

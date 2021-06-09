#include "core_pins.h"
#include "motion_buffer.h"
#include "pin_maps.h"


int main(void){

  Serial.begin(9600);
  
  pinMode(X_STEP, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_DIR, OUTPUT);

  initialize_motion_state();

  double t = 0.003;
  uint32_t d = 1000;

  while(1){

    
    add_move_to_buffer(50, 0, 0.0, t);
    add_move_to_buffer(d, 0, t, t);
    add_move_to_buffer(50, 0, t,0.0);

    
    add_move_to_buffer(50, Y_AXIS, 0.0, t);
    add_move_to_buffer(d,  Y_AXIS, t, t);
    add_move_to_buffer(50, Y_AXIS, t,0.0);
    
    add_move_to_buffer(1000, DWELL, t,0.0);
    
    add_move_to_buffer(50, INVERT, 0.0, t);
    add_move_to_buffer(d, INVERT, t, t);
    add_move_to_buffer(50, INVERT, t,0.0);

    
    add_move_to_buffer(50, Y_AXIS | INVERT, 0.0, t);
    add_move_to_buffer(d,  Y_AXIS | INVERT, t, t);
    add_move_to_buffer(50, Y_AXIS | INVERT, t,0.0);

    
    Serial.print("Begining motion with moves:");
    Serial.println(mstate.buffer_size);
    
    start_motion();

  
    while(mstate.move){
      Serial.println(mstate.buffer_size);
      Serial.println(mstate.steps);
      
      delay(100);
    }
    Serial.print("Ending motion with moves:");
    Serial.println(mstate.buffer_size);
    
  }
}

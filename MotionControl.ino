#include "core_pins.h"
#include "motion_buffer.h"
#include "pin_maps.h"
#include "dda.h"


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
    add_move_to_buffer(d + 50, 0, t, t);
    add_move_to_buffer(d + 100, 0, t, 0.0);

    add_move_to_buffer(d + 100, 50, 0.0, t);
    add_move_to_buffer(d + 100, d + 50,  t, t);
    add_move_to_buffer(d + 100, d + 100, t, 0.0);
    
    add_move_to_buffer(d + 50, d + 100, 0.0, t);
    add_move_to_buffer(50,  d + 100, t, t);
    add_move_to_buffer(0, d + 100, t, 0.0);

    add_move_to_buffer(0, d + 50, 0.0, t);
    add_move_to_buffer(0, 50, t, t);
    add_move_to_buffer(0, 0, t, 0.0);

    add_move_to_buffer(50, 50, 0.0, t);
    add_move_to_buffer(500, 500, t, t);
    add_move_to_buffer(550, 550, t, 0.0);
    add_move_to_buffer(500, 500, 0.0, t);
    add_move_to_buffer(50, 50, t, t);
    add_move_to_buffer(0, 0, t, 0.0);

    

    
    Serial.print("Begining motion with moves:");
    Serial.println(mstate.buffer_size);
    
    start_motion();

  
    while(mstate.move){
      Serial.print(mstate.buffer_size);
      Serial.print(' ');
      Serial.print(dda.length_acc);
      Serial.print(' ');
      Serial.println(dda.length);
      
      delay(200);
    }
    Serial.print("Ending motion with moves:");
    Serial.println(mstate.buffer_size);
    
  }
}

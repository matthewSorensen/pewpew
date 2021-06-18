#include "core_pins.h"
#include "motion_buffer.h"
#include "pin_maps.h"
#include "dda.h"
#include "special_events.h"

// Things to implement:
// * planner that can deal with full s-curve motion.
// * s-curve motion (+ refactor those calculations to be testable like the dda)
// homing?
// track internal state more
// deal with really short segments - right now, if we break a move into sub-step
// segments, we'll still get correct step counts, but it'll run at maximum speed.

void add_move_blocking(double x, double y, double v0, double v1){
  while(free_buffer_spaces()==0){
    delayMicroseconds(10);
  }
  add_move_to_buffer(x, y, v0, v1);
}

// Good max velocity = 2 mm/s
// Good a = 32 mm/s^2
void add_segment(double* prev, double x, double y, double v, double a){
  double dx, dy, l, acc_length;
  double sx, sy, sl;
  double spx, spy;
  spx = steps_per_mm[0]  * prev[0];
  spy = steps_per_mm[1]  * prev[1];
    
  dx = x - prev[0];
  dy = y - prev[1];
  sx = steps_per_mm[0] * dx;
  sy = steps_per_mm[1] * dy;
  l = sqrt(dx * dx + dy * dy);
  sl = sqrt(sx * sx + sy * sy);  
  // How long will it take to accelerate/decelerate?
  acc_length = v*v / (2 * a);
  if(2 * acc_length < l){
    double sv = (v * sl / l) / 1000.0;
    sv *= 0.001;
    double scale = acc_length / l;

    add_move_blocking(spx + scale * sx, spy + scale * sy,0,sv);
    add_move_blocking(spx + (1 - scale) * sx,spy + (1 - scale) * sy ,sv,sv);
    add_move_blocking(spx + sx, spy + sy,sv,0);
  }else{
    double sv = sqrt(a * l) * sl / (1000000 * l);
    add_move_blocking(spx + 0.5 * sx, spy + 0.5 * sy, 0, sv);
    add_move_blocking(spx + sx, spy + sy,sv,0);   
  }
  prev[0] = x;
  prev[1] = y;
}


int main(void){

  Serial.begin(9600);
  
  pinMode(X_STEP, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(LASER, OUTPUT);

  initialize_motion_state();

  while(1){

    
    double prev[2];
    prev[0] = 0.0;
    prev[1] = 0.0;
    

    for(int i = 0; i < 15; i ++){

      special_segment_t* seg = add_event_to_buffer();
      seg->event_flags = FIRE_LASER;
      mstate.buffer_size += 1;
      
      add_segment(prev, 2 * cos(i * 2 * 3.14156 / 15), 2 * sin(i * 2 * 3.14156 / 15), 4.0, 32.0);

      seg = add_event_to_buffer();
      mstate.buffer_size += 1;
      
      add_segment(prev, 0.0, 0.0, 2.0, 32.0);
    }

    for(uint32_t i = 0; i < mstate.buffer_size; i ++){
      Serial.print(motion_buffer[i].coords[0], 5);
      Serial.print(' ');
      Serial.print(motion_buffer[i].coords[1], 5);
      Serial.print(' ');
      Serial.print(motion_buffer[i].start_velocity, 5);
      Serial.print(' ');
      Serial.print(motion_buffer[i].end_velocity, 5);
      Serial.println(' ');
    }

    
    Serial.print("Begining motion with moves:");
    Serial.println(mstate.buffer_size);
    
    start_motion();

  
    while(mstate.move){
      Serial.print(mstate.buffer_size);
      Serial.print(' ');
      Serial.print(' ');
      Serial.println(dda.prev_length, 5);
      
      delay(200);
    }
    Serial.print("Ending motion with moves:");
    Serial.println(mstate.buffer_size);

    delay(5000);
    
  }
}

#include <stdint.h>
#include <math.h>
#include "motion_buffer.h"
#include "core_pins.h"
#include "pin_maps.h"

#define TIE 2
#define TEN 1
#define TIF 1

// We're running at a bus frequency of 150MHz, which gives us...150 ticks per us...
#define TICKS_PER_US 150
// How long do we leave the pin high? Datasheet says 2.5us, but...
#define STEP_PULSE_LENGTH 10


motion_segment_t motion_buffer[MOTION_BUFFER_SIZE];
volatile motion_state_t mstate;
volatile uint32_t manual_trigger;


void initialize_motion_state(void){
  // Set up the PIT timers
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 1;
  CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL; // FBus (usually 150MHz)
  NVIC_ENABLE_IRQ(IRQ_PIT);
  attachInterruptVector(IRQ_PIT,stepper_isr);

  mstate.current_move = 0;
  mstate.buffer_size = 0;  
  mstate.move = NULL;

  manual_trigger = 0;
}

uint32_t free_buffer_spaces(void){
  return MOTION_BUFFER_SIZE - mstate.buffer_size;
}


uint32_t add_move_to_buffer(uint32_t steps, uint32_t direction, double start_velocity, double end_velocity){
  if(free_buffer_spaces() < 1)
    return 0;
  motion_segment_t* seg = &motion_buffer[(mstate.current_move + mstate.buffer_size) & MOTION_BUFFER_MASK];
  seg->steps = steps;
  seg->direction = direction;
  seg->start_velocity = start_velocity;
  seg->end_velocity = end_velocity;

  mstate.buffer_size += 1;
  return 1;
}

void compute_next_step(void){
  double dt;
  double v;
  // If there are no more steps in this segment, signal that and fail
  if(mstate.steps == 0){
    mstate.step_bitmask = 0;
    return;
  }
  // Otherwise, grab a step and set the step bits
  if(mstate.direction & Y_AXIS){
    mstate.step_bitmask = PIN_BITMASK(Y_STEP);
  }else{
    mstate.step_bitmask = PIN_BITMASK(X_STEP);
  }
  mstate.steps -= 1;
  // Figure out how long until the next step
  v = mstate.velocity*mstate.velocity + 2 * mstate.acceleration;
  if(v < 0.0){
    v = 0.0;
  }else{
    v = sqrt(v);
  }
  
  dt = 2 / (mstate.velocity + v);
  mstate.velocity = v;
  mstate.delay = round(dt * TICKS_PER_US);
}

uint32_t initialize_next_seg(uint32_t first){
  double dt;
  uint32_t dir;
  // If we're not starting a series of moves, advance along the ring buffer and
  // release the previous move.
  if(!first){
    mstate.current_move = (mstate.current_move + 1) & MOTION_BUFFER_MASK;
    mstate.buffer_size -= 1;
  }
  // If, after this, there aren't any more moves, null out a bunch of fields in the
  // current state and fail.
  if(mstate.buffer_size == 0){
    mstate.move = NULL;
    mstate.steps = 0;
    mstate.velocity = 0;
    mstate.acceleration = 0;
    return 0;
  }
  // Otherwise, copy a bunch of fields
  mstate.move = &motion_buffer[mstate.current_move];
  mstate.steps = mstate.move->steps;
  mstate.velocity = mstate.move->start_velocity;
  dir = mstate.move->direction;
  mstate.direction = dir;
  
  // And then compute how long this move will take, as a way to find the accleration
  dt = 2 * mstate.steps / (mstate.velocity + mstate.move->end_velocity);
  mstate.acceleration = (mstate.move->end_velocity - mstate.velocity) / dt;

  // Figure out what the direction pins should be
  mstate.dir_bitmask = 0;
  if(dir & INVERT){
    if(dir & Y_AXIS)
      mstate.dir_bitmask |= PIN_BITMASK(Y_DIR);
    else
      mstate.dir_bitmask |= PIN_BITMASK(X_DIR);
  }
  compute_next_step();
  return 1;
}


void stepper_isr(void){
  // Stepper pulse reset - reenter the ISR a few us after setting the pulse pin, and turn it off
  if(PIT_TFLG2){
    STEP_CLEAR = STEP_BITMASK;
    // Stop this timer, since it's a one-shot thingy
    PIT_TCTRL2 = 0;
    PIT_TFLG2 = TIF;
    // Output the next set of direction bits!
    DIR_REG = (DIR_REG & ~DIR_BITMASK) | mstate.dir_bitmask;
    
    return;
  }
  // ... not sure why this would happen, but otherwise make sure we're not getting stray PIT interrupts
  if(!(PIT_TFLG1 || manual_trigger)) return;
  manual_trigger = 0;

  // Turn off the timer...
  PIT_TCTRL1 = 0;
  PIT_TFLG1 = TIF;

  if(mstate.move != NULL){
    
    // Output the next pulse, trigger the pulse reset ISR, and set the timer for the next round
    if(mstate.step_bitmask){
      STEP_SET = mstate.step_bitmask; // Output the next pulse
      PIT_TCTRL2 = TIE | TEN; // Trigger the reset timer
      PIT_LDVAL1 = mstate.delay; // Update the delay
      PIT_TCTRL1 = TIE | TEN; // Trigger the next pulse
    }
      
    compute_next_step(); // Actually compute the step bits and delay for the next pulse
    if(mstate.step_bitmask == 0){ // If there were no steps left in the move, go on to the next one
      initialize_next_seg(0);
    }
  }
}
  
void start_motion(void){
  // Grab a chunk, or nope out if we don't have any
  if(!initialize_next_seg(1))
    return;
  // Output the direction bits and wait a bit (?)
  DIR_REG = (DIR_REG & ~DIR_BITMASK) | mstate.dir_bitmask;
  // Set up the step pulse reset timer
  PIT_LDVAL2 = STEP_PULSE_LENGTH * TICKS_PER_US;
  // Configure, but don't fire the main timing clock
  PIT_LDVAL1 = 0;
  PIT_TCTRL1 = TIE;
  // Then manually call the ISR to fire the first step of the move
  manual_trigger = 1;
  stepper_isr();
}

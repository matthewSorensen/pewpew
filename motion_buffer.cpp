#include <stdint.h>
#include <math.h>
#include "motion_buffer.h"
#include "core_pins.h"
#include "pin_maps.h"
#include "dda.h"
#include "special_events.h"

#define TIE 2
#define TEN 1
#define TIF 1

// We're running at a bus frequency of 150MHz, which gives us...150 ticks per us...
#define TICKS_PER_US 150
// How long do we leave the pin high? Datasheet says 2.5us, but...
#define STEP_PULSE_LENGTH 10

#define MIN_STEP_TICKS 1500


motion_segment_t motion_buffer[MOTION_BUFFER_SIZE];
volatile motion_state_t mstate;
volatile uint32_t manual_trigger;


void initialize_motion_state(void){
  // Set up the PIT timers
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 1;
  CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
  NVIC_ENABLE_IRQ(IRQ_PIT);
  attachInterruptVector(IRQ_PIT,stepper_isr);

  mstate.current_move = 0;
  mstate.buffer_size = 0;  
  mstate.move = NULL;
  mstate.move_id = 0;
  mstate.move_flag = 0;
  
  for(int i = 0; i < NUM_AXIS; i++){
    mstate.end[i] = 0.0;
    mstate.position[i] = 0;
  }
  manual_trigger = 0;
}

uint32_t free_buffer_spaces(void){
  return MOTION_BUFFER_SIZE - mstate.buffer_size;
}


// If there's space in the motion buffer for a new segment, return the segment. If not, return NULL.
// Note that this doesn't actually record the segment as taken (by incrementing the number of elements in the buffer),
// as otherwise the stepper ISR could see an uninitialized move!
motion_segment_t* next_free_segment(void){
  if(MOTION_BUFFER_SIZE <= mstate.buffer_size)
    return NULL;
  return  &motion_buffer[(mstate.current_move + mstate.buffer_size) & MOTION_BUFFER_MASK];
}


void compute_next_step(void){
  double dt;
  double v;
  double length;

  for(int i = 0; i < NUM_AXIS; i++){
    mstate.step_update[i] = 0;
  }
  
  uint32_t step_mask = compute_step(&length,mstate.step_update);
  uint32_t ticks;
  // If there are no more steps in this segment, signal that and fail
  mstate.step_bitmask = step_mask;
  if(!step_mask)
    return;
  // Figure out how long until the next step - first compute the end velocity
  // via v^2 = v0^2 + 2 a dx
  v = mstate.velocity*mstate.velocity + 2 * mstate.acceleration * length;
  v = v <= 0.0 ? 0 : sqrt(v);
  // Then compute how long the move will take, as we know the average velocity.
  dt = 2 / (mstate.velocity + v);
  mstate.velocity = v;
  ticks = round(dt * TICKS_PER_US);
  mstate.delay = ticks < MIN_STEP_TICKS ? MIN_STEP_TICKS : ticks;
}

// Returns 0 if we either failed to find a move or there's nothing left to do in the new move
// Returns 1 if there's something left to do - either steps or a delay. Sets all the relevant fields
// in the motion state.
uint32_t initialize_next_seg(uint32_t first){
  motion_segment_t* move;
  double dt;
  // If we're not starting a series of moves, advance along the ring buffer and
  // release the previous move.
  if(!first){
    mstate.current_move = (mstate.current_move + 1) & MOTION_BUFFER_MASK;
    mstate.buffer_size -= 1;
  }
  // If, after this, there aren't any more moves, null out the current move,
  // and note that we have 
  if(mstate.buffer_size == 0){
    mstate.move = NULL;
    return 0;
  }

  move = &motion_buffer[mstate.current_move];
  mstate.move = move;
  mstate.move_id = move->move_id;
  mstate.move_flag = move->move_flag;
  // If it's a special event, don't initialize the dda...
  if(mstate.move_flag)
    return 1;
  
  // Initialize the dda, from the end point of the last move, and the end of the new one, giving us
  // our new direction mask
  mstate.dir_bitmask = initialize_dda(mstate.end, move->coords);
  // Then we can update the end coordinates and velocity
  for(int i = 0; i<NUM_AXIS; i++){
    mstate.end[i] = move->coords[i];
  }
  mstate.velocity = mstate.move->start_velocity; 
  // And then compute how long this move will take, as a way to find the accleration
  dt = 2 * dda.qlength / (mstate.velocity + mstate.move->end_velocity);
  mstate.acceleration = (mstate.move->end_velocity - mstate.velocity) / dt;

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
    if(mstate.move_flag){
      uint32_t delay = execute_event((event_segment_t*) mstate.move);
      if(delay){ // We need to keep waiting for a bit
	PIT_LDVAL1 = TICKS_PER_US * delay;
	PIT_TCTRL1 = TIE | TEN;
	return;
      }else{
	// Ok, we're now done with that event! Try to grab a new segment...
	initialize_next_seg(0);
	if(mstate.move == NULL)
	  return;
	// If it's not a move, output the new direction bitmasks
	if(!mstate.move_flag)
	  DIR_REG = (DIR_REG & ~DIR_BITMASK) | mstate.dir_bitmask;
	// And set up a new timer, either to execute the event, or wait a bit and take the first
	// step of the move...
	PIT_LDVAL1 = TICKS_PER_US;
	PIT_TCTRL1 = TIE | TEN;
	return;
      }
    }else if(mstate.step_bitmask){
      // Output the next pulse, trigger the pulse reset ISR, and set the timer for the next round
      STEP_SET = mstate.step_bitmask; // Output the next pulse
      PIT_TCTRL2 = TIE | TEN; // Trigger the reset timer
      PIT_LDVAL1 = mstate.delay; // Update the delay
      PIT_TCTRL1 = TIE | TEN; // Trigger the next pulse
      // Update the step counter
      for(int i = 0; i < NUM_AXIS; i++){
	mstate.position[i] += mstate.step_update[i];
      }
      
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

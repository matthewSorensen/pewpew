#include "homing.h"
#include "pin_maps.h"
#include "motion_buffer.h"
#include "core_pins.h"
#include "machine_state.h"
#include "motion_buffer.h"

#define TIE 2
#define TEN 1
#define TIF 1


volatile homing_state_t homing_state;

// Look at the current homing state and set the direction pins to reflect the current state
void compute_and_set_direction_bits(void){

  uint32_t dir_bits = 0;

  for(int i = 0; i < NUM_AXIS; i++){
    uint32_t bit = 0;
    homing_phase_t phase = homing_state.current_phase[i];
    if(phase == HOMING_BACKOFF){
      bit = !bit;
    }
    if(home_pins[i].flags & HOME_REVERSE){
      bit = !bit;
    }
    if(bit){
      dir_bits |= motor_pins[i].dir_pin_bitmask;
    }
  }

  DIR_REG = (DIR_REG & ~DIR_BITMASK) | dir_bits;
}


void homing_isr(void){
  // PIT2 is reserved for clearing step pulses. Maybe a bit decadent, but why not?
  if(PIT_TFLG2){
    STEP_CLEAR = STEP_BITMASK;
    PIT_TCTRL2 = 0;
    PIT_TFLG2 = TIF;
    return;
  }

  
  if(!PIT_TFLG1)
    return;
  
  // Turn off the timer, perhaps only to turn it back on later 
  PIT_TCTRL1 = 0;
  PIT_TFLG1 = TIF;
  // Figure out what's up with all the axes!
  uint32_t steps = 0;
  for(int i = 0; i < NUM_AXIS; i++){
    homing_phase_t phase = homing_state.current_phase[i];
    homing_pins_t axis = home_pins[i];
    if(phase == HOMING_DONE) continue;

    uint32_t switch_state = !!(LIMIT_REG & axis.limit_pin_bitmask) ^ !!(axis.flags & HOME_INVERT) ^ (phase == HOMING_BACKOFF);

    if(switch_state){
      homing_state.current_phase[i] = HOMING_DONE;
      homing_state.unhomed_axes -= 1;
      mstate.position[i] = axis.home_position;
    }else{ // Otherwise keep truckin'
      steps |= motor_pins[i].step_pin_bitmask;
    } 
  }
  // If we have another step pulse (and thus aren't all done), set the pins
  // and set up all the timers for the next pulse
  if(steps){
    STEP_SET = steps; // Output the next pulse
    PIT_TCTRL2 = TIE | TEN; // Trigger the reset timer
    PIT_TCTRL1 = TIE | TEN; // Trigger the next pulse
  }else{
    set_status(STATUS_IDLE);
    send_status_message(0);
    attachInterruptVector(IRQ_PIT,stepper_isr);
  }
}


void start_homing(void* message){
  homing_message_t* ptr = (homing_message_t*) message;
  
  uint32_t axes = ptr->axes;
  homing_phase_t direction = (homing_phase_t) ptr->direction;
  double speed = ptr->speed;
  uint32_t step_delay = 150.0 / speed;
  
  homing_state.unhomed_axes = 0;
  if(direction == HOMING_DONE) return; // We're done!


  for(int i = 0; i < NUM_AXIS; i++){
    
    if(!(axes & (1<<i)) || (home_pins[i].flags & HOME_NONE)){
      homing_state.current_phase[i] = HOMING_DONE;
      continue;
    }
    
    homing_state.current_phase[i] = HOMING_APPROACH;
    homing_state.unhomed_axes += 1;
  }

  if(!homing_state.unhomed_axes) return;
  // Set the direction bits for that, and wait a bit...
  compute_and_set_direction_bits();
  // Set up the PIT timers and replace any ISRs with the homing step ISR
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 1;
  CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
  NVIC_ENABLE_IRQ(IRQ_PIT);
  PIT_TCTRL1 = 0;
  attachInterruptVector(IRQ_PIT,homing_isr);
  // And fire the ISR a microsecond after this exits
  PIT_LDVAL2 = 150 * 10; //STEP_PULSE_LENGTH * TICKS_PER_US;
  PIT_LDVAL1 = step_delay;
  PIT_TCTRL1 = TIE | TEN;
  cs.status = STATUS_HOMING;
  send_status_message(0);
}


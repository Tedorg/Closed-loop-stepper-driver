//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__
#include <stdbool.h> 
#include <stdio.h>
#include "util.h"



#define STATUS_OK 0
// Define system executor bit map. Used internally by realtime protocol as realtime command flags,
// which notifies the main program to execute the specified realtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the realtime protocol only needs to check for a non-zero value to
// know when there is a realtime command to execute.
#define EXEC_STATUS_REPORT      BIT0 // bitmask 00000001
#define EXEC_DRV_FAULT          BIT1 // bitmask 00000010
#define EXEC_UNREACH_TARGET     BIT2 // bitmask 00000100
#define EXEC_FEED_HOLD          BIT3 // bitmask 00001000
#define EXEC_RESET              BIT4 // bitmask 00010000
#define EXEC_DRV_DISABLED       BIT5 // bitmask 00100000
#define EXEC_MOTION_CANCEL      BIT6 // bitmask 01000000
#define EXEC_SLEEP              BIT7 // bitmask 10000000


#define STATE_STATUS_REPORT      BIT0 // bitmask 00000001
#define STATE_RESERVED_1         BIT1 // bitmask 00000010
#define STATE_REACHED_TARGET     BIT2 // bitmask 00000100
#define STATE_IDLE               BIT3 // bitmask 00001000
#define STATE_OPEN_LOOP_MODE     BIT4 // bitmask 00010000
#define STATE_CLOSED_LOOP_MODE   BIT5 // bitmask 00100000
#define STATE_INIT_STEPPER       BIT6 // bitmask 01000000
#define STATE_RESERVED_2         BIT7 // bitmask 10000000


extern volatile uint8_t exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
extern volatile uint8_t exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.

//interrupt vars

extern volatile int U ;       //control effort (abs)
extern volatile int r ;   //setpoint
extern volatile int r_1 ;   //PREVIOUS setpoint
extern volatile int y ;   // measured angle
extern volatile int v ;  // estimated velocity  (velocity loop)
extern volatile int yw ;  // "wrapped" angle (not limited to 0-360)
extern volatile int yw_1 ;
extern volatile double e;  // e = r-y (error)
extern volatile double p;  // proportional effort
extern volatile double i;  // integral effort

extern volatile double u;  //real control effort (not abs)
extern volatile double u_1;
extern volatile double e_1;
extern volatile float u_2;
extern volatile float e_2;
extern volatile float u_3;
extern volatile float e_3;
extern volatile long u_d;  //neede to reduce pwm frequenzy  if motor stucked

extern volatile uint8_t _busy ;
extern volatile vref_value;


extern volatile long wrap_count;
extern float wrap_quotient; // for fast division position calculation
extern volatile int y_1;

extern volatile long step_count;  //For step/dir interrupt
extern int stepNumber; // step index for cal routine


extern volatile double ITerm;
extern volatile double DTerm;
extern char mode;
extern volatile bool dir;
extern volatile bool fault;  
extern volatile bool fault_message_flag;  // indicates that a fault message has been fired
extern uint16_t current_vref_step; // maximum 100 count


extern bool print_yw;     //for step response, under development...

void state_machine(void);
void set_state(uint8_t code);
void set_alarm(uint8_t code);
void reset_status(void);
void reset_alarm(void);
void clear_state(uint8_t mask);
void clear_alarm(uint8_t mask);
void report_status_message();
void report_alarm_message();
void print_status(void);
void print_alarm(void);


#endif






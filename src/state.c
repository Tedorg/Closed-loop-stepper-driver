// Contains the declaration of the state variables for the control loop
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "state.h"
#include "parameter.h"
#include "stepper.h"
#include "stm32f1xx.h"

volatile uint8_t exec_state; // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t exec_alarm; // Global realtime executor bitflag variable for setting various alarms.

// interrupt vars

volatile int U = 0;  // control effort (abs)
volatile int r = 0;  // setpoint
volatile int r_1 = 0;  // setpoint
volatile int y = 0;  // measured angle
volatile int v = 0;  // estimated velocity  (velocity loop)
volatile int yw = 0; // "wrapped" angle (not limited to 0-360)
volatile int yw_1 = 0;
volatile double e = 0.0; // e = r-y (error)
volatile double p = 0.0; // proportional effort
volatile double i = 0.0; // integral effort

volatile double u = 0.0;   // real control effort (not abs)
volatile double u_1 = 0.0; // value of u at previous time step, etc...
volatile double e_1 = 0.0; // these past values can be useful for more complex controllers/filters
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long u_d = 10000;  //neede to reduce pwm frequenzy  if motor stucked
volatile uint8_t _busy = 0;
volatile vref_value = 0; // analog read of digital poti vref 

volatile long wrap_count = 0; // keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
float wrap_quotient = 1.0f/(float)ENCODER_COUNTS_PER_REVOLUTION; // for fast division position calculation

volatile int y_1 = 0;

volatile long step_count = 0; // For step/dir interrupt (closed loop)
int stepNumber = 0;           // open loop step number (used by 's' and for cal routine)

volatile double ITerm = 0;
volatile double DTerm = 0;

uint16_t current_vref_step = 30; // maximum 100 count

char mode;
volatile bool dir = true;
volatile bool fault = false;
volatile bool fault_message_flag = false; // indicates that a fault message has been fired

bool print_yw = false; // for step response, under development...

void state_machine(void)
{
  //read analog imput from the digital potentiometer
   vref_value = read_analog(7);
  if (exec_alarm & EXEC_STATUS_REPORT)
  {
    disable_pid_timer();
    enable_stepper(false);
    report_alarm_message(exec_alarm);
    CLEAR_BIT(exec_state, STATE_INIT_STEPPER);
    CLEAR_BIT(exec_alarm, STATE_STATUS_REPORT);
  }

  if (exec_state & STATE_STATUS_REPORT)
  {
    report_status_message(exec_state);
    CLEAR_BIT(exec_state, STATE_STATUS_REPORT);
  }
   if (exec_state & STATE_OPEN_LOOP_MODE)  set_led_1(0);
   if (exec_state & STATE_CLOSED_LOOP_MODE) set_led_1(1);
}

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.

void set_state(uint8_t code)
{
  SET_BIT(exec_state, code);
}
void set_alarm(uint8_t code)
{
  fault_message_flag = true;
  SET_BIT(exec_alarm, EXEC_STATUS_REPORT);
  SET_BIT(exec_alarm, code);
}

void clear_state(uint8_t mask)
{
  CLEAR_BIT(exec_state, mask);
}
void clear_alarm(uint8_t mask)
{
  CLEAR_BIT(exec_alarm, mask);
}

void reset_status() { exec_state = 0; }
void reset_alarm() { exec_alarm = 0; }

void report_status_message()
{

  printf("[%d]\n", exec_state);
}

// Prints alarm messages.
void report_alarm_message()
{
  printf("[%d]\n", exec_alarm);
}

void print_status()
{
  printf("State: \n");
  if (exec_state & STATE_STATUS_REPORT)
    printf("STATUS_REPORT \n");
  if (exec_state & STATE_REACHED_TARGET)
    printf("REACHED_TARGET \n");
  if (exec_state & STATE_IDLE)
    printf(" IDLE \n");
  if (exec_state & STATE_OPEN_LOOP_MODE)
    printf("OPEN_LOOP_MODE \n");
  if (exec_state & STATE_CLOSED_LOOP_MODE)
    printf("CLOSED_LOOP_MODE \n");
  if (exec_state & STATE_INIT_STEPPER)
    printf("INIT_STEPPER \n");
  if (exec_state & STATE_RESERVED_1)
    printf("RESERVED_1 \n");
  if (exec_state & STATE_RESERVED_2)
    printf("RESERVED_2 \n");
}
void print_alarm()
{
  printf("Alarm: \n");
  if (exec_alarm & EXEC_STATUS_REPORT)
    printf("STATUS_REPORT  \n");
  if (exec_alarm & EXEC_DRV_FAULT)
    printf("DRV_FAULT  \n");
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
// void report_feedback_message(uint8_t message_code)
// {
//   printPgmString(PSTR("[MSG:"));
//   switch(message_code) {
//     case MESSAGE_CRITICAL_EVENT:
//       printPgmString(PSTR("Reset to continue")); break;
//     case MESSAGE_ALARM_LOCK:
//       printPgmString(PSTR("'$H'|'$X' to unlock")); break;
//     case MESSAGE_ALARM_UNLOCK:
//       printPgmString(PSTR("Caution: Unlocked")); break;
//     case MESSAGE_ENABLED:
//       printPgmString(PSTR("Enabled")); break;
//     case MESSAGE_DISABLED:
//       printPgmString(PSTR("Disabled")); break;
//     case MESSAGE_SAFETY_DOOR_AJAR:
//       printPgmString(PSTR("Check Door")); break;
//     case MESSAGE_CHECK_LIMITS:
//       printPgmString(PSTR("Check Limits")); break;
//     case MESSAGE_PROGRAM_END:
//       printPgmString(PSTR("Pgm End")); break;
//     case MESSAGE_RESTORE_DEFAULTS:
//       printPgmString(PSTR("Restoring defaults")); break;
//     case MESSAGE_SPINDLE_RESTORE:
//       printPgmString(PSTR("Restoring spindle")); break;
//     case MESSAGE_SLEEP_MODE:
//       printPgmString(PSTR("Sleeping")); break;

//   }
//   report_util_feedback_line_feed();
// }

// // Welcome message
// void report_init_message()
// {
//   printPgmString(PSTR("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
// }

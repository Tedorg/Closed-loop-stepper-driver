#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#define firmware_version "0.0.1" // firmware version
#define identifier "x"           // change this to help keep track of multiple mechaduinos (printed on startup)

#include "stm32f1xx.h"

// Utility
//#define DEBUG
//#define DIAGNOSE
#define MAX_PARSE_VALUES 9

#define LED_BUILD_IN (13)             // Build in LED at PC13 only for Debugging
#define DIR_TO_DRIVER_PIN (4)         // PA4
#define PWM_TO_DRIVER_PIN (6)         // PA6
#define PWM_CURRENT_TO_DRIVER_PIN (7) // PA7 Dynamically adjust Torque at DRV8825

#define STEP_PIN (1)               // PA1 Input Steps ->Interrupt gets Signal from MASTER board
#define DIR_PIN (0)                // PA0 Input Dir    ->  gets Signal from MASTER board
#define ERROR_FROM_DRIVER_PIN (15) // PA15 Input Error    ->  gets Error from Driver

#define STEPS_PER_REVOLUTION 200
#define ENCODER_COUNTS_PER_REVOLUTION 4000
#define MICRO_STEPS (byte)16
#define HALF_REVOLUTION ((MICRO_STEPS * STEPS_PER_REVOLUTION)/2)

// PWM for VREF
#define VREF_MIN 5
#define VREF_MAX 38 //
#define VREF_FIXED 15 //

#define STEP_PWM_MIN_HZ 2000
#define STEP_PWM_MAX_HZ 30000

// Invert encoder direction
#define ENCODER_DIR_INVERTED 0

// Invert Stepper direction
#define STEPPER_DIR_INVERTED 0
extern volatile float Fs;
extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;
extern float Ts;
extern float fc;
extern float e_t; // error tolernce
extern uint16_t v_ref_min ;
extern uint16_t v_ref_max;
extern uint16_t v_ref_open_loop;
extern uint16_t activityCount ; ///< How many ticks since last setpoint change.
extern uint16_t activityThres ; ///< Threshold for turning off the output.
extern uint8_t errThres;       ///< Threshold with hysteresis.
extern int32_t integral;       ///< Sum of previous errors for integral.
extern long setpoint;          ///< Position reference.
extern float emaAlpha;         ///< Weight factor of derivative EMA filter.
extern float prevInput;        ///< (Filtered) previous input for derivative.
extern  uint8_t pid_busy ;


void set_gains(char *args, uint16_t lenght);
void store_data_to_flash(void);
void read_data_from_flash(void);
char s(float f);
void parameterQuery(void);
extern const uint32_t lookup_frequencies[734][2];
extern const uint32_t lookup[4000];

#endif

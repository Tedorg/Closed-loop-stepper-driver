#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <stdint.h>
extern uint16_t current_vref_step;

void init_stepper(void);
void get_dir(void);
void set_dir(bool _dir);
void sleep_stepper(bool on);
void enable_stepper(bool on);
void set_microstep(byte mode);
void set_microstep_pins(byte code);
void one_step(void);
void init_vref(void);
void store_vref(void);
void set_vref_threshold(uint8_t target_vref_min);
void set_vref_pwm(uint8_t step);
void disable_step_pwm(void);
void enable_one_step(void);
void init_step_pwm(void);
void set_step_pwm( uint32_t frequenzy);
void disable_step_pwm(void);
void init_hardware_decoder_1(void);
void init_hardware_decoder_2(void);
void reset_position(void);
void calibrate(void);




#endif
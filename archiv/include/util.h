#ifndef __UTILS_H__
#define __UTIL_H__


void init_uart(void);
void init_io(void );
void Interrupt_Config(void);
void init_timer3_for_pwm(void);
void set_frequenzy_timer3_for_pwm(void);

void SysTick_Handler(void);
void init_clock(void);
void ms_delay(int ms);


#endif
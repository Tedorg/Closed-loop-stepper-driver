


#ifndef __PROTOCOL_H__
#define  __PROTOCOL_H__

void enable_pid_timer(void);
void disable_pid_timer(void);
void set_setpoint(int t);

void init_pid(void);
void _pid(void);
uint16_t pid_2(void);
 void resetActivityCounter(void);
 void resetIntegral(void);
void setEMACutoff(float f_c);
float calcAlphaEMA(float fn);


#endif
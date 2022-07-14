#ifndef __SERIAL_H__
#define __SERIAL_H__

// serial.h
// Functions available in the stm32f0discovery serial library
// Author: Frank Duignan
#define NEWLINE '\n'
#define LINEFEED 0x0a

void init_uart(void);
int ReadCom(int Max,unsigned char *Buffer);
int _write(int file, char *ptr, int len);
int WriteCom(int Count, char *Buffer);
int rxAvailable();
int eputs(char *String);
int egets(char *String, int size);
#endif
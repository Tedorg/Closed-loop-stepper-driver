#ifndef __UTILS_H__
#define __UTIL_H__


// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

#ifndef bool
#define bool _Bool 
#endif // boolo

#define abs(x) ((x)>0?(x):-(x))
  
#define mod(a,b) ((a%b+b)%b)

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define SIZEOF(arr) sizeof(arr) / sizeof(*arr)


#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)
#define BIT16 (1 << 16)
#define BIT17 (1 << 17)
#define BIT18 (1 << 18)
#define BIT19 (1 << 19)
#define BIT20 (1 << 20)
#define BIT21 (1 << 21)
#define BIT22 (1 << 22)
#define BIT23 (1 << 23)
#define BIT24 (1 << 24)
#define BIT25 (1 << 25)
#define BIT26 (1 << 26)
#define BIT27 (1 << 27)
#define BIT28 (1 << 28)
#define BIT29 (1 << 29)
#define BIT30 (1 << 30)
#define BIT31 (1 << 31)

#define PAGE(NUMBER) (NUMBER *0x400 +134217728)
#define PAGE_CALC(_BYTES) (_BYTES/0x400)


extern volatile uint32_t usTicks;
extern uint32_t time;

typedef uint8_t byte;

#ifndef EEPROM_EMULATION_SIZE
#define EEPROM_EMULATION_SIZE (1024 - sizeof(_Bool) - sizeof(uint32_t))
#endif
typedef uint8_t byte;

#define enable_interrupts() asm(" cpsie i ")
#define disable_interrupts() asm(" cpsid i ")

uint16_t status_aux_1(void);
uint16_t status_aux_2(void);
void set_led_1(byte state);
void set_led_2(byte state);

void init_io(void );
void init_analog(void);
void interrupt_config(void);

void SysTick_Handler(void);
void init_clock(void);
void setup_timer(uint16_t hz);
void asm_delay(int ticks);
uint16_t position(void);
//void position2(void);
void parse_message(char *input, int length);
void serial_check(void);
void print_position(void);  
void print_setpoint(void);
void print_enc_position(void); 
void print_real_vref_value(void);
void diagnostic(void);

void delay_us(uint32_t dlyTicks);

uint16_t * faktorize_frequenzy_computed(uint16_t hz);
uint16_t * faktorize_frequenzy(uint32_t hz);
int * ctoi(char *input, int length);
#endif
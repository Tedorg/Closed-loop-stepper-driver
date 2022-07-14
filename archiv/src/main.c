#include <stdio.h>
#include "stm32f1xx.h"
#include "util.h"
#include "parameter.h"
#include "FLASH_PAGE_F1.h"
#include "string.h"
uint32_t data2[] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9};
uint32_t Rx_Data[30];
int number = 123;
float RxVal;
int main(void)
{
     
 SystemInit();
    init_clock();
  
    init_io();
    init_uart();
    Interrupt_Config();
 
    if (SysTick_Config(SystemCoreClock / 1000) != 0) // 1ms tick
    {
        // Error Handling
    }
    init_timer3_for_pwm();
    set_frequenzy_timer3_for_pwm();
    printf("Firmware %s \nid %s\n",firmware_version,identifier);
  //Flash_Write_NUM(0x08012000, number);
  RxVal = Flash_Read_NUM(0x0);
  printf("FALsbh read %d \n", RxVal);
    for (;;)
    {
        ms_delay(2000);
        GPIOC->ODR ^= (1 << LED_BUILD_IN); // toggle diodes
    }
    
    return 0;
}




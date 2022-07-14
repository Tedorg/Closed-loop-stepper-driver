#include <stdio.h>
#include <math.h>

#include "stm32f1xx.h"
//#include <FlashStorage_STM32F1.h>

#include "util.h"
#include "state.h"
#include "parameter.h"
#include "stepper.h"

#include "string.h"
#include "nvm.h"
#include "serial.h"

uint32_t time;
uint32_t previous_time = 0;
uint8_t toggler = 0;
//#define DIAGNOSE
#ifdef DIAGNOSE
uint32_t test_position = 0;
#endif

int main(void)
{
  SystemInit();
  init_clock();
  init_io();
  init_analog();

  init_uart();
  interrupt_config();

  if (SysTick_Config(SystemCoreClock / 1000000) != 0) // 1000ms tick
  {
    printf("Fault:  %ld \n", SystemCoreClock);
  }
  // eputs("\r\ntest\r\n");
  printf("SystemCoreClock %ld\n", SystemCoreClock);
  set_led_1(1);
  // init_step_pwm();
  // set_step_pwm(1000);
  // char text[50];
  // memset(lookup,0,sizeof(lookup));

  // printf("size of:  %d\n",sizeof(lookup));
  // memset(Rx_Data,0,sizeof(Rx_Data));

  disable_interrupts();

  enable_interrupts();
  init_hardware_decoder_2();

  read_data_from_flash();
set_vref_threshold(v_ref_min);  // setup_timer(Fs);
  //  size_t n = sizeof(Rx_Data)/sizeof(Rx_Data[0]);
  //  int i = 0;
  //  printf("len : %d \n", n);
  // while (i<=n)printf("values: %d \n", Rx_Data[i++] );
  // init_stepper();
  // set_vref_pwm(20);
  set_led_1(0);
  // printf("get page:  %d\n",GetPage(0x08001000));

  // init_step_pwm();
  //  set_step_pwm(10);
  // calibrate();
  // size_t n = sizeof(frequencies);
  // printf("sizeof frequenciesp %d \n", PAGE_CALC(n));
 // printf("FLASH: PAGE 82 0x%x \n", PAGE(82));
// printf("half: %d \n",  HALF_REVOLUTION );
  set_state(STATE_IDLE);
  //set_state(STATE_OPEN_LOOP_MODE);
  static uint8_t toggle = 0;
  for (;;)
  {
    
    time = usTicks;
    if (time - previous_time > 6000)
    {
      toggle = !toggle;
      set_led_1(toggle);
      // parameterQuery();
      if (exec_state & STATE_CLOSED_LOOP_MODE)
      {
        uint8_t _u = (uint8_t)fabs(ITerm) + v_ref_min ;
        uint8_t _vref = constrain(_u, v_ref_min, v_ref_max);
        set_vref_pwm(_vref);
      }
      if (exec_state & STATE_OPEN_LOOP_MODE){
        if(exec_state & STATE_INIT_STEPPER)set_vref_pwm(v_ref_open_loop);
      }
      state_machine();
      #ifdef DIAGNOSE
    uint32_t idx = TIM1->CNT;
    if (idx != test_position)
      printf("%ld  %ld \n", idx,yw);
    test_position = idx;
#endif
      previous_time = time;
      // if(exec_state & STATE_CLOSED_LOOP_MODE)printf("%d %d\n", r, yw);
    }

    serial_check();
    // if (exec_state & STATE_OPEN_LOOP_MODE)
    //   position();


    //   asm_delay(10000);
    // GPIOC->ODR ^= (1 << LED_BUILD_IN); // toggle diodes
  }

  return 0;
}

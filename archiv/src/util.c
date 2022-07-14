#include <stdio.h>
#include "stm32f1xx.h"
#include "state.h"
#include "parameter.h"
#include "FLASH_PAGE_F1.h"
#include "string.h"


volatile uint32_t msTicks = 0 ;
volatile uint32_t flag = 0 ;
volatile uint32_t devider_frequenzy = 2;



void init_uart(){
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

    // PA2 (TxD) shall use the alternate function with push-pull
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF2 + GPIO_CRL_MODE2, GPIO_CRL_CNF2_1 + GPIO_CRL_MODE2_1);

    // Enable transmitter, receiver and receive-interrupt of USART2
    USART2->CR1 = USART_CR1_UE + USART_CR1_TE + USART_CR1_RE + USART_CR1_RXNEIE;

    // Set baudrate, assuming that USART2 is clocked with 
    // the same frequency as the CPU core (no prescalers).
    //USART2->BRR = (0x1D4C);
    
    // With > 36 MHz system clock, the USART2 receives usually half of it:
    USART2->BRR = (SystemCoreClock /16/9600); //74880 baudrate

    // Enable interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
    
    printf("%ld\n",GPIOA->IDR & (0x1));
    printf("%s\n","ready");


}



// Redirect standard output to the serial port
int _write(int file, char *ptr, int len)
{
    for (int i=0; i<len; i++)
    {
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = *ptr++;
    }
    return len;
}

// called after each received character
void USART2_IRQHandler()
{
    char received=USART2->DR;

    // send echo back
    while(!(USART2->SR & USART_SR_TXE));
    USART2->DR = received;
}



void init_io()
{
    // Enable Port A, B and alternate functions
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPBEN+ RCC_APB2ENR_IOPCEN + RCC_APB2ENR_AFIOEN);

    // PA6 = Timer 3 channel 1 alternate function output
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6 + GPIO_CRL_MODE6, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);

    // PA7 = Timer 3 channel 2 alternate function output
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7 + GPIO_CRL_MODE7, GPIO_CRL_CNF7_1 + GPIO_CRL_MODE7_0);

    // PB0 = Timer 3 channel 3 alternate function output
   // MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0 + GPIO_CRL_MODE0, GPIO_CRL_CNF0_1 + GPIO_CRL_MODE0_0);

    // PB1 = Timer 3 channel 4 alternate function output
   // MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF1 + GPIO_CRL_MODE1, GPIO_CRL_CNF1_1 + GPIO_CRL_MODE1_0);

/* ENABLE all OUTputs */
// LED at PC13 for DeBUG
    GPIOC->CRH |=GPIO_CRH_MODE13_0; // set PC13 to be general purpose output

// PA1 as Input Interrupt
GPIOA->CRL &= ~(0xf<<4);  // clear bits (7:6:5:4) PA1
GPIOA->CRL |= (8<<4);  // Bits (7:6:5:4) = 1:0:0:0  --> PA1 in Input Mode in Pull-up/ Pull-down mode
GPIOA->ODR |= (0<<1);  //  --> PA1 is in Pull DOWN mode


// PA0 as Input 
 GPIOA->CRL &= ~(0xf<<0);  // clear bits (3:2:1:0) PA0
GPIOA->CRL |= (8<<0);  // Bits (3:2:1:0) = 1:0:0:0  --> PA0 in Input Mode in Pull-up/ Pull-down mode
GPIOA->ODR |= (0<<0);  //  --> PA0 is in Pull DOWN mode


}

void Interrupt_Config (void){
    /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Enable the AFIO CLOCK bit in RCC register  -> siehe io_config
	2. Configure the EXTI configuration Regiter in the AFIO
	3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt
	
	********************************************************/
	

	AFIO->EXTICR[0] &= ~(0xf<<4);  // Bits[7:6:5:4] = (0:0:0:0)  -> configure EXTI1 line for PA1
	
	EXTI->IMR |= (1<<1);  // Bit[1] = 1  --> Disable the Mask on EXTI 1
	
	EXTI->RTSR |= (1<<1);  // Enable Rising Edge Trigger for PA1
	
	EXTI->FTSR &= ~(1<<1);  // Disable Falling Edge Trigger for PA1
	
	NVIC_SetPriority (EXTI1_IRQn, 1);  // Set Priority
	
	NVIC_EnableIRQ (EXTI1_IRQn);  // Enable Interrupt

}

void EXTI1_IRQHandler(void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Check the Pin, which trgerred the Interrupt
	2. Clear the Interrupt Pending Bit
	
	********************************************************/

 
	GPIOC->ODR ^= (1<<LED_BUILD_IN);  // toggle diodes
	if (EXTI->PR & (1<<1))    // If the PA1 triggered the interrupt
    {
        flag = 1;
        EXTI->PR |= (1 << 1); // Clear the interrupt flag by writing a 1
    }
    if (GPIOA->IDR & (0x1))
    {
        step_count++;
    }
    else
    {
        step_count--;
    }

    printf("%ld\n",step_count);
}


void init_timer3_for_pwm()
{
    // Enable timer 3
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

    // Timer 3 channel 1 compare mode = PWM1 with the required preload buffer enabled
    MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M + TIM_CCMR1_OC1PE, 
        TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);

    // Timer 3 channel 2 compare mode=PWM1 with the required preload buffer enabled
    MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE, 
        TIM_CCMR1_OC2M_2 + TIM_CCMR1_OC2M_1 + TIM_CCMR1_OC2PE);

    // Timer 3 channel 3 compare mode = PWM1 with the required preload buffer enabled
   /*  MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_OC3M + TIM_CCMR1_OC1PE, 
        TIM_CCMR2_OC3M_2 + TIM_CCMR2_OC3M_1 + TIM_CCMR1_OC1PE); */

    // Timer 3 channel 4 compare mode = PWM1 with the required preload buffer enabled
  /*   MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_OC4M + TIM_CCMR1_OC1PE, 
       TIM_CCMR2_OC4M_2 + TIM_CCMR2_OC4M_1 + TIM_CCMR1_OC1PE); */

    // Timer 3 enable all four compare outputs
    SET_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);

    // Timer 3 inverse polarity for all four compare outputs
    // SET_BIT(TIM3->CCER, TIM_CCER_CC1P + TIM_CCER_CC2P + TIM_CCER_CC3P + TIM_CCER_CC4P);
   
    // Timer 3 auto reload register, defines the maximum value of the counter in PWM mode.
  //7  TIM3->ARR = 50000; // 8000000/50000 = 160 pulses per second
    
    // Timer 3 clock prescaler, the APB2 clock is divided by this value +1.
  //  TIM3->PSC = 0; // divide clock by 1 

    // Timer 3 enable counter and auto-preload
   // SET_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
}

void set_frequenzy_timer3_for_pwm(void){

    TIM3->ARR = 5000; // 8000000/50000 = 160 pulses per second
    
    // Timer 3 clock prescaler, the APB2 clock is divided by this value +1.
    TIM3->PSC = 0; // divide clock by 1 

    // Timer 3 enable counter and auto-preload
    SET_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
    TIM3->CCR1 =  TIM3->ARR/100;
    // TIM3->CCR2 = 400;
    // TIM3->CCR3 = 4000;
    // TIM3->CCR4 = 40000; 

}






void SysTick_Handler(void)  
{
    msTicks++ ;
	//LEDPORT->ODR ^= (1<<LED1);  // toggle diodes
	//ms_delay(msTicks);
}


void init_clock()
{
    // Because the debugger switches PLL on, we may need to switch
    // back to the HSI oscillator before we can configure the PLL

    // Enable HSI oscillator
    SET_BIT(RCC->CR, RCC_CR_HSION);

    // Wait until HSI oscillator is ready
    while(!READ_BIT(RCC->CR, RCC_CR_HSIRDY)) {}

    // Switch to HSI oscillator
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait until the switch is done
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {}

    // Disable the PLL
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until the PLL is fully stopped
    while(READ_BIT(RCC->CR, RCC_CR_PLLRDY)) {}
    
    // Flash latency 2 wait states
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);

    // Enable HSE oscillator
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // Wait until HSE oscillator is ready
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY)) {}

    // 72 MHz using the 8 MHz HSE oscillator with 9x PLL, lowspeed I/O runs at 36 MHz
    WRITE_REG(RCC->CFGR, RCC_CFGR_PLLSRC + RCC_CFGR_PLLMULL6 + RCC_CFGR_PPRE1_DIV2);

    // Enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until PLL is ready
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY)) {}

    // Select PLL as clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    // Update variable
    SystemCoreClock=48000000;
    
    // Disable the HSI oscillator
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

void ms_delay(int ms)
{
   while (ms-- > 0) {
      volatile int x=500;
      while (x-- > 0)
         __asm("nop");
   }
}
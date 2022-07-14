
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#include "stm32f1xx.h"

#include "parameter.h"
#include "protocol.h"

#include "util.h"
#include "stepper.h"
#include "state.h"
#include "string.h"

volatile uint32_t usTicks = 0;

volatile uint32_t flag = 0;

uint8_t toggler_pid = 0;

#define bufferLength 64          // serial buffer length
char serialBuffer[bufferLength]; // serial buffer

// 9 is the max number of values I need to store for one single command. for example:
// b x1,y1, ax1,ay1, ax2,ay2, x2,y2, targetspeed  = 9 values

uint16_t status_aux_1(void)
{
    return GPIOB->IDR & (1 << 12);
}
uint16_t status_aux_2(void)
{
    return GPIOB->IDR & (1 << 13);
}
void set_led_1(byte state)
{
    GPIOB->BSRR = GPIO_BSRR_BR7; // Set PB7 bit
    if (state)
        GPIOB->BSRR = GPIO_BSRR_BS7; // Set PB bit
}

void set_led_2(byte state)
{
    // GPIOB->BSRR = GPIO_BSRR_BR8; // Set PB8 bit
    // if (state)
    // GPIOB->BSRR = GPIO_BSRR_BS8; // Set PB15 bit
}

void init_io()
{
    // Enable Port A, B and alternate functions
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPBEN + RCC_APB2ENR_IOPCEN + RCC_APB2ENR_AFIOEN);

    /*https://tty.uchuujin.de/2016/02/stm32-from-scratch-serial/https://tty.uchuujin.de/2016/02/stm32-from-scratch-serial/*/
    /* ENABLE all OUTputs */

    // PA6 = Timer 3 channel 1 alternate function output
    //	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6 + GPIO_CRL_MODE6, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);
    // GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;
    // GPIOA->CRL &= ~(GPIO_CRL_CNF6_0 | GPIO_CRL_MODE6_1);

    // LED at PC13 for DeBUG
    GPIOC->CRH |= GPIO_CRH_MODE13_0; // set PC13 to be general purpose output
    /* ENABLE PB7 PB8 as output for Status-LED */
    GPIOB->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;
    GPIOB->CRL &= ~(GPIO_CRL_CNF7_0 | GPIO_CRL_MODE7_1);

    GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1;
    GPIOB->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_MODE8_1);

    /* Dir to Driver PA4*/
    GPIOA->CRL |= GPIO_CRL_MODE4_1 | GPIO_CRL_CNF4_1;
    GPIOA->CRL &= ~(GPIO_CRL_CNF4_0 | GPIO_CRL_MODE4_1);

    /* Sleep to Driver PA5*/
    GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;
    GPIOA->CRL &= ~(GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_1);

    /* MS1 MS2 MS3*/

    GPIOB->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1; // MS1 PB0
    GPIOB->CRL &= ~(GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0_1);

    GPIOB->CRL |= GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1; // MS2 PB1
    GPIOB->CRL &= ~(GPIO_CRL_CNF1_0 | GPIO_CRL_MODE1_1);

    GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1; // MS3 PB10
    GPIOB->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_MODE10_1);

    /* Enable Stepper*/

    GPIOB->CRH |= GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1; // PB11
    GPIOB->CRH &= ~(GPIO_CRH_CNF11_0 | GPIO_CRH_MODE11_1);

    /* Enable DS1804*/
    GPIOA->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1; // Up and Down
    GPIOA->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_MODE10_1);

    GPIOA->CRH |= GPIO_CRH_MODE11_1 | GPIO_CRH_CNF11_1;    //
    GPIOA->CRH &= ~(GPIO_CRH_CNF11_0 | GPIO_CRH_MODE11_1); // Increment

    GPIOB->CRH |= GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1;    //
    GPIOB->CRH &= ~(GPIO_CRH_CNF15_0 | GPIO_CRH_MODE15_1); // CS
    GPIOB->BSRR = GPIO_BSRR_BR15;                          // CS Pin is low

    /* ENABLE AUX Inputs*/
    /* PB13 PB12*/
    GPIOB->CRH &= ~(0xf << 16); // clear bits (19:18:17:16)
    GPIOB->CRH |= (8 << 16);    // Bits (7:6:5:4) = 1:0:0:0  --> PA12 in Input Mode in Pull-up/ Pull-down mode
    GPIOB->ODR |= (0 << 12);    //  --> PA12 is in Pull DOWN mode

    GPIOB->CRH &= ~(0xf << 20); // clear bits (7:6:5:4)
    GPIOB->CRH |= (8 << 20);    // Bits (7:6:5:4) = 1:0:0:0  --> PA13 in Input Mode in Pull-up/ Pull-down mode
    GPIOB->ODR |= (1 << 13);    //  --> PA13 is in Pull DOWN mode

    // PA1 as Input Interrupt
    GPIOA->CRL &= ~(0xf << 4); // clear bits (7:6:5:4) PA1
    GPIOA->CRL |= (8 << 4);    // Bits (7:6:5:4) = 1:0:0:0  --> PA1 in Input Mode in Pull-up/ Pull-down mode
    GPIOA->ODR |= (0 << 1);    //  --> PA1 is in Pull DOWN mode

    // PA0 as Input
    GPIOA->CRL &= ~(0xf << 0); // clear bits (3:2:1:0) PA0
    GPIOA->CRL |= (8 << 0);    // Bits (3:2:1:0) = 1:0:0:0  --> PA0 in Input Mode in Pull-up/ Pull-down mode
    GPIOA->ODR |= (0 << 0);    //  --> PA0 is in Pull DOWN mode

    // PB9 as Input
    GPIOB->CRH &= ~(0xf << 4); // clear bits (7:6:5:4) PB9
    GPIOB->CRH |= (8 << 4);    // Bits (7:6:5:4) = 1:0:0:0  --> PB9 in Input Mode in Pull-up/ Pull-down mode
    GPIOB->ODR |= (0 << 9);    //  --> PB9 is in Pull UP mode

    // Configure PA7 as analog input ADC12_IN7

    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7 + GPIO_CRL_MODE1, 0);
}

void init_analog()
{
    // Divide APB2 clock frequency by 8
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_0 + RCC_CFGR_ADCPRE_1);

    // Enable clock for ADC
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);

    // Switch the ADC on
    SET_BIT(ADC1->CR2, ADC_CR2_ADON);

    // Select software start trigger
    MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, ADC_CR2_EXTSEL_0 + ADC_CR2_EXTSEL_1 + ADC_CR2_EXTSEL_2);

    // Set sample time to 41.5 cycles
    MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0, ADC_SMPR2_SMP0_2);

    // Delay 20 ms
    asm_delay(20);

    // Start calibration
    SET_BIT(ADC1->CR2, ADC_CR2_ADON + ADC_CR2_CAL);

    // Wait until the calibration is finished
    while (READ_BIT(ADC1->CR2, ADC_CR2_CAL))
        ;
}
// Read from an analog input
uint16_t read_analog(int channel)
{
    // Number of channels to convert: 1
    MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 0);

    // Select the channel
    MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, channel);

    // Clear the finish flag
    CLEAR_BIT(ADC1->SR, ADC_SR_EOC);

    // Start a conversion
    // These two bits must be set by individual commands!
    SET_BIT(ADC1->CR2, ADC_CR2_ADON);
    SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);

    // Wait until the conversion is finished
    while (!READ_BIT(ADC1->SR, ADC_SR_EOC))
        ;

    // Return the lower 12 bits of the result
    return ADC1->DR & 0b111111111111;
}

void interrupt_config(void)
{

    /*************>>>>>>> STEP IRQ INIT <<<<<<<<************

    1. Enable the AFIO CLOCK bit in RCC register  -> siehe io_config
    2. Configure the EXTI configuration Regiter in the AFIO
    3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
    4. Configure the Rising Edge / Falling Edge Trigger
    5. Set the Interrupt Priority
    6. Enable the interrupt

    ********************************************************/

    AFIO->EXTICR[0] &= ~(0xf << 4); // Bits[7:6:5:4] = (0:0:0:0)  -> configure EXTI1 line for PA1

    EXTI->IMR |= (1 << 1); // Bit[1] = 1  --> Disable the Mask on EXTI 1

    EXTI->RTSR |= (1 << 1); // Enable Rising Edge Trigger for PA1

    EXTI->FTSR &= ~(1 << 1); // Disable Falling Edge Trigger for PA1

    NVIC_SetPriority(EXTI1_IRQn, 1); // Set Priority

    NVIC_EnableIRQ(EXTI1_IRQn); // Enable Interrupt

    /*************>>>>>>> DIR IRQ INIT <<<<<<<<************
     ********************************************************/

    AFIO->EXTICR[0] &= ~(0xf << 0); // Bits[3:2:1:0] = (0:0:0:0)  -> configure EXTI1 line for PA0

    EXTI->IMR |= (1 << 0);  // Bit[1] = 1  --> Disable the Mask on EXTI 0
    EXTI->RTSR |= (1 << 0); // Enable Rising Edge Trigger for PA0
    EXTI->FTSR |= (1 << 0); // Enable Falling Edge Trigger for PA0

    NVIC_SetPriority(EXTI0_IRQn, 2); // Set Priority

    NVIC_EnableIRQ(EXTI0_IRQn); // Enable Interrupt

    /*************>>>>>>> AUX IRQ INIT <<<<<<<<************
     ********************************************************/
    MODIFY_REG(AFIO->EXTICR[2], AFIO_EXTICR3_EXTI9, AFIO_EXTICR3_EXTI9_PB);
    SET_BIT(EXTI->IMR, EXTI_IMR_MR9);
    // SET_BIT(EXTI->RTSR, EXTI_RTSR_TR9);
    EXTI->RTSR &= ~(1 << 9); // Disable Rising Edge Trigger for PA1

    EXTI->FTSR |= (1 << 9); // Enable Falling Edge Trigger for PA0

    // Enable the interrupt handler call
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // Clear pending interrupt flag
    SET_BIT(EXTI->PR, EXTI_PR_PR9);

    /*************>>>>>>> FAULT IRQ INIT <<<<<<<<************
     ********************************************************/
    // Assign EXTI13 to PC13 with rising edge
    MODIFY_REG(AFIO->EXTICR[3], AFIO_EXTICR4_EXTI13, AFIO_EXTICR4_EXTI13_PB);
    SET_BIT(EXTI->IMR, EXTI_IMR_MR13);
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR13); // enable Rising Edge Trigger for PB13
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR13); // enable Rising Edge Trigger for PB13

    // Enable the interrupt handler call
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // Clear pending interrupt flag
    SET_BIT(EXTI->PR, EXTI_PR_PR13);
}
void EXTI1_IRQHandler(void)
{
    /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
    STep counter IRQ
    1. Check the Pin, which trgerred the Interrupt
    2. Clear the Interrupt Pending Bit

    ********************************************************/

    if (EXTI->PR & (1 << 1)) // If the PA1 triggered the interrupt
    {

        EXTI->PR |= (1 << 1); // Clear the interrupt flag by writing a 1
    }
    // if (GPIOA->IDR & (0x1))
    if (dir)
    {
        r++;
    }
    else
    {
        r--;
    }

    if (exec_state & STATE_CLOSED_LOOP_MODE)
        set_setpoint(r);
    if (exec_state & STATE_OPEN_LOOP_MODE)
    {
        setpoint = r;
        one_step();
    }

#ifdef DEBUG
    printf("%d\n", r);
#endif
}
/*************>>>>>>> DIR Interrupt <<<<<<<<************/
void EXTI0_IRQHandler(void)
{
    /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
    DIR IRQ
    1. Check the Pin, which trgerred the Interrupt
    2. Clear the Interrupt Pending Bit

    ********************************************************/

    GPIOC->ODR ^= (1 << LED_BUILD_IN); // toggle diodes
    if (EXTI->PR & (1 << 0))           // If the PA1 triggered the interrupt
    {
        flag = 1;
    }
    // if (GPIOA->IDR & (0x1))
    if (GPIOA->IDR & (0x1))
    {
        dir = false;
    }
    else
    {
        dir = true;
    }
    set_dir(dir);
    EXTI->PR |= (1 << 0); // Clear the interrupt flag by writing a 1
#ifdef DEBUG
    printf("dir: %d\n", dir);
#endif
}
/*************>>>>>>>  AUX 1  IRQ <<<<<<<<************

  ********************************************************/
void EXTI9_5_IRQHandler()
{
    SET_BIT(EXTI->PR, EXTI_PR_PR9);
    if ((GPIOB->IDR & (0x200)))
    {
        reset_position();
    }
}
/*************>>>>>>> FAULT Interrupt <<<<<<<<************/

void EXTI15_10_IRQHandler()
{
    /*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
    FAult  IRQ
    ********************************************************/

    SET_BIT(EXTI->PR, EXTI_PR_PR13);
    // EXTI->PR |= (1 << 4); // Clear the interrupt flag by writing a 1

    if (!(GPIOB->IDR & (0x2000)))
    {
        set_alarm(EXEC_DRV_FAULT);
    }

    // {
    //     fault = true;
    // }
    // else
    // {
    //     fault = false;
    // }
}

/*Timer 4 Pwm for VREF STepper Driver*/
/*PB6 PB7*/

void SysTick_Handler(void) // Enter here every 1 ms
{
    usTicks++;
}

void init_clock()
{
    // Because the debugger switches PLL on, we may need to switch
    // back to the HSI oscillator before we can configure the PLL

    // Enable HSI oscillator
    SET_BIT(RCC->CR, RCC_CR_HSION);

    // Wait until HSI oscillator is ready
    while (!READ_BIT(RCC->CR, RCC_CR_HSIRDY))
    {
    }

    // Switch to HSI oscillator
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait until the switch is done
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
    {
    }

    // Disable the PLL
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until the PLL is fully stopped
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY))
    {
    }

    // Flash latency 2 wait states
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);
    // Enable HSE oscillator
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // Wait until HSE oscillator is ready
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY))
    {
    }

    // 72 MHz using the 8 MHz HSE oscillator with 9x PLL, lowspeed I/O runs at 36 MHz
    WRITE_REG(RCC->CFGR, RCC_CFGR_PLLSRC + RCC_CFGR_PLLMULL6 + RCC_CFGR_PPRE1_DIV1);

    // Enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until PLL is ready
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY))
    {
    }

    // Select PLL as clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    // Update variable
    // SystemCoreClock = 48000000;
    SystemCoreClockUpdate();

    // Disable the HSI oscillator
    // CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

void setup_timer(uint16_t hz)

{
    /*************>>>>>>> PID timer <<<<<<<<************/

    uint16_t *values = faktorize_frequenzy_computed(hz);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 Periph clock

    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ

    TIM2->ARR = values[1];     // 48000 - 1;     /* 4 MHz -> 1 KHz      */
    TIM2->PSC = values[0];     /* 4 Hz               */
    TIM2->DIER = TIM_DIER_UIE; /* interrupt on update */

    // Timer 3 channel 2 compare mode=PWM1 with the required preload buffer enabled
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE,
               TIM_CCMR1_OC2M_2 + TIM_CCMR1_OC2M_1 + TIM_CCMR1_OC2PE);
    TIM2->CCR2 = TIM2->ARR / 4;

    /*************>>>>>>> Position Fault timer <<<<<<<<************/
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // TIM4->SR &= ~TIM_SR_UIF;
    TIM4->ARR = 48000;
    TIM4->PSC = 3000;
    TIM4->CNT = 0;
    TIM4->CR1 |= TIM_CR1_OPM;
    TIM4->CR1 |= TIM_CR1_URS;

    TIM4->DIER |= TIM_DIER_UIE;
    // NVIC_EnableIRQ(TIM4_IRQn); // Enable IRQ
}

void TIM2_IRQHandler(void)
{
    // if(exec_state&STATE_OPEN_LOOP_MODE) pid();
    // if(exec_state&STATE_CLOSED_LOOP_MODE)
    _pid();
}

void TIM4_IRQHandler(void)
{
    TIM4->SR &= ~TIM_SR_UIF;
    // printf("%s\n", "control "); // Clean UIF Flag
}

uint16_t position()
{
    y = lookup[TIM1->CNT];

    if ((TIM1->SR & 1 << 0))
    {
        if ((y - y_1) < -(HALF_REVOLUTION))
            wrap_count += 1; // Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
        else if ((y - y_1) > (HALF_REVOLUTION))
            wrap_count -= 1;
        else
        {
            y_1 = y;
            TIM1->SR &= ~TIM_SR_UIF; // Clean UIF Flag
            return 1;
        }
    }
    yw = (y + ((HALF_REVOLUTION * 2) * wrap_count));
    y_1 = y;

    return 0;
}

void parse_message(char *input, int length)
{

    switch (input[0])
    {
    case 'A': // print
        printf("%c \n", 'a');
        break;
    case 'p': // print
     
       if (input[1] == 'e')
        {
            print_enc_position();
        }
        else if (input[1] == 'p')
        {
            print_setpoint();
        }
        else {
           print_position();
        }

        break;

    case 's': // step
        dir ? r++ : r--;
        setpoint = r;
        one_step();

        print_position();
        break;
    case 'i': // init steppers
        if (!(exec_state & STATE_INIT_STEPPER))
            init_stepper();
        set_state(STATE_INIT_STEPPER);
        // print_position();
        break;

    case 'd': // dir
        if (dir)
        {
            dir = false;
        }
        else
        {
            dir = true;
        }
        break;
    case 'c': // hybrid mode
        printf("%c \n", 'a');
        // calibrate();
        break;
    case '?':
        input[1] == '?' ? print_status() : report_status_message();

        break;
    case '!':
        input[1] == '!' ? print_alarm() : report_alarm_message();

        break;

    case 'q':
        parameterQuery();
        // reset_alarm();         //cal routine
        break;

    case 'e':;
        int f = atoi(&input[1]);
#ifdef DEBUG
        printf("PWM-frequenz: %d Hz \n", (uint16_t)f);
#endif
        if (exec_state & STATE_CLOSED_LOOP_MODE)
        {
            disable_pid_timer();
            clear_state(STATE_CLOSED_LOOP_MODE);
            set_state(STATE_OPEN_LOOP_MODE);
        }
        if (!(exec_state & STATE_INIT_STEPPER))
            init_stepper();
        set_step_pwm(f);
        break;

    case 'f':;
        Fs = atof(&input[1]);
        Ts = 1.0 / Fs;
#ifdef DEBUG
        printf("PID-frequenz: %d Hz \n", (uint16_t)Fs);
#endif
        setup_timer((uint16_t)Fs);
        break;

    case 'm':;
        int mi = atoi(&input[1]);
        set_microstep(mi);
        break;

    case 'r':; // new setpoint
        r = atol(&input[1]);
        set_setpoint(r);
        break;

    case 'x':
        enable_pid_timer();

        break;
    case 'y':
        v_ref_open_loop = atol(&input[1]);
        break;
    case 'n':
        NVIC_SystemReset();
        break;
    case 'v':
        enable_one_step();
        break;

    case 'u':; // anticogging
        uint16_t vref = atoi(&input[1]);
        if (exec_state & STATE_OPEN_LOOP_MODE)
            v_ref_open_loop = vref;

        set_vref_threshold(vref);
#ifdef DEBUG
        printf("VREF :%d\n", vref);
#endif
        break;

    case 'k':
        diagnostic();
        // parameterEditmain();
        break;

    case 'g':
        set_gains(&input[1], length);
        break;

    case 'h':
        store_data_to_flash();
        printf("\nsaved\n");

        break;

    case 'j':
        read_data_from_flash();
        break;

    case 'z':

        reset_position();
        break;
    case '~':
        reset_alarm();
        break;
    default:
        break;
    }
}

void serial_check()
{ // Monitors serial for commands.  Must be called in routinely in loop for serial interface to work.
    serialBuffer[bufferLength];

    if (rxAvailable())
    {
        uint16_t length = egets(serialBuffer, bufferLength);
        parse_message(serialBuffer, length);
    }
}
/*print calculated Postiton*/
void print_position()
{
    printf("[%d]\n", yw);
}
void print_setpoint()
{
    printf("[%d]\n", setpoint);
}
/*print raw position*/
void print_enc_position()
{
    printf("[%d]\n", TIM1->CNT);
}

void diagnostic()

{

    /*Check if encoder fits to steps*/
    /*first step Zero every thing*/
    printf("Start:%d \n", TIM1->CNT);
    // reset_position();
    printf("r:%d  y:%d  yw: %d  wc: %d\n", TIM1->CNT, y, yw, wrap_count);
    for (int i = 0; i < 3200; i++)
    {
        position();
        printf("r:%d  y:%d  yw: %d  wc: %d\n", TIM1->CNT, y, yw, wrap_count);
        one_step();
        delay_us(2999);
    }

    //     int minU = 20000, maxU = 0, minD = 20000, maxD = 0, D = 0;

    //     for (int i = 1; i <= 2000; i++)
    //     {

    //         if (i % 50 == 0)
    //         {
    //             uint8_t _e = (uint8_t)fabs(e);
    //             uint8_t _vref = constrain(_e, VREF_MIN, VREF_MAX);
    //             set_vref_pwm(_vref);

    //             // print_position();
    //         }
    //         D = r - yw;
    //         if (U < minU)
    //             minU = U;
    //         if (U > maxU)
    //             maxU = U;
    //         if (D < minD)
    //             minD = D;
    //         if (D > maxD)
    //             maxD = D;
    //         r = r + 1;
    //         delay_us(200);
    //     }
    // #ifdef DEBUG
    //     printf("minU: %d\tmaxU: %d\tminD: %d\tmaxD: %d\n", minU, maxU, minD, maxD);
    //     parameterQuery();
    // #endif
}

void delay_us(uint32_t dlyTicks)
{
    uint32_t curTicks;
    curTicks = usTicks;
    while ((usTicks - curTicks) < dlyTicks)
        ;
}

void asm_delay(int ticks)
{
    while (ticks-- > 0)
    {

        __asm("nop");
    }
}
uint16_t *faktorize_frequenzy_computed(uint16_t hz)
{
    static uint16_t data[2]; // return prescaler and frequeny as a array
    uint16_t arr = 70000;    // f equals the max vlaue of a unsigned 32 int.
    uint16_t psc = 0;
    if (hz < 734)
    {
        psc = lookup_frequencies[hz][0];
        arr = lookup_frequencies[hz][1];
    }
    else
    {
        psc = 1;
        arr = SystemCoreClock / hz;
    }
    data[0] = psc;
    data[1] = arr;
    return data;
}
uint16_t *faktorize_frequenzy(uint32_t hz)
{

    static uint32_t prescaler_frequenzy[2]; // return prescaler and frequeny as a array
    uint32_t f = 120000;                    // f equals the max vlaue of a unsigned 32 int.
    uint32_t sc_Clock;                      // sc_ Clock contains scaled down Core Clock vlaues, divided by prescaler
    uint32_t prescaler = 0;
    while (f > 65535 - 1)
    { // Tim->ARR is an 16 bit value, as long as it is bigger than an uint16 calclual

        prescaler = prescaler + 1;
        sc_Clock = SystemCoreClock / prescaler;
        f = sc_Clock / (hz);
        // printf("ppprescaler: %d  frequenzy %d \n", prescaler,(uint16_t)f);

        if (prescaler > 100000)
            break;
    }
    prescaler_frequenzy[0] = prescaler - 1;
    prescaler_frequenzy[1] = (uint32_t)f;

    return prescaler_frequenzy;
}

int *ctoi(char *input, int length)
{
    static int lVals[MAX_PARSE_VALUES]; // long values
    // parses the char * input with length
    // looks for comma separated longs
    // puts them in lVals and returns how many longs were found.

    char *pEnd;
    int v = 0; // counter for values
    char c;
    if (length < 1)
        return 0;
    // lVals[v] = strtol(&input[0], &pEnd, 10);	// get the first value

    for (int i = 0; i < length - 1; i++)
    { // length-1 because we always parse from the position after the comma: atof(input[i+1])
        c = input[i];
        if (c == ',' && v < MAX_PARSE_VALUES)
        {
            lVals[v] = strtol(&input[i + 1], &pEnd, 10);
            v++;
        }
    }
    return lVals;
}
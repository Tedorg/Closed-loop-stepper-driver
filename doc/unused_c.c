 /*************>>>>>>> PID timer <<<<<<<<************/

    uint16_t *values = faktorize_frequenzy_computed(hz);

    // RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  // Enable PORTC Periph clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 Periph clock
    // // Clear PC8 and PC9 control register bits
    // GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
    //                 GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    // Configure PC.8 and PC.9 as Push Pull output at max 10Mhz
    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ

    TIM2->ARR = values[1];     // 48000 - 1;     /* 4 MHz -> 1 KHz      */
    TIM2->PSC = values[0];     /* 4 Hz               */
    TIM2->DIER = TIM_DIER_UIE; /* interrupt on update */
                               // TIM2->CR1 = TIM_CR1_CEN;   /* enable */

    // Timer 3 channel 2 compare mode=PWM1 with the required preload buffer enabled
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE,
               TIM_CCMR1_OC2M_2 + TIM_CCMR1_OC2M_1 + TIM_CCMR1_OC2PE);
    TIM2->CCR2 = TIM2->ARR / 4;

    //    GPIOA->CRL = 0x22222222;    /* PA0-7 2MHz out      */
    //    NVIC_EnableIRQ(TIM6_IRQn);
    //    TIM6->PSC = 1 << 12;        /* 4 MHz -> 1 KHz      */
    //    TIM6->ARR = 1 << 8;         /* 4 Hz               */
    //    TIM6->DIER = TIM_DIER_UIE;  /* interrupt on update */
    //    TIM6->CR1 = TIM_CR1_CEN;    /* enable */

    /*************>>>>>>> Position Fault timer <<<<<<<<************/



// void position2()
// {

//     // max 65535
//     // whenn is wrap count negative? 
//     c_old: 100  wc: 0  c:20
//     positiv: c - c_old  

//     uint32_t c = TIM1->CNT;
//     int8_t sign = 1; // vorzeichen
//     // case CNT flipt below zero so its 4294967295 minus counts but wrap count is low
//     if (c > (4294967295 - 2147483648) && (wrap_count < 1000))
//     {
//         sign = -1;
//         c = (4294967295 - c) * wrap_quotient;
//         uint32_t remainder = c % (uint32_t)ENCODER_COUNTS_PER_REVOLUTION;

//         y = lookup[remainder];
//     }

//     yw = (y + (((MICRO_STEPS * STEPS_PER_REVOLUTION)) * wrap_count));

//     // rotate n by e bits, avoiding undefined behaviors
//     // cf https://blog.regehr.org/archives/1063
//     uint32_t rotr32(uint32_t n, uint32_t e)
//     {
//         return (n >> e) | (n << ((-e) & 31));
//     }
//     // does d divide n?
//     // d = 2**e * d_odd; dbar =
//  multiplicative_inverse(d_odd)
//  // thresh = 0xffffffff / d
//         bool gm_divisible(uint32_t n,uint32_t e, uint32_t dbar,uint32_t thresh)
//     {
//         return rotr32(n * dbar, e) <= thresh;
//     }
//     // Newton ’s method per Warren ,
//     // Hacker’s Delight pp. 246--247
//     uint32_t multiplicative_inverse(uint32_t d)
//     {
//         uint32_t x0 = d + 2 * ((d + 1) & 4);
//         uint32_t x1 = x0 * (2 - d * x0);
//         uint32_t x2 = x1 * (2 - d * x1);
//     }
// }


}

void set_vref_threshold(uint8_t target_vref_min)
{

	v_ref_min = constrain(target_vref_min, 0, 200);
}

void set_vref_pwm(uint8_t target_vref_step)
{

	if (current_vref_step != target_vref_step)
	{
		GPIOA->BSRR = GPIO_BSRR_BR15; // CS Pin is low

		// if(target_vref_step >100)return;
		uint16_t heading = abs(target_vref_step - current_vref_step);

		if (target_vref_step > current_vref_step)
		{
			GPIOA->BSRR = GPIO_BSRR_BS10; // set U/D Low
		}
		else
		{
			GPIOA->BSRR = GPIO_BSRR_BR10; // set U/D higf
		}

		for (uint8_t i = 0; i < heading; i++)
		{
			GPIOA->BSRR = GPIO_BSRR_BS11; // PA11 Hich to low
			asm_delay(100);
			GPIOA->BSRR = GPIO_BSRR_BR11; // PA11 Hich to low
			asm_delay(100);
		}
		current_vref_step = target_vref_step;
	}
}

void one_step()
{
	/************** One pulse mode configuration ************/
	/* One Pulse Mode selection */
	// TIM3->CR1 |= TIM_CR1_OPM;
	/********************************************************/
	// GPIOA->BSRR = GPIO_BSRR_BS6; // Set PA6 bit
	// asm_delay(15);
	// GPIOA->BSRR = GPIO_BSRR_BR6; // Rsegtet PA6 bit
	// 							 // 							 // step_count++;11,59775
	// 							 // 	TIM3->ARR = 50000; // 8000000/50000 = 160 pulses per second
	GPIOB->BSRR = GPIO_BSRR_BS8; // Set PA6 bit
	asm_delay(45);
	GPIOB->BSRR = GPIO_BSRR_BR8; // Rsegtet PA6 bit
	// 	// Timer 3 clock prescaler, the APB2 clock is divided by this value +1.
	// 	TIM3->PSC =11; // divide clock by 1

	// 	// Timer 3 enable counter and auto-preload
	// 	SET_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
	// 	TIM3->CCR1 = TIM3->ARR / 2;
	// set_step_pwm(100);
	//User only wants one pulse (Single mode), so write '1 in the OPM bit in the TIMx_CR1
	//TIM3->CR1 |= TIM_CR1_OPM;
	//TIM3->CR1 &= ~TIM_CR1_OPM;

}
void disable_step_pwm(void)
{
	CLEAR_BIT(GPIOA->CRL, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);
	//CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
	// Enable timer 3
	// CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

	// Timer 3 channel 1 compare mode = PWM1 with the required preload buffer enabled
	// CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M);
	// CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);
	//CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_2);
	//CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_1);
	// CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);
	// Timer 3 enable compare output
	//CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E);
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);
	set_step_pwm(0);
	 GPIOA->BSRR = GPIO_BRR_BR6;
}

void init_step_pwm()
{
	/* TIM3_CH1:PA6, TIM3_CH2: PA7, TIM3_CH3: PB0, TIM3_CH4: PB1 */
	// PA6 = Timer 3 channel 1 alternate function output
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6 + GPIO_CRL_MODE6, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);
    //  GPIOA->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6;
    //GPIOA->CRL &= ~(GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0);
SET_BIT(GPIOA->CRL , GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);

	// Enable timer 3
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

	// Timer 3 channel 1 compare mode = PWM1 with the required preload buffer enabled
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M + TIM_CCMR1_OC1PE,
			   TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);
	// Timer 3 enable compare output
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);
}

void set_step_pwm(uint32_t frequenzy)
{

	// if (exec_state & STATE_OPEN_LOOP_MODE)
	// {
	// 	// Timer 3 enable counter and auto-preload
	// 	SET_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
	// 	TIM3->CCR1 = TIM3->ARR / 2;
	// 	// TIM3->CCR2 = TIM3->ARR / 4;
	// 	//  TIM3->CCR3 = 4000;
	// 	//  TIM3->CCR4 = 40000;
	// }

	if (frequenzy == 0)
	{
		// CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
		CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
	}

	if (frequenzy < STEP_PWM_MIN_HZ || frequenzy > STEP_PWM_MAX_HZ)
		return;

	uint16_t *values = faktorize_frequenzy_computed(frequenzy);
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
	if (values)
	{

		TIM3->ARR = values[1]; // 8000000/50000 = 160 pulses per second

		// Timer 3 clock prescaler, the APB2 clock is divided by this value +1.
		TIM3->PSC = values[0]; // divide clock by 1

		// Timer 3 enable counter and auto-preload
		SET_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
		TIM3->CCR1 = TIM3->ARR / 2;
		// TIM3->CCR2 = TIM3->ARR / 4;
		//  TIM3->CCR3 = 4000;
		//  TIM3->CCR4 = 40000;
	}
}

void enable_one_step()
{
	if (exec_state & STATE_CLOSED_LOOP_MODE)
	{
	disable_step_pwm();
	clear_state(STATE_CLOSED_LOOP_MODE);
	}

	// if (!(exec_state & STATE_OPEN_LOOP_MODE))
	// {
		set_state(STATE_OPEN_LOOP_MODE);
	// 	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);
	// // Timer 3 enable compare output
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E);
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);

	// 	CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);

	// 	  GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;
	// 	 GPIOA->CRL &= ~(GPIO_CRL_CNF6_0 | GPIO_CRL_MODE6_1);
			
	

}

ONEPULSE MODE
	
// SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
// MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M + TIM_CCMR1_OC1PE,
// 			   TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);
// 			   	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);
// 	SET_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);
// 	//Map TI2FP2 on TI2 by writing CC2S=01 in the TIMx_CCMR1 register.
// 		TIM3->CCMR1 |= TIM_CCMR1_CC2S_1;
// 		TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S_0);
//  	//TI2FP2 must detect a rising edge, write CC2P=0 in the TIMx_CCER register.
// 		 TIM3->CCER &= ~(TIM_CCER_CC2P);
// 	//Configure TI2FP2 as trigger for the slave mode controller (TRGI) by writing TS=110 in the TIMx_SMCR register.
// //TI2FP2 is used to start the counter by writing SMS to ‘110 in the TIMx_SMCR register (trigger mode).
// 		 TIM3->SMCR |= TIM_SMCR_TS_0 |TIM_SMCR_TS_1;
// 		 TIM3->SMCR &= ~TIM_SMCR_TS_2;
// 		 TIM3->CCR1 = 500;
// 		TIM3->ARR = 1000;
// 		// MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M,
// 		TIM3->CCMR1	|= TIM_CCMR1_OC1M_2 |TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);

	// Timer 3 enable counter d
	// CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
	/* ENABLE PA6 as output for STEP TO DRV */




reaching
perl -e 'for my $a (0..127) {
     printf "page %d 0x%x \n",$a, $a*1024 + 0x8000000;
           
 }'
 
  printf "%+6.0f,", sin($a * 3.1415 / 180 / 10.0)*1024;
           print "\n" if $a % 10 == 9;
           
           
           

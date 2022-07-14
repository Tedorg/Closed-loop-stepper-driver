#include <stdio.h>
#include <string.h>

#include "stm32f1xx.h"
#include "parameter.h"
#include "protocol.h"
#include "util.h"
#include "state.h"
#include "stepper.h"

void init_stepper()
{
	init_vref();
	sleep_stepper(false);
	set_vref_pwm(VREF_MIN);
	set_microstep(MICRO_STEPS);
	enable_stepper(true);
	set_dir(dir);
	set_state(STATE_INIT_STEPPER);
}

void get_dir()
{

	if (GPIOA->IDR & (0x1))
	{
		dir = true;
	}
	else
	{
		dir = false;
	}
}

void set_dir(bool _dir)
{
	if (_dir)
	{
		GPIOA->BSRR = GPIO_BSRR_BS4; // Set PA4 bit
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BR4; // reset PA4 bit
	}
}

void enable_stepper(bool on)
{
	if (on)
	{
		GPIOB->BSRR = GPIO_BSRR_BR11; // Set PA4 bit
	}
	else
	{
		GPIOB->BSRR = GPIO_BSRR_BS11; // reset PA4 bit
	}
}
void sleep_stepper(bool on)
{
	// sleep pin High to enable
	if (on)
	{
		GPIOA->BSRR = GPIO_BSRR_BR5; // Set PA5 bit
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BS5; // reset PA4 bit
	}
}

void set_microstep(byte mode)
{
	// sleep pin High to enable

	// 000  Full
	// 100  half
	// 010  1/4
	// 110  1/8
	// 001  1/16
	// 101  1/32
	switch (mode)
	{
	case 1:
		set_microstep_pins(0x0); // full 000
		break;

	case 2:
		set_microstep_pins(0x4); // half 100
		break;

	case 4:
		set_microstep_pins(0x2); // 1/4  010
		break;

	case 8:
		set_microstep_pins(0x6); // 1/8  110
		break;

	case 16:
		set_microstep_pins(0x1); // 1/16  001
		break;

	case 32:
		set_microstep_pins(0x5); // 1/32  101
		break;

	default:
		printf("%s \n", "not supported microstep mode");
	}
}

void set_microstep_pins(byte code)
{
	GPIOB->BSRR = GPIO_BSRR_BR0;  // Rsegtet PB0 bit
	GPIOB->BSRR = GPIO_BSRR_BR1;  // Rsegtet PB1 bit
	GPIOB->BSRR = GPIO_BSRR_BR10; // Rsegtet PB11 bit
	if (code & (1 << 0))
		GPIOB->BSRR = GPIO_BSRR_BS0; // set PB0 bit
	if (code & (1 << 1))
		GPIOB->BSRR = GPIO_BSRR_BS1; // set PB1 bit
	if (code & (1 << 2))
		GPIOB->BSRR = GPIO_BSRR_BS10; // set PB1 bit
}
void init_vref()
{
	set_vref_pwm(100);
	set_vref_pwm(0);
	set_vref_pwm(100);
	set_vref_pwm(0);
	set_vref_pwm(100);
	set_vref_pwm(0);
	current_vref_step = 0;
}
void store_vref()
{

	/*Digitalpotentiometer 10 kOhm DIL
	make sure DP starts with a low value
	U/D -> High increment up
	Wiper position of the DS1804 can be stored using the INC and CS inputs.
	Storage of the wiper position takes place whenever the CS input
	transitions from low-to-high while the INC is high.
	Once this condition has occurred the value of the current wiper
	position will be written to EEPROM memory.
	 */

	GPIOB->BSRR = GPIO_BSRR_BR15; // CS Pin is low; // CS Pin is low
	GPIOA->BSRR = GPIO_BSRR_BR10; // U/D is Low
	GPIOA->BSRR = GPIO_BSRR_BR11; // INC is low
	asm_delay(1000);
	GPIOA->BSRR = GPIO_BSRR_BS11; // INC is HIGH
	asm_delay(1000);			  // spme time to settle
	GPIOB->BSRR = GPIO_BSRR_BS15; // PC Pin is HIGH
	asm_delay(1000);
	GPIOB->BSRR = GPIO_BSRR_BR15; // PC Pin is HIGH

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
	set_step_pwm(0);
	GPIOA->BSRR = GPIO_BRR_BR6;
}

void init_step_pwm()
{
	/* TIM3_CH1:PA6, TIM3_CH2: PA7, TIM3_CH3: PB0, TIM3_CH4: PB1 */
	// PA6 = Timer 3 channel 1 alternate function output
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6 + GPIO_CRL_MODE6, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);
   
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
		
	}
}

void enable_one_step()
{
	if (exec_state & STATE_CLOSED_LOOP_MODE)
	{
	disable_step_pwm();
	clear_state(STATE_CLOSED_LOOP_MODE);
	}

	 if (!(exec_state & STATE_OPEN_LOOP_MODE)){

set_state(STATE_OPEN_LOOP_MODE);
  setup_timer(450);
       init_pid();
	    TIM2->CR1 = TIM_CR1_CEN; /* enable  */

	 }
	// {
		
	// 	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);
	// // Timer 3 enable compare output
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E);
	// CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);

	// 	CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);

	// 	  GPIOA->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1;
	// 	 GPIOA->CRL &= ~(GPIO_CRL_CNF6_0 | GPIO_CRL_MODE6_1);
			
	

}

void init_hardware_decoder_1(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // AFIO might not even be needed?

	// GPIO must be input floating which is default so no code to write for that

	// value to count up to : 16 bit so max is 0xFFFF = 65535
	TIM4->ARR = 0xFFFF;

	// per datasheet instructions
	TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0); // step 1 and 2
	TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);		  // step 3 and 4
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;		  // step 5
	TIM4->CNT = 0;										  // reset  counter
	TIM4->CR1 |= TIM_CR1_CEN;							  // step 6
}

void init_hardware_decoder_2(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // AFIO might not even be needed?

	// GPIO must be input floating which is default so no code to write for that

	// value to count up to : 16 bit so max is 0xFFFF = 65535
	// T1C1 PA8
	// T1C2 PA9
	TIM1->ARR = 4000 - 1; // its 0 Indexed
	// TIM1->ARR =65536-1;  //its 0 Indexed

	// per datasheet instructions
	TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0); // step 1 and 2
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);		  // step 3 and 4
	TIM1->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;		  // step 5
	TIM1->CNT = 0;										  // reset  counter
	TIM1->CR1 |= TIM_CR1_CEN;							  // step 6
														  //  TIM1->CR1 |= TIM_CR1_URS;
}
void reset_position(void)
{
	// Disable closed loop mode
	if (exec_state & STATE_CLOSED_LOOP_MODE)
	{
		disable_pid_timer();
		clear_state(STATE_CLOSED_LOOP_MODE);
	}

	TIM1->CNT = 0;
	wrap_count = 0;
	yw = 0;
	y = 0;
	setpoint = 0;
	r = 0;
	set_setpoint(0);
	// enable_pid_timer();
	// set_state(STATE_CLOSED_LOOP_MODE);
}

void calibrate()
{ /// this is the calibration routine

	int encoderReading = 0; // or float?  not sure if we can average for more res?
	int currentencoderReading = 0;
	int lastencoderReading = 0;
	int avg = 30; // how many readings to average

	uint32_t fullStepReadings[STEPS_PER_REVOLUTION + 1];
	uint32_t microStepReadings[STEPS_PER_REVOLUTION * MICRO_STEPS + 1];

	// memset(microStepReadings,0,sizeof(microStepReadings));

	int ticks = 0;
	init_step_pwm();
	disable_step_pwm();
	set_microstep(1);
	// init_vref();
	// sleep_stepper(0);
	set_vref_pwm(18);
	// set_microstep(1);
	// enable_stepper(1);
	enable_one_step();
	// one_step();
	encoderReading = TIM1->CNT;
	delay_us(1000);

	dir = true;
	set_dir(dir);
	one_step();
	printf("posA %ld\n", TIM1->CNT); // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...

	if ((TIM1->CNT) < 0) // check which way motor moves when dir = true
	{
		printf("Wired backwards: %ld\n", TIM1->CNT - encoderReading); // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
		return;
	}

	TIM1->CNT = 0;
	delay_us(1000);
	printf("idx  soll  ist\n");
	for (int x = 0; x < STEPS_PER_REVOLUTION; x++)
	{ // step through all full step positions, recording their encoder readings

		encoderReading = 0;
		delay_us(2900); // moving too fast may not give accurate readings.  Motor needs time to settle after each step.
		// lastencoderReading = TIM1->CNT;

		for (int reading = 0; reading < avg; reading++)
		{ // average multple readings at each step
			currentencoderReading = TIM1->CNT;

			if ((currentencoderReading - lastencoderReading) < (-(ENCODER_COUNTS_PER_REVOLUTION / 2)))
			{
				currentencoderReading += ENCODER_COUNTS_PER_REVOLUTION;
			}
			else if ((currentencoderReading - lastencoderReading) > ((ENCODER_COUNTS_PER_REVOLUTION / 2)))
			{
				currentencoderReading -= ENCODER_COUNTS_PER_REVOLUTION;
			}

			encoderReading += currentencoderReading;
			delay_us(8000);
		}
		encoderReading = encoderReading / avg;
		if (encoderReading > ENCODER_COUNTS_PER_REVOLUTION)
		{
			encoderReading -= ENCODER_COUNTS_PER_REVOLUTION;
		}
		else if (encoderReading < 0)
		{
			encoderReading += ENCODER_COUNTS_PER_REVOLUTION;
		}

		fullStepReadings[x] = encoderReading;

		printf("%d,%ld,%d\n", x, x * 20, fullStepReadings[x]);
		lastencoderReading = encoderReading;
		one_step();
	}
	fullStepReadings[STEPS_PER_REVOLUTION] = 4000; // LAst Encoder Read is a  Dummy, since for loop opnly count until 200 -1
	printf("%d,%ld,%d\n", STEPS_PER_REVOLUTION, fullStepReadings[STEPS_PER_REVOLUTION], STEPS_PER_REVOLUTION * 20);

	// sectorSizeEncoder = 4000/(int)STEPS_PER_REVOLUTION;
	//  sectorSizeSteps = (float)MICRO_STEPS;
	//  int ticksSum = 0;

	/*
	 for (int i = 0; i <STEPS_PER_REVOLUTION; i++)
	{
		ticks = fullStepReadings[i + 1] - fullStepReadings[i];
		//printf("ticks: %ld:  \n",ticks);      //print readings as a sanity check

		float steps = (float)MICRO_STEPS/ticks;

		for(int j = 0; j<ticks;j++){
			printf(",%d ",(int)(steps*j)+(i*(int)MICRO_STEPS));      //print readings as a sanity check
			if(step_count%10 == 0)printf("\n ");      //print readings as a sanity check
			step_count++;
			if(step_count > 4000)break;
		}

	}
	printf("\ncount: %ld   \n",step_count);      //print readings as a sanity check
 */

	// 	float steps = (int)MICRO_STEPS/(float)ticks;
	// 	ticksSum += ticks;
	// 	printf("f: %ld: enc: %ld  ticks %ld  ticksSum: %ld  steps: %f   -----\n",step_count,fullStepReadings[i],ticks, ticksSum,steps);      //print readings as a sanity check
	// 	step_count++;

	// 	for(int j = 0; j<ticks;j++){

	// 		printf("m: %ld: %ld \n",step_count,(int)(steps*j)+fullStepReadings[i]);
	// 		step_count++;
	// 	}

	// 	}
}

// 	if (x % 20 == 0)
// 	{
// 		SerialUSB.println();
// 		SerialUSB.print(100 * x / spr);
// 		SerialUSB.print("% ");
// 	}
// 	else
// 	{
// 		SerialUSB.print('.');
// 	}

// 	oneStep();
// }
// SerialUSB.println();

// // SerialUSB.println(" ");
// // SerialUSB.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
// // SerialUSB.println(" ");/
// for (int i = 0; i < spr; i++)
// {
// 	ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
// 	if (ticks < -15000)
// 	{
// 		ticks += cpr;
// 	}
// 	else if (ticks > 15000)
// 	{
// 		ticks -= cpr;
// 	}
// 	// SerialUSB.println(ticks);

// 	if (ticks > 1)
// 	{ //note starting point with iStart,jStart
// 		for (int j = 0; j < ticks; j++)
// 		{
// 			stepNo = (mod(fullStepReadings[i] + j, cpr));
// 			// SerialUSB.println(stepNo);
// 			if (stepNo == 0)
// 			{
// 				iStart = i;
// 				jStart = j;
// 			}
// 		}
// 	}

// 	if (ticks < 1)
// 	{ //note starting point with iStart,jStart
// 		for (int j = -ticks; j > 0; j--)
// 		{
// 			stepNo = (mod(fullStepReadings[spr - 1 - i] + j, cpr));
// 			// SerialUSB.println(stepNo);
// 			if (stepNo == 0)
// 			{
// 				iStart = i;
// 				jStart = j;
// 			}
// 		}
// 	}
// }

// // The code below generates the lookup table by intepolating between
// // full steps and mapping each encoder count to a calibrated angle
// // The lookup table is too big to store in volatile memory,
// // so we must generate and store it into the flash on the fly

// // begin the write to the calibration table
// page_count = 0;
// page_ptr = (const uint8_t *)lookup;
// SerialUSB.print("Writing to flash 0x");
// SerialUSB.print((uintptr_t)page_ptr, HEX);
// SerialUSB.print(" page size PSZ=");
// SerialUSB.print(NVMCTRL->PARAM.bit.PSZ);

// for (int i = iStart; i < (iStart + spr + 1); i++)
// {
// 	ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

// 	if (ticks < -15000)
// 	{ //check if current interval wraps over encoder's zero positon
// 		ticks += cpr;
// 	}
// 	else if (ticks > 15000)
// 	{
// 		ticks -= cpr;
// 	}
// 	//Here we print an interpolated angle corresponding to each encoder count (in order)
// 	if (ticks > 1)
// 	{ //if encoder counts were increasing during cal routine...

// 		if (i == iStart)
// 		{ //this is an edge case
// 			for (int j = jStart; j < ticks; j++)
// 			{
// 				store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j) / float(ticks))), 360000.0));
// 			}
// 		}

// 		else if (i == (iStart + spr))
// 		{ //this is an edge case
// 			for (int j = 0; j < jStart; j++)
// 			{
// 				store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j) / float(ticks))), 360000.0));
// 			}
// 		}
// 		else
// 		{ //this is the general case
// 			for (int j = 0; j < ticks; j++)
// 			{
// 				store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j) / float(ticks))), 360000.0));
// 			}
// 		}
// 	}

// 	else if (ticks < 1)
// 	{ //similar to above... for case when encoder counts were decreasing during cal routine
// 		if (i == iStart)
// 		{
// 			for (int j = -ticks; j > (jStart); j--)
// 			{
// 				store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
// 			}
// 		}
// 		else if (i == iStart + spr)
// 		{
// 			for (int j = jStart; j > 0; j--)
// 			{
// 				store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
// 			}
// 		}
// 		else
// 		{
// 			for (int j = -ticks; j > 0; j--)
// 			{
// 				store_lookup(0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0));
// 			}
// 		}
// 	}
// }

// if (page_count != 0)
// 	write_page();

// SerialUSB.println(" ");
// SerialUSB.println(" ");
// SerialUSB.println("Calibration complete!");
// SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
// SerialUSB.println(" ");
// SerialUSB.println(" ");

// void init_vref_pwm()
// {
//     // PA6 = Timer 3 channel 1 alternate function output
//     MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF6 + GPIO_CRL_MODE6, GPIO_CRL_CNF6_1 + GPIO_CRL_MODE6_0);
//     // Enable timer 4
//     SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
//     // Timer 4 channel 1 compare mode = PWM1 with the required preload buffer enabled
//     MODIFY_REG(TIM4->CCMR1, TIM_CCMR1_OC1M + TIM_CCMR1_OC1PE,
//                TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);
//     // Timer 4 enable all four compare outputs
//     SET_BIT(TIM4->CCER, TIM_CCER_CC1E + TIM_CCER_CC2E + TIM_CCER_CC3E + TIM_CCER_CC4E);
// }
/*

void enable_vref_pwm(float pulse_width)
{
	float p = (pulse_width/100.0)*(float)(VREF_MAX_COUNTER_VALUE-VREF_MIN_COUNTER_VALUE);
uint16_t _mod= constrain( p,(int)0,(int)VREF_MAX_COUNTER_VALUE)+ VREF_MIN_COUNTER_VALUE;

 printf("percent: %f  pulse: %d\n",  p,_mod);
	TIM4->ARR = VREF_ARR_VALUE; // 8000000/50000 = 160 pulses per second

	// Timer 3 clock prescaler, the APB2 clock is divided by this value +1.
	TIM4->PSC = 10; // divide clock by 1

	// Timer 3 enable counter and auto-preload
	SET_BIT(TIM4->CR1, TIM_CR1_CEN + TIM_CR1_ARPE);
	TIM4->CCR1 = _mod;


}

void disable_vref_pwm(void)
{
	CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);
	GPIOB->BSRR = GPIO_BRR_BR6;
} */
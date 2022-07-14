

#include <stdio.h>
#include <math.h>

#include "stm32f1xx.h"

#include "parameter.h"
#include "protocol.h"

#include "util.h"
#include "stepper.h"
#include "state.h"

void enable_pid_timer()
{
    if (!(exec_state & STATE_INIT_STEPPER))
        init_stepper();

    if (exec_state & STATE_IDLE)
        clear_state(STATE_IDLE);
    if (exec_state & STATE_OPEN_LOOP_MODE)clear_state(STATE_OPEN_LOOP_MODE);

    if (!(exec_state & STATE_CLOSED_LOOP_MODE))
    {
          set_state(STATE_CLOSED_LOOP_MODE);
        // position();
      init_step_pwm();
      // set_setpoint(yw);
        setup_timer(450);
       init_pid();
        
        TIM2->CR1 = TIM_CR1_CEN; /* enable  */
     
       // clear_state(STATE_OPEN_LOOP_MODE);
    }
}

void disable_pid_timer()
{
   
       // disable_step_pwm();
        //set_step_pwm(0);
       TIM2->CR1 &= ~TIM_CR1_CEN; /* disable */
      // disable_step_pwm();
       enable_one_step();
    
    if (exec_state & ~STATE_IDLE)
        set_state(STATE_IDLE);
}
/*************>>>>>>> PID IRQ <<<<<<<<************/

// void resetActivityCounter() { activityCount = 0; }
/// Set the number of seconds after which the motor is turned off, zero to
/// keep it on indefinitely.
// void setActivityTimeout(float s) {
//         if (s == 0)
//             activityThres = 0;
//         else
//             activityThres = uint16_t(s / Ts) == 0 ? 1 : s / Ts;
// }
void set_setpoint(int t)
{
    setpoint = t;
    resetActivityCounter();
   resetIntegral();
    u_1 = 0;
    u_d = 0;
    // if (exec_state & STATE_OPEN_LOOP_MODE){
    //     for (int i = 0; i < setpoint; i++){
    //         one_step();
    //         delay_us(1000);
    //     }
    // }
}


void _pid()
{
    


    if (position())
        activityCount++;
    if (_busy)
    {
        ++_busy;
    }
    else
    {
        if (exec_state & STATE_CLOSED_LOOP_MODE){

        _busy = 1;
        e = (double)(setpoint - yw);

        if (!(exec_state & STATE_REACHED_TARGET))
        {
            ITerm += (e); // Integral wind up limit
            if (ITerm > 200)
                ITerm = 200;
            else if (ITerm < -200)
                ITerm = -200;
        }

        DTerm = (double)(yw - yw_1);

        u = (pKp * e) + (pKi * ITerm) + (pKd * DTerm);
        // uint8_t _vref = constrain(abs((uint8_t)e),VREF_MIN,VREF_MAX);
        // set_vref_pwm(_vref);
        //  if(u != u_1) printf("e: %d  u: %d \n",(int)e,(int)u);

        if (u < 0)
        {
            set_dir(false);
            u = u*-1;
            
        }
        else
        {
            set_dir(true);
        }

        if (fabs(e) < e_t)
        {
            if (ITerm < -0.2)
                ITerm += 0.1f;
            if (ITerm > 0.2)
                ITerm -= 0.1f;
            set_led_2(true);
            U = 0;
            set_step_pwm(U);

            if (exec_state & ~STATE_REACHED_TARGET)
                set_state(STATE_REACHED_TARGET);
        }
        else
        {
            if (exec_state & STATE_REACHED_TARGET)
                clear_state(STATE_REACHED_TARGET);
            if (abs(yw_1 - yw) > 10)
            {

                u_d = constrain(u_d - 500, 0, (STEP_PWM_MAX_HZ / 2));
            }
            else
            {
                u_d = constrain(u_d + 500, 0, (STEP_PWM_MAX_HZ / 2)); // this means stepper is not moving, so reduce pwm frequenzy
            }
            U = constrain(u, STEP_PWM_MIN_HZ, STEP_PWM_MAX_HZ);
            U = constrain((U-u_d), STEP_PWM_MIN_HZ, STEP_PWM_MAX_HZ);
            set_step_pwm(U);
            set_led_2(false);
        }
        // set_led_2(toggler);
        
       
       
    }
    }
     u_1 = u;

        e_1 = e;
        yw_1 = yw;
     _busy = 0;
     TIM2->SR &= ~TIM_SR_UIF; // Clean UIF Flag
    

}

void init_pid(void)
{
    resetActivityCounter();
    resetIntegral();
}
uint16_t pid_2()
{

    position();

    // The error is the difference between the reference (setpoint) and the
    // actual position (input)
    int32_t error = setpoint - yw;
    e = (double)error;
    // The integral or sum of current and previous errors
    int32_t newIntegral = integral + error;
    // Compute the difference between the current and the previous input,
    // but compute a weighted average using a factor α ∊ (0,1]
    float diff = emaAlpha * (prevInput - yw);
    // Update the average
    prevInput -= diff;

    // Check if we can turn off the motor
    if (activityCount >= activityThres && activityThres)
    {
        float filtError = setpoint - prevInput;
        if (filtError >= -errThres && filtError <= errThres)
        {
            errThres = 2; // hysteresis
            integral = newIntegral;
            return 0;
        }
        else
        {
            errThres = 1;
        }
    }
    else
    {
        ++activityCount;
        errThres = 1;
    }

    bool backward = false;
    int32_t calcIntegral = backward ? newIntegral : integral;

    // Standard PID rule
    ITerm = calcIntegral;
    DTerm = diff;

    float output = pKp * error + pKi * calcIntegral + pKd * diff;

    if (output < 0)
    {
        set_dir(false);
    }
    else
    {
        set_dir(true);
    }
    // U = constrain(fabs(u), STEP_PWM_MIN_HZ, STEP_PWM_MAX_HZ);
    int pwm = abs(output);

    // Clamp and anti-windup
    if (pwm > STEP_PWM_MAX_HZ)
    {
        pwm = STEP_PWM_MAX_HZ;
    }
    else if (pwm < STEP_PWM_MIN_HZ)
    {
        pwm = STEP_PWM_MIN_HZ;
    }
    else
    {
        integral = newIntegral;
    }

    u = pwm;
    yw_1 = yw;
    y_1 = y;
    TIM2->SR &= ~TIM_SR_UIF; // Clean UIF Flag

    return pwm;
}

/// Reset the activity counter to prevent the motor from turning off.
void resetActivityCounter() { activityCount = 0; }
/// Set the number of seconds after which the motor is turned off, zero to
/// keep it on indefinitely.
void resetIntegral()
{
    integral = 0;
    ITerm = 0;
}

/// Set the cutoff frequency (-3 dB point) of the exponential moving average
/// filter that is applied to the input before taking the difference for
/// computing the derivative term.
void setEMACutoff(float f_c)
{
    Ts = 1 / Fs;
    float f_n = f_c * Ts; // normalized sampling frequency
    emaAlpha = f_c == 0 ? 1 : calcAlphaEMA(f_n);
}
float calcAlphaEMA(float fn)
{
    // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    const float c = cos(2 * (float)(M_PI)*fn);
    return c - 1 + sqrt(c * c - 4 * c + 3);
}

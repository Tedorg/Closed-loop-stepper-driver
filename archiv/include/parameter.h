#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#define firmware_version "0.0.1"    //firmware version
#define identifier "x"              // change this to help keep track of multiple mechaduinos (printed on startup)

#include "stm32f1xx.h"


 
    #define LED_BUILD_IN (13)  //Build in LED at PC13 only for Debugging
    #define DIR_TO_DRIVER_PIN (5)   //PA5
    #define PWM_TO_DRIVER_PIN (6)   //PA6
    #define PWM_CURRENT_TO_DRIVER_PIN (7)   //PA7 Dynamically adjust Torque at DRV8825
   
   
    #define STEP_PIN (1)   //PA1 Input Steps ->Interrupt gets Signal from MASTER board
    #define DIR_PIN (0)    //PA0 Input Dir    ->  gets Signal from MASTER board
    #define ERROR_FROM_DRIVER_PIN (6)    //BA0 Input Error    ->  gets Error from Driver
    










#endif

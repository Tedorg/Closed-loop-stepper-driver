Commands Serial (baudrate 115200)

 s  -  step
 d  -  dir


 c  -  calibration routine - todo
 a  -  aknowledge incoming char - driver ready
 e  -  enable fix speed in Hz
 q  -  parameter query
 p  -  print position (encoder)
 pp -  print setpoint
 pe -  print raw encoder (Debuggin)
 

 x  -  set Closed loop
 v  -  set Open Llop


 y  - y<vref> set fixed vref - for open loop
 u  - u<vref> set minimum vref - closed loop

 n  -  software reset stm32
 r  -  enter new setpoint
 z  -  zeroing Position
 ~  -  reset alarm

 
 k  -  Diagnsotics - todo 
 ?  -  report Status uint8
 ??  - report Status string
 !  -  report Alarm uint8
 !! -  report Alarm string
 g  -  gp<proportianl gain> gd<differential gain> gi<integral gain> ge<error tolerance> gc<cut-off frequenzy> -- for PID Kp,Kd,Ki, error tol, cut off frequenzy (float)
 h  -  store gains to flash
 j  -  read gains from flash
 f  -  f<frequenz>  pid Frequenz
 m  -  m<microstep> 1,2,4,8,16,32


Fehlercodes:

EXEC_STATUS_REPORT      BIT0 // bitmask 00000001
EXEC_DRV_FAULT          BIT1 // bitmask 00000010
EXEC_UNREACH_TARGET     BIT2 // bitmask 00000100
EXEC_FEED_HOLD          BIT3 // bitmask 00001000
EXEC_RESET              BIT4 // bitmask 00010000
EXEC_DRV_DISABLED       BIT5 // bitmask 00100000
EXEC_MOTION_CANCEL      BIT6 // bitmask 01000000
EXEC_SLEEP              BIT7 // bitmask 10000000



Statuscodes:
STATE_STATUS_REPORT      BIT0 // bitmask 00000001
STATE_REACHING_TARGET    BIT1 // bitmask 00000010
STATE_REACHED_TARGET     BIT2 // bitmask 00000100
STATE_IDLE               BIT3 // bitmask 00001000
STATE_CONTROL_MODE       BIT4 // bitmask 00010000
STATE_CLOSED_LOOP_MODE   BIT5 // bitmask 00100000
STATE_RESERVED_1         BIT6 // bitmask 01000000
STATE_RESERVED_2         BIT7 // bitmask 10000000



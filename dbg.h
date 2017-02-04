#if !defined(__DBG_H__)
#define __DBG_H__
#include <Arduino.h>



#define DEBUG_OFF
//#define DEBUG_ON


#ifdef DEBUG_ON
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUD_RATE 115200

#define dbg(v)		 DEBUG_SERIAL.println(v);	
#define dbg2(t,v)	 DEBUG_SERIAL.print(F(t));DEBUG_SERIAL.println(v);  
// #define dbg2(t,cha)    SERIAL_PC.print(t);SERIAL_PC.println(cha);//  SERIAL_SPEAK.print(t);SERIAL_SPEAK.println(cha);  

#else
#define dbg(cha)
#define dbg2(t,cha)	

#endif // DEBUG_ON

#endif // 0

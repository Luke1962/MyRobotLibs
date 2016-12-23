// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       MACRO PER INTERFACCIARSI CON IL MODULO VOCE									///
// ///																						///
// ///																						///
// ////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(__SPEAK_H__)
	#define __SPEAK_H__
	#define SPEACHDELAY 700
	#include <Arduino.h>
	#include "systemConfig.h"
	#include "Commands_Enum.h"

	#if defined OPT_SPEECH 
		//#define SERIAL_MMI_SPEED 115200
		//#define SERIAL_MMI 
		#define SPEAKCMD SERIAL_MMI.print(cmdSpeech); SERIAL_MMI.print(",");
		#define SPEAK_CIAO			SPEAKCMD	SERIAL_MMI.print("h;"); 			
		#define SPEAK_OIOI			SPEAKCMD	SERIAL_MMI.print("O;"); 			
		#define SPEAK_CIAOCHISEI	SPEAKCMD	SERIAL_MMI.print("ciao  ki sei;"); 	
		#define SPEAK_OK			SPEAKCMD	SERIAL_MMI.print("k;");				
		#define SPEAK_TEST			SPEAKCMD	SERIAL_MMI.print("t;");				
		#define SPEAK_MODE_SLAVE			SPEAKCMD	SERIAL_MMI.print("SLEIV;");			
		#define SPEAK_AUTONOMO		SPEAKCMD	SERIAL_MMI.print("AUTONOMO;");	
		#define SPEAK(s)			SPEAKCMD	SERIAL_MMI.print(s);SERIAL_MMI.print(";");
	#else 
		#define SPEAK_CIAO  /*SERIAL_MMI.print("h");*/
		#define SPEAK_OK /*SERIAL_MMI.print("k");*/
		#define SPEAK_TEST /*SERIAL_MMI.print("t");*/
		#define SPEAK_MODE_SLAVE		/*SERIAL_MMI.print("MODE_SLAVE");*/		
		#define SPEAK_AUTONOMO	/*	SERIAL_MMI.print("AUTONOMO"); */
		#define SPEAK(s) /* SERIAL_MMI.print(s);*/
	#endif
#endif
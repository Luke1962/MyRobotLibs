#ifndef __SpeakSerialInterface_h__
#define __SpeakSerialInterface_h__

#pragma region SPEAK
#include <ChibiOS_AVR/ChibiOS_AVR.h>
#include <hwMMI_config.h>
#include <MyRobotLibs\dbg.h>
/// da inserire nelle dir di ricerca 
///C:\Program Files %28x86%29\Arduino\hardware\arduino\avr\libraries\SoftwareSerial
#include <SoftwareSerial.h>		

SoftwareSerial SwSerialSpeech(42, Pin_SpeechSerialTX); // RX, TX
#define SERIAL_SPEAK SwSerialSpeech		// seriale verso il modulo voce 
#define SERIAL_SPEAK_BAUD_RATE 9600
/*
void speakSerial(const char inStr[]) {
	bool blDrop = false;
	int waitCnt = 0;
	//mi assicuro che la stringa venga inviata interamente
	while ( (digitalRead(Pin_SpeechSerialBusy)==1) && !blDrop)
	{
		waitCnt++;
		if (waitCnt>=10)
		{
			blDrop = true;
			waitCnt = 0;//resetto il contatore
			dbg("..Spbusy")
		}
		dbg(".")
		chThdSleepMilliseconds(200);//	chThdYield();//	
	}
	osalSysDisable();
	SERIAL_SPEAK.print(inStr);
	osalSysEnable();
	dbg2(">>SPEAK riceve: ", inStr)


}*/
#define SPEAK(s) speakFifo(s)
#define SPEAK_CIAO				SPEAK("h"); 			
#define SPEAK_OIOI				SPEAK("O"); 			
#define SPEAK_CIAOCHISEI		SPEAK("ciao  ki sei"); 	
#define SPEAK_OK				SPEAK("k");				
#define SPEAK_TEST				SPEAK("t");				
#define SPEAK_MODE_SLAVE		SPEAK("SLEIV");			
#define SPEAK_AUTONOMO			SPEAK("AUTONOMO");		


#pragma endregion

#endif

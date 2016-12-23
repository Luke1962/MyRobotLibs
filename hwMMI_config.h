/*
 * hw_config.h
 *
 * Created: 25/01/2015 16:53:17
 *  Author: Luca
 */ 


#ifndef HWMMI_CONFIG_H_
#define HWMMI_CONFIG_H_
#include <Arduino.h>
 //----------------------------------------------
 // P O R T E   S E R I A L I    			  ---
 //----------------------------------------------

#define SERIAL_PC Serial		//  comandi dal PC
#define SERIAL_PC_BAUD_RATE 115200

 /// messaggio di test
 /// 31,1,1,1,1,300,100,220,1,1,1,1,1,1,70;
#define SERIAL_BT Serial	//Serial2,   mettere Serial per test interfaccia
#define SERIAL_BT_BAUD_RATE 115200

#define SERIAL_WIFI Serial		//  comunicazione bidirezionale via TCP
#define SERIAL_WIFI_BAUD_RATE 115200

#define SERIAL_ROBOT Serial1		// SERIAL_ROBOT=Serial1  
#define SERIAL_ROBOT_BAUD_RATE 115200


#define Pin_SpeechSerialBusy 41
#define Pin_SpeechSerialTX 43
 //----------------------------------------------
 // pin configurati dalle librerie			  ---
 //----------------------------------------------
 // display TFT
#define Pin_TFT_RS 36// 40
#define Pin_TFT_WR 37// 41
#define Pin_TFT_CS 38// 42
#define Pin_TFT_RST 39// 43

// Touch screen
#define Pin_TS_TIRQ  2
#define Pin_TS_CS  3	///was 9
#define Pin_TS_BUSY 4

//----------------------------------------------
// pin da configurare in INPUT -----------------
// N.B. Arduino (Atmega) pins default to inputs, n a high-impedance state. !!!
//----------------------------------------------

#define Pin_ROT_ENCODER_A 2
#define Pin_ROT_ENCODER_B 3
#define Pin_ROT_ENCODER_SWITCH 7

 



//----------------------------------------------
// pin da configurare in OUTPUT				----
//----------------------------------------------
#define Pin_Buzzer 10		//=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
#define  Pin_ONBOARD_LED 13

//iNGRESSI ANALOGICI----------------------------------------------
#define Pin_xxxxx A0 // A0 =54 per recuperara l'indice dell'array uso [Pin_AnaBase -Pin_AnaLight]
#define Pin_AnaBase 54 // A0 =54 per recuperara l'indice dell'array uso [Pin_AnaBase -Pin_AnaLight]
#define Pin_AnaPot1 A0		// Potenziometro che regola la velocità MAX
#define Pin_AnaVbat A1		// Potenziometro che regola la velocità MAX
#define Pin_AnaLight A2		// Sensore di luce LDR
//----------------------------------------------

//void InitMMIHardware() {
//	// configuro i pin in uscita	(delault in ingresso)----
//	pinMode(Pin_Buzzer, OUTPUT);			 
//	pinMode(Pin_ONBOARD_LED, OUTPUT);
//
//}


#endif /* HW_CONFIG_MOTORS_H_ */

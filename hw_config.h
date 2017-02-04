/*
 * hw_config.h
 *
 * Created: 25/01/2015 16:53:17
 *  Author: Luca
 */ 
 //////////////////////////////////////////////////////////////////////////
 //																		//
 //						 O P Z I O N I   R O B O T						//
 //																		//
 //////////////////////////////////////////////////////////////////////////

#define OPT_COMPASS 1
#define OPT_SERVOSONAR 1	//INCLUDI FUNZIONALITA' SERVO-SONAR
#define OPT_ENCODERS  0	//INCLUDI ENCODERS
 //#define OPT_BT 0	//bluetooth
 //#define OPT_SPEECH 0
#define OPT_GPS 1
#define OPT_LDS 1	//Laser Distance sensor


#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

// parametri ficici del robot---------------
#define	STEPSPERCM 23.342725f 
#define	MMPERSTEPS  0.42840f 

//----------------------------------------------
// pin da configurare in INPUT -----------------
// N.B. Arduino (Atmega) pins default to inputs, In a high-impedance state. !!!
//----------------------------------------------


#define Pin_EncRa	2			// encoder Right Motor segnale a	
#define Pin_EncLa	3			// encoder Left Motor segnale a	

#define Pin_irproxy_FW 28		// sensore di prossimità anteriore
#define Pin_PIR1	29			// Sensore presenza umana

#define Pin_EncRb	31			// encoder Right Motor segnale b
#define Pin_SonarEcho 32		// pin ECHO da Sonar
#define Pin_EncLb	33			// encoder Left Motor segnale b
#define Pin_irproxy_FWHL 34		// sensore di prossimità anteriore per alta luce
#define Pin_irproxy_BK 35		// sensore di prossimità POSTERIORE

#define Pin_BumbRight 37
#define Pin_BumbCenter 39
#define Pin_BtState 40
#define Pin_BumbLeft 41

#define Pin_irproxy_FR 43	// IR Proxy anteriore Destro
#define Pin_irproxy_FL 45	// IR Proxy anteriore sinistro

//----------------------------------------------
// pin da configurare in OUTPUT				----
//----------------------------------------------

#define Pin_ServoSonarPWM  9	// 
#define Pin_MotCK 10		//=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
#define  Pin_ONBOARD_LED 13

#define Pin_LaserOn 22	// 23 Accensione Laser 1=CW  0=CCW
#define Pin_MotCWR 23	// 23 left CW  - 24 RIGHT MOTOR
#define Pin_MotCWL 24	// 23 RIGHT MOTOR 1=CW  0=CCW
#define Pin_MotENR 25	// 26 RIGHT MOTOR ENABLE
#define Pin_MotENL 26	// 25 LEFT MOTOR ENABLE

#define Pin_SonarTrig 30	// pin TRIGGER vs Sonar

#define Pin_Rele1	36
#define Pin_Rele2	38
#define Pin_SwitchTop  40	// deviatore vicino arduino
//#define Pin_BtOnOff  42

#define  Pin_LED_TOP_R 46
#define  Pin_LED_TOP_G 47
#define  Pin_LED_TOP_B 48

#define  Pin_LED_TOP Pin_LED_TOP_G
//iNGRESSI ANALOGICI----------------------------------------------
#define Pin_AnaBase 54 // A0 =54 per recuperara l'indice dell'array uso [Pin_AnaBase -Pin_AnaLight]
#define Pin_AnaPot1 A0		// Potenziometro che regola la velocità MAX
#define Pin_AnaVbat A1		// Potenziometro che regola la velocità MAX
#define Pin_AnaLight A2		// Sensore di luce LDR
//----------------------------------------------


#endif /* HW_CONFIG_MOTORS_H_ */

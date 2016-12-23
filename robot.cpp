#include "robot.h"

//#include <../robot/robotmodel/robotModel.h>
#include "robot.h"
//#include "speak.h" spostata nel main
#include "dbg.h"


#include <FrequencyTimer2/FrequencyTimer2.h>	
#include <TimerThree/TimerThree.h> //usato per il clock motori
#include <FlexiTimer2/FlexiTimer2.h>

#include <digitalWriteFast/digitalWriteFast.h>

//#include "printf.h"
#include <ClickEncoder\ClickEncoder.h>

// compass
#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
#include <Adafruit_HMC5883_U\Adafruit_HMC5883_U.h>	//compass

#if OPT_GPS

#include <TinyGPSplus/TinyGPS++.h> //deve essere incluso anche nel main

// The serial connection to the GPS device
//SoftwareSerial SerialGps(RXPin, TXPin);


TinyGPSPlus Gps;


#pragma region  path_recorder
//// registra i movimenti fatti
//
//// include stack library header.
////#include <StackArray/StackArray.h>
//
//struct motionHistory_t {
//	commandDir_e cmdDir;
//	int val;
//};
//// create a stack of numbers.
//StackArray <motionHistory_t> stack;
#pragma endregion

#endif


//operatingMode_e operatingMode;
//unsigned long previousToggleAccel = 0;

#define MOTORS_DISABLE 		digitalWriteFast( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE ); digitalWriteFast(Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE);
#define MOTORS_ENABLE 		digitalWriteFast( Pin_MotENR, ROBOT_MOTORENABLE_ACTIVE ); digitalWriteFast(Pin_MotENL, ROBOT_MOTORENABLE_ACTIVE);


#if OPT_ENCODERS
	//Encoder EncL(Pin_EncLa,Pin_EncLb);
	//Encoder EncR(Pin_EncRa,Pin_EncRb);
	
	inline int32_t encRead(Encoder_state_t enc) {
		noInterrupts();
		int32_t ret = enc.position;
		Serial.println(ret);
		interrupts();
		return ret;
	}
	inline void encWrite(Encoder_state_t enc, int32_t p) {
		noInterrupts();
		enc.position = p;
		interrupts();
	}
	
	/*
	static void encUpdate(Encoder_internal_state_t *enc) {
	
		uint8_t p1val = DIRECT_PIN_READ(enc->pin1_register, enc->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(enc->pin2_register, enc->pin2_bitmask);
		uint8_t state = enc->state & 3;
		if (p1val) state |= 4;
		if (p2val) state |= 8;
		enc->state = (state >> 2);
		switch (state) {
			case 1: case 7: case 8: case 14:
			enc->position++;
			return;
			case 2: case 4: case 11: case 13:
			enc->position--;
			return;
			case 3: case 12:
			enc->position += 2;
			return;
			case 6: case 9:
			enc->position -= 2;
			return;
		}
	};
	*/
//	static void ISRencR(void) { encUpdate(encR); }	//{ update(interruptArgs[0]); }
	//static void ISRencL(void) { encUpdate(encL); }	//{ update(interruptArgs[1]); }
	
	//////////////////////////////////////////////////////////////////////////
	// INTERRUPT SERVICE ROUTINE	impostata dal robot_c::begin()
	//////////////////////////////////////////////////////////////////////////
	void robot_c::ISRencR() {		
		noInterrupts();

		//uint8_t p1val = DIRECT_PIN_READ(robot.encR.pin1_register, robot.encR.pin1_bitmask);
		//uint8_t p2val = DIRECT_PIN_READ(robot.encR.pin2_register, robot.encR.pin2_bitmask);

			//provo così
		uint8_t p1val = digitalReadFast( Pin_EncRa );
		uint8_t p2val = digitalReadFast( Pin_EncRb );

		uint8_t state = robot.encR.state & 3;
		if (p1val) state |= 4;
		if (p2val) state |= 8;
		robot.encR.state = (state >> 2);
		switch (state) {
			case 1: case 7: case 8: case 14:
			robot.encR.position++;
			return;
			case 2: case 4: case 11: case 13:
			robot.encR.position--;
			return;
			case 3: case 12:
			robot.encR.position += 2;
			return;
			case 6: case 9:
			robot.encR.position -= 2;
			return;
		}
		Serial.print(">");
		interrupts();
		
		
	};
	void robot_c::ISRencL( void ) {
		noInterrupts();
		//uint8_t p1val = DIRECT_PIN_READ(robot.encL.pin1_register, robot.encL.pin1_bitmask);
		//uint8_t p2val = DIRECT_PIN_READ(robot.encL.pin2_register, robot.encL.pin2_bitmask);

		uint8_t p1val = digitalReadFast( Pin_EncLa );
		uint8_t p2val = digitalReadFast( Pin_EncLb );


		uint8_t state = robot.encL.state & 3;
		if (p1val) state |= 4;
		if (p2val) state |= 8;
		robot.encL.state = (state >> 2);
		switch (state) {
			case 1: case 7: case 8: case 14:
			robot.encL.position++;
			return;
			case 2: case 4: case 11: case 13:
			robot.encL.position--;
			return;
			case 3: case 12:
			robot.encL.position += 2;
			return;
			case 6: case 9:
			robot.encL.position -= 2;
			return;
		}
		#if DEBUG_ENCODERS
		Serial.print("<");	// digitalWriteFast(13, !digitalReadFast(13));
		#endif
		interrupts();
	};
		
#endif

///----------------------------------------------------------
// Reserve space for 10 entries in the average bucket.
// Change the type between < and > to change the entire way the library works.
///----------------------------------------------------------
Average<float> avgPing(SONAR_MAX_SAMPLES);
	
	
///----------------------------------------------------------
//  ROBOT CONSTRUCTOR
///---------------------------------------------------------
robot_c::robot_c(){
	this->begin(operatingMode_e::MODE_SLAVE);
}
///inizializzazione di base
void robot_c::_begin(){  
	dbg("begin..")
	// configuro i pin in uscita								----
	pinMode(Pin_MotCK, OUTPUT);			digitalWrite(Pin_MotCK,1); // il CK è a logica negata : impulso 0 di ampiezza minima di 0.5 uSec. --pinMode(FREQUENCYTIMER2_PIN, OUTPUT);

	pinMode(Pin_MotCWR, OUTPUT);		digitalWrite(Pin_MotCWR,0);
	pinMode( Pin_MotENR, OUTPUT );		digitalWrite( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );
	pinMode(Pin_MotCWL, OUTPUT);		digitalWrite(Pin_MotCWL,0);
	pinMode( Pin_MotENL, OUTPUT );		digitalWrite( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );
	pinMode(Pin_Rele1, OUTPUT);			digitalWrite(Pin_Rele1,1);
	pinMode(Pin_Rele2, OUTPUT);			digitalWrite(Pin_Rele2,1);	// i Rele vanno a logica negata
	pinMode(Pin_LaserOn, OUTPUT);		digitalWrite(Pin_LaserOn,0);
	pinMode(Pin_SonarTrig, OUTPUT);		digitalWrite(Pin_SonarTrig,128);	// pin TRIGGER vs Sonar
	pinMode( Pin_LED_TOP_R, OUTPUT );		digitalWrite( Pin_LED_TOP_R, 0 );	// led superiore
	pinMode( Pin_LED_TOP_G, OUTPUT );		digitalWrite( Pin_LED_TOP_G, 0 );	// led superiore
	pinMode( Pin_LED_TOP_B, OUTPUT );		digitalWrite( Pin_LED_TOP_B, 0 );	// led superiore

	//--------------------------------------------------------------

 	// configuro i pin in ingresso (default) ----
	pinMode(Pin_irproxy_FW, INPUT);
	pinMode(Pin_irproxy_FWHL, INPUT);
	pinMode(Pin_irproxy_FR, INPUT);		
	pinMode(Pin_irproxy_FL, INPUT);		
	pinMode(Pin_irproxy_BK, INPUT);		
	pinMode(Pin_PIR1, INPUT);
	pinMode( Pin_SwitchTop, INPUT_PULLUP );	//Deviatore vs Ground

	pinMode( Pin_BumbRight , INPUT_PULLUP );	// Bumper
	pinMode( Pin_BumbCenter, INPUT_PULLUP );	// Bumper
	pinMode( Pin_BumbLeft, INPUT_PULLUP );	// Bumper

	pinMode(Pin_EncRa,INPUT_PULLUP);	//encoder Right Motor segnale a
	pinMode(Pin_EncRb,INPUT_PULLUP);	//encoder Right Motor segnale b
	pinMode(Pin_EncLa,INPUT_PULLUP);	//encoder Left Motor segnale a
	pinMode(Pin_EncLb,INPUT_PULLUP);	//encoder Left Motor segnale b
	pinMode( Pin_SonarEcho,INPUT);	// pin ECHO da Sonar
	//-----------------------------------------

	#if	OPT_BT
		pinMode(Pin_BtOnOff, OUTPUT);
		pinMode(Pin_BtState, INPUT);  // Bluetooth
		digitalWrite(Pin_BtOnOff,1);		//abilito il BlueTooth
	#endif
	#if OPT_ENCODERS

		attachInterrupt( Pin_EncRa, robot_c::ISRencR, RISING );
		attachInterrupt( Pin_EncLa, robot_c::ISRencL, RISING );

		//attachInterrupt( Pin_EncRa, &robot_c::ISRencR, CHANGE );
		//attachInterrupt( Pin_EncLa, &robot_c::ISRencL, CHANGE );
		//pEncR = &EncR;
		//pEncL = &EncL;
			
		//			Encoder_internal_state_t encR;
		//			Encoder_internal_state_t encL;			
	#endif

	//-----------------------------------------
	// Setup Sensori
	//-----------------------------------------
	dim.WeelRadius = 59;  //mm
	status.ts = 0;
	status.sensors.irproxy.bk = 0;
	status.sensors.irproxy.fl = 0;
	status.sensors.irproxy.fr = 0;
	status.sensors.irproxy.fw = 0;
	status.sensors.irproxy.fwHL=0;

	status.sensors.EncL = 0;
	status.sensors.EncR = 0;
	status.sensors.pirDome =false;
	status.sensors.gps.sats = 0;
	status.sensors.gps.lat = 0.0;
	status.sensors.gps.lng = 0.0;
	status.sensors.gps.alt = 0.0;

	statusOld = status;

	posHome.x	= 0.0;
	posHome.y	= 0.0;
	posHome.r	= 0.0;
	status.posCurrent.x = 0.0;
	status.posCurrent.y = 0.0;
	status.posCurrent.r = 0.0;

	//-----------------------------------------
	// Setup Motori
	//-----------------------------------------
	status.cmd.cwL = false;
	status.cmd.cwR = false;
	status.cmd.enL = false;
	status.cmd.enR = false;
	status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX; //+ROBOT_MOTOR_CLOCK_microsecondMIN)/2;
	status.cmd.stepsDone=0;
	status.cmd.targetSteps=0;
	status.cmd.targetSpeed= 30; //35 cm/s max ma meglio 30/cms
	status.cmd.acceleration = ROBOT_MOTOR_CLOCK_ACCEL ; // tra 8 e 5 ok

	status.parameters.maxspeed_ck =ROBOT_MOTOR_CLOCK_microsecondMAX;
	status.parameters.minspeed_ck =ROBOT_MOTOR_CLOCK_microsecondMIN;
		
//	FlexiTimer2::set(ROBOT_MOTOR_CLOCK_microsecondMAX, 0.000001, this->_motorCKpulse); // ogni [1] tick con cadenza [2] every 500 1ms "ticks"

	//-----------------------------------------
	// Setup RELE
	//-----------------------------------------
	setRele(1,0);
	setRele(2,1);	//webcam



	#if	0
		//-----------------------------------------
		// configuro Bluetooth					 ----
		//-----------------------------------------
		//		digitalWrite(Pin_BtOnOff, LOW);	//modalità comando BT
		bt.cmd("AT+BAUD4", 4000); //9600 baud
		digitalWrite(Pin_BtOnOff, HIGH);

		//DEBUG_PRINTLN("Starting recovery in 3 seconds.");
		//btSerial.cmdMode2Start(powerPin);

		//// Provide some time for the user to start the serial monitor
		//delay(3000);

		//// For curiosity's sake, ask for the old settings
		//btSerial.cmd("AT+UART?");

		//// Now set the baud to 19200N1
		//btSerial.cmd("AT+UART=9600,0,0");

		//// Exit command mode and switch to the new baud setting
		//btSerial.cmd("AT+RESET");
		//btSerial.cmdMode2End();
		  
		dbg("Waiting BT pairing...");
		while (!bt.connected()){
			dbg(".");
			};
		dbg("OK BT Connected..");
	#endif


 	setPose(0, 0, 90);
  
 	status.cmd.commandDir = STOP;
  
	//this->readSensors();


};	// end begin()

void robot_c::begin(operatingMode_e initialOperatingMode) {
	status.operatingMode = initialOperatingMode;
	_begin();
}

#if OPT_COMPASS && OPT_SERVOSONAR && OPT_LDS && OPT_COMPASS
	void robot_c::begin( Servo *pServoSonar, NewPing *pSonar, VL53L0X *pDistanceSensor, COMPASS_CLASS *pCompass){
 		this->_pServoSonar = pServoSonar;
		this->_pSonar = pSonar;
		this->_pLDS = pDistanceSensor;
		this->_pCompass = pCompass;

		_begin();
		beginServosonar(pServoSonar, pSonar);
		beginLDS(pDistanceSensor);
		beginCompass(pCompass);

		this->readSensors();
	}
#endif

#if OPT_SERVOSONAR

	void robot_c::beginServosonar( Servo *pServoSonar, NewPing *pSonar){
		this->_pServoSonar = pServoSonar;
		this->_pSonar = pSonar;
		if (!_pServoSonar->attached())
		{
			this->_pServoSonar->attach( Pin_ServoSonarPWM );
		}

		//-----------------------------------------
		// Setup parametri Sonar
		//-----------------------------------------
		this->status.parameters.sonarScanSpeed = 100;  // ms di attesa tra due posizioni
		status.parameters.sonarMaxDistance=500; //in cm
		status.parameters.sonarScanSweeps =  1;

		status.parameters.sonarStartAngle =  0;
		status.parameters.sonarEndAngle  = 180; // ampiezza angolo di scansione in gradi
		status.parameters.sonarStepAngle  =  5; //  ampiezza step in gradi della scansione (ok anche 10)
		//		status.parameters.sonarScanSteps = (int)(status.parameters.sonarScanAngle / status.parameters.sonarStepAngle);
		status.parameters.sonarMedianSamples = 10;
		// inizializzo l'array delle echo
		for (int i = 0; i <180; i++) 
		{
			status.sensors.sonarEchos[i] = 0;
		}


	}

	//////////////////////////////////////////////////////////////////////////
	/// Esegue una doppia passata del sonar e memorizza le distanze in Cm in statussensors.sonarEchos[]
	/// in:  status.parameters.sonarStartAngle = angolo iniziale
	/// in:  status.parameters.sonarEndAngle = ampiezza scansione in gradi
	/// in: status.parameters.sonarStepAngle = ampiezza singolo step in gradi
	/*	
	void robot_c::SonarScanBatch_old( Servo *pServoSonar, NewPing *_pSonar )
	{
		_pServoSonar->attach( Pin_ServoSonarPWM );

		// Accendo il Laser
		LASER_ON
		int servoPos, i,n;

		// resetto l'array
		for ( i = 0; i < 255; i++){status.sensors.sonarEchos[i] = 0;}

		// per ogni sweep (default 1)
		for (n = 0; n  < status.parameters.sonarScanSweeps; n++){
			servoPos = status.parameters.sonarStartAngle;
			i = 0; // indice  nell'array
			while ((servoPos <= status.parameters.sonarEndAngle) && (i<=255))
			{
				//sposto il sonar
				_pServoSonar->write( servoPos );
				delay( 50 );

				// Send ping, get ping time in microseconds (uS).
				unsigned int uS = _pSonar->ping(); 
				status.sensors.sonarEchos[i] = uS / US_ROUNDTRIP_CM;

				//status.sensors.sonarEchos[i] = SonarPingAtPos( servoPos ); //_pSonar->ping_cm();// Send ping, get distance.  
				
				//calcolo nuova posizione sonar
				servoPos +=  status.parameters.sonarStepAngle; 

				// incremento l'indice  nell'array
				i++; 
			}
			//inversione direzione

			//	Serial.print( "1,RAM:" ); Serial.print( freeRam() ); Serial.print( ";" );
			//
			//servoPos = status.parameters.sonarEndAngle;
			//i--;	
			//while ((servoPos >= status.parameters.sonarStartAngle) && (i > 0))  // sweep nell'altro senso
			//{
			//	delay( status.parameters.sonarScanSpeed );  // Wait [ms] between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.              

			//	// Media con l'andata-------------------------------
			//	status.sensors.sonarEchos[i] =	SonarPingAtPos( servoPos );  // (int)((this->_pSonar->ping_cm() + status.sensors.sonarEchos[servoPos]) >> 1);  // >>1 = divisione per 2
			//	servoPos -=  status.parameters.sonarStepAngle;
			//	i--;

			//}
			//	Serial.print( "1,RAM:" ); Serial.print( freeRam() ); Serial.print( ";" );
			
		}

		//Salvo l'indice per sapere poi quanti dati inviare'
		status.parameters.sonarScanSteps = i;

		// Spengo il Laser
		LASER_OFF

		_pServoSonar->detach();

	}
*/

	
	
	


	//////////////////////////////////////////////////////////////////////////
	// Esegue una scansione di 180 ° e memorizza l'angolo in gradi con la maggiore distanza
	//////////////////////////////////////////////////////////////////////////
	void robot_c::SonarScanBatch( Servo *_pServoSonar, NewPing *_pSonar ){
		_pServoSonar->attach( Pin_ServoSonarPWM );

		// Accendo il Laser
		LASER_ON;
		int nSweep,servoPos, i, direction;
		int maxDistAtAngle = 0;
		int maxDistAtIndex = 0;	// memorizza l'indice che ha la massima distanza fno a quel momento

		// resetto l'array che poi dovrò inviare con kbSonarSendData
		for ( i = 0; i < SONAR_ARRAY_SIZE; i++){status.sensors.sonarEchos[i] = 0;}

		i = 0; // indice  nell'array
		servoPos = 0;// status.parameters.sonarStartAngle;
		direction = 1; // alterna tra 1 e -1 tra uno sweep in una direzione e ritorno

		// per ogni sweep (default 1)
		for (nSweep = 0; nSweep < status.parameters.sonarScanSweeps; nSweep++) {
			// per ogni angolo tra sonarStartAngle e sonarEndAngle
			while ( (servoPos >= status.parameters.sonarStartAngle) && (servoPos <= status.parameters.sonarEndAngle) && (i>=0) && (i<SONAR_ARRAY_SIZE))
			{
				
				_pServoSonar->write( servoPos );//sposto il sonar

				//delay(max(analogRead(Pin_AnaPot1),30));// Wait [ms] between pings (about 20 pings/sec). 29ms should be the shortest delay between pings. 
				
				
				// L E T T U R A  S O N A R
						
				//unsigned int uS = _pSonar->ping(); // Send ping, get ping time in microseconds (uS).
				unsigned int uS = _pSonar->ping_median(status.parameters.sonarMedianSamples); // Send ping, get ping time in microseconds (uS).
				delay( 30 );

				if (nSweep == 0){	// prima passata -> assegno sempllicemente il valore
					status.sensors.sonarEchos[i] = uS / US_ROUNDTRIP_CM;
				//	ardprintf("servoPos %d , echo %d", servoPos, status.sensors.sonarEchos[i]);
				}
				else //passate successive -> faccio la media con i valori precedenti
				{
					status.sensors.sonarEchos[i] = (status.sensors.sonarEchos[i]  +  uS / US_ROUNDTRIP_CM)*0.5;
				}

				//debug
				//printf("\n >servoPos %d , echo %u" , servoPos, status.sensors.sonarEchos[i]);
				dbg2("servoPos",servoPos)
 				dbg2("eco",uS)
				/*	
					status.sensors.sonarEchos[i] = _pSonar->ping_cm();
				*/
				// Memorizzo l'angolo se la distanza è maggiore di quella memorizzata in precedenza
				// ma inferiore a quella massima per scartare le mancate eco
				if ((status.sensors.sonarEchos[i] > status.sensors.sonarEchos[maxDistAtIndex]) || (status.sensors.sonarEchos[i] = NO_ECHO))
				{
					maxDistAtAngle = servoPos;
					maxDistAtIndex = i;
				}
				
				
				servoPos +=  status.parameters.sonarStepAngle*direction; //calcolo nuova posizione sonar		
				i += direction;		//// incremento l'indice  nell'array al posto di  i++;
			}

			// Memorizzo l'indice che riporta il numero di campioni al primo sweep solamente
			if (nSweep==0)
			{
				status.parameters.sonarScanSteps = i;
			}

			//inversione direzione
			direction = -direction;
			
			// riporto servoPos e i all'interno del range valido di scansione
			servoPos += status.parameters.sonarStepAngle*direction;
			i += direction;	

		}	//sweep successivo

		//Serial.print("maxDistAtAngle:"); Serial.println(maxDistAtAngle);

		
		LASER_OFF	// Spengo il Laser

		_pServoSonar->detach();	//scollego il Servo

		status.parameters.SonarMaxDistAngle = maxDistAtAngle;
	}
	#pragma region SonarScanBatch_bkp
		//int robot_c::SonarScanBatch_bkp(Servo *pServoSonar, NewPing *_pSonar) {
	//	_pServoSonar->attach(Pin_ServoSonarPWM);

	//	// Accendo il Laser
	//	LASER_ON;
	//	int nSweep, servoPos, i, direction;
	//	int maxDistAtAngle = 0;
	//	int maxDistAtIndex = 0;	// memorizza l'indice che ha la massima distanza fno a quel momento

	//							// resetto l'array
	//	for (i = 0; i < 255; i++) { status.sensors.sonarEchos[i] = 0; }

	//	i = 0; // indice  nell'array
	//	servoPos = 0;// status.parameters.sonarStartAngle;
	//	direction = 1; // alterna tra 1 e -1 tra uno sweep in una direzione e ritorno
	//	status.parameters.sonarStepAngle = 30;
	//	// per ogni sweep (default 1)
	//	for (nSweep = 0; nSweep < 1; nSweep++) {
	//		while ((servoPos >= 0) && (servoPos <= 180) && (i<255) && (i >= 0))
	//		{

	//			_pServoSonar->write(servoPos);//sposto il sonar
	//			delay(100);//attendo smorzamento oscillazioni

	//					   //unsigned int uS = _pSonar->ping(); // Send ping, get ping time in microseconds (uS).
	//			unsigned int uS = _pSonar->ping_median(10); // Send ping, get ping time in microseconds (uS).
	//														//delay( 50 );

	//			if (nSweep == 0) {	// prima passata -> assegno sempllicemente il valore
	//				status.sensors.sonarEchos[i] = uS / US_ROUNDTRIP_CM;
	//				//	ardprintf("servoPos %d , echo %d", servoPos, status.sensors.sonarEchos[i]);
	//			}
	//			else //passate successive -> faccio la media con i valori precedenti
	//			{
	//				status.sensors.sonarEchos[i] = (status.sensors.sonarEchos[i] + uS / US_ROUNDTRIP_CM)*0.5;
	//				//	ardprintf("servoPos %d , echo %d" , servoPos, status.sensors.sonarEchos[i]);
	//			}

	//			// Memorizzo l'angolo se la distanza è maggiore di quella memorizzata in precedenza
	//			// ma inferiore a quella massima per scartare le mancate eco
	//			if ((status.sensors.sonarEchos[i] > status.sensors.sonarEchos[maxDistAtIndex]) && (status.sensors.sonarEchos[i] <= MAX_SENSOR_DISTANCE))
	//			{
	//				maxDistAtAngle = servoPos;
	//				maxDistAtIndex = i;
	//			}


	//			servoPos += status.parameters.sonarStepAngle*direction; //calcolo nuova posizione sonar		
	//			i += direction;		//// incremento l'indice  nell'array al posto di  i++;
	//		}

	//		//inversione direzione
	//		direction = -direction;

	//		// riporto servoPos e i all'interno del range valido di scansione
	//		servoPos += status.parameters.sonarStepAngle*direction;
	//		i += direction;

	//	}
	//	//Serial.print("maxDistAtAngle:"); Serial.println(maxDistAtAngle);


	//	LASER_OFF	// Spengo il Laser

	//		_pServoSonar->detach();	//scollego il Servo

	//	return maxDistAtAngle;
	//}

	#pragma endregion

 
	/// ///////////////////////////////////////////////////////////////////////
	// Esegue un singolo ping alla posizione specificata(0-180°) e ritorna la distanza
	/// ///////////////////////////////////////////////////////////////////////
	int robot_c::SonarPingAtPos( int pos ){
		//		analogWrite(Pin_ServoSonarPWM, servoPos);
		//		this->_pServoSonar->attach(Pin_ServoSonarPWM);

		// Posiziono il sonar all'angolo desiderato
 		this->_pServoSonar->write( pos ); //servoSonar.write( servoPos );
		
		delay( 50 );	//serve a stabilizzare le oscillazioni
		return SonarPing();

		//unsigned int echoTime = 0;
		//echoTime = this->_pSonar->ping();         // Calls the ping method and returns with the ping echo distance in uS.
		//int cm;
		//cm = echoTime / US_ROUNDTRIP_CM;              // Call the ping method and returns the distance in centimeters (no rounding).

		////Serial.println( echoTime );
		//return cm;

//		this->_pServoSonar->detach();
	}
	int robot_c::SonarPingAtPosAvg( int pos, float *maxStdDev , int samples ){
		int dist = 0;
		LASER_ON
		if (samples> SONAR_MAX_SAMPLES)
		{
			samples = SONAR_MAX_SAMPLES;
		}
		// Posiziono il sonar all'angolo desiderato
 		this->_pServoSonar->write( pos ); //servoSonar.write( servoPos );
		
		delay( 50 );	//serve a stabilizzare le oscillazioni

		for (int i = 0; i < samples; i++)
		{

			avgPing.push(SonarPing());
		}
		
		LASER_OFF
		if (avgPing.stddev() < *maxStdDev) // se la deviazione è inferiore alla massima accettata
		{
			dist= avgPing.mode() ;
		}
		else
		{
			dist= -1; //dato invalido
		}

		*maxStdDev = avgPing.stddev();
		return dist;

 	}	//////////////////////////////////////////////////////////////////////////
	/// Esegue un singolo ping alla posizione corrente e ritorna  la distanza
	//////////////////////////////////////////////////////////////////////////
	int robot_c::SonarPing( ){
		return  this->_pSonar->ping_cm();// / US_ROUNDTRIP_CM;

	}
	/// Ritorna la distanza in cm



#endif // SERVO_SONAR

#if OPT_LDS

	void robot_c::beginLDS(VL53L0X *pDistanceSensor) {
		this->_pLDS = pDistanceSensor;
		_begin();
		_pLDS->init();
		this->readSensors();
	}
	int robot_c::getLDSDistance(){

		uint16_t d = this->_pLDS->readRangeSingleMillimeters();
		if (!this->_pLDS->timeoutOccurred()) { 
			if (d/10 < LDS_MAX_DISTANCE_CM)
			{
				return d / 10;
			}
			else
			{
				return LDS_MAX_DISTANCE_CM;
			}
			 
		}
		else
		{
			dbg("LDS TIMEOUT");
			return LDS_TIMEOUT_DISTANCE_CM ;
		}
	}
//////////////////////////////////////////////////////////////////////////
// Esegue una scansione di 180 ° e memorizza l'angolo in gradi con la maggiore distanza
//////////////////////////////////////////////////////////////////////////
//void robot_c::LDSScanBatch( Servo *_pServoSonar, VL53L0X *pDistanceSensor){
void robot_c::LDSScanBatch(){
	_pServoSonar->attach( Pin_ServoSonarPWM );

	int nSweep;
	int servoPos;
	int i;
	int direction;
	int maxDistAtAngle = 0;
	int maxDistAtAngleCnt;

	int bestEscapeAtAngle=-1;
	int bestEscapeWidth = 0;
//			int maxDistAtIndex = 90;	// memorizza l'indice che ha la massima distanza fno a quel momento

	// resetto l'array che poi dovrò inviare con kbSonarSendData
	for ( i = 0; i < SONAR_ARRAY_SIZE; i++){status.sensors.sonarEchos[i] = LDS_TIMEOUT_DISTANCE_CM;}

	i = 0; // indice  nell'array
	servoPos = 0;// status.parameters.sonarStartAngle;
	direction = 1; // alterna tra 1 e -1 tra uno sweep in una direzione e ritorno

	// per ogni sweep (default 1)
	for ( nSweep = 0; nSweep < status.parameters.sonarScanSweeps; nSweep++) {
		// per ogni angolo tra sonarStartAngle e sonarEndAngle
		while ( (servoPos >= status.parameters.sonarStartAngle) && (servoPos <= status.parameters.sonarEndAngle) && (i>=0) && (i<SONAR_ARRAY_SIZE))
		{
				
			_pServoSonar->write( servoPos );//sposto il sonar (_pServoSonar limita già a 180)

			//delay(30);// Wait [ms] between pings (about 20 pings/sec). 29ms should be the shortest delay between pings. 
				
				
			// Accendo il Laser
			LASER_ON;
			// L E T T U R A  LASER DISTANCE	ritorna  -1 se oltre il range					
			uint16_t dist = getLDSDistance();
			//dbg2("lds:",dist);
			LASER_OFF	// Spengo il Laser


			if (nSweep == 0){	// prima passata -> assegno semplicemente il valore
				status.sensors.sonarEchos[servoPos] = dist ;
				//	ardprintf("servoPos %d , echo %d", servoPos, status.sensors.sonarEchos[i]);
			}
			else //passate successive -> faccio la media con i valori precedenti
			{
				if (status.sensors.sonarEchos[servoPos] >0 ) //se la lettua precedente è valida
				{
					status.sensors.sonarEchos[servoPos] =( (status.sensors.sonarEchos[servoPos]  + dist  )>>1);
				}
				else
				{
					status.sensors.sonarEchos[servoPos] = dist;
				}
						
			}

			//debug
			//printf("\n >servoPos %d , echo %u" , servoPos, status.sensors.sonarEchos[i]);
			dbg(servoPos);
			dbg2("\t", dist);


			/// //////////////////////////////////////////////////////
			/// RICERCA PRIMO ANGOLO CON LA MAGGIORE DISTANZA 
			/// //////////////////////////////////////////////////////
			if(dist>= LDS_MAX_DISTANCE_CM)
			{
				//salvo la posizione e poi ne faccio la media
				maxDistAtAngle += servoPos;
				maxDistAtAngleCnt++;
			}
			else
			{
				if (maxDistAtAngleCnt> 0)
				{
					///se la distanza non è quella massima
					/// interrompo l'accumul e restituisco la posizione media
					maxDistAtAngle /= maxDistAtAngleCnt;
					dbg2("possible exit at:", maxDistAtAngle);
					dbg2("\t cnt:", maxDistAtAngleCnt);

					if (bestEscapeWidth<0 || bestEscapeWidth<maxDistAtAngleCnt)
					{
						bestEscapeAtAngle = maxDistAtAngle;
						bestEscapeWidth = maxDistAtAngleCnt;
					}

					// reset per una nuova ricerca ad altri angoli
					maxDistAtAngleCnt = 0;
					maxDistAtAngle = 0;

				}

			}

			// Memorizzo l'angolo se la distanza è maggiore di quella memorizzata in precedenza
			// ma inferiore a quella massima per scartare le mancate eco
			//if ((status.sensors.sonarEchos[servoPos] > status.sensors.sonarEchos[maxDistAtAngle]) )
			//{
			//	maxDistAtAngle = servoPos;
			//	dbg2("new Max at:", servoPos);
			//	 
			//}
				
				
			servoPos +=  status.parameters.sonarStepAngle*direction; //calcolo nuova posizione sonar		
			//i += direction;		//// incremento l'indice  nell'array al posto di  i++;
		}

		// Memorizzo l'indice che riporta il numero di campioni al primo sweep solamente
		//if (nSweep==0)
		//{
		//	status.parameters.sonarScanSteps = i;
		//}

		//inversione direzione
		direction = -direction;
			
		// riporto servoPos e i all'interno del range valido di scansione
		servoPos += status.parameters.sonarStepAngle*direction;
		//i += direction;	

	}	//sweep successivo

	//Serial.print("maxDistAtAngle:"); Serial.println(maxDistAtAngle);
	dbg("end Scan");
		

	_pServoSonar->detach();	//scollego il Servo

	status.parameters.SonarMaxDistAngle = maxDistAtAngle;
	dbg2("bestUscitaAtAngle:", bestEscapeAtAngle);
	dbg2("bestUscita cnt:", bestEscapeWidth);
	LASER_OFF	// Spengo il Laser

}
///////////////////////////////////////////////////////////////////////////////////////////
/// Esegue un singolo ping alla posizione specificata(0-180°) e ritorna la distanza in cm
///////////////////////////////////////////////////////////////////////////////////////////
int robot_c::getLDSDistance(int pos) {
	//		analogWrite(Pin_ServoSonarPWM, servoPos);
	//		this->_pServoSonar->attach(Pin_ServoSonarPWM);

	// Posiziono il sonar all'angolo desiderato
	this->_pServoSonar->write(pos); //servoSonar.write( servoPos );

	delay(50);	//serve a stabilizzare le oscillazioni
	return getLDSDistance();

	//unsigned int echoTime = 0;
	//echoTime = this->_pSonar->ping();         // Calls the ping method and returns with the ping echo distance in uS.
	//int cm;
	//cm = echoTime / US_ROUNDTRIP_CM;              // Call the ping method and returns the distance in centimeters (no rounding).

	////Serial.println( echoTime );
	//return cm;

	//		this->_pServoSonar->detach();
}
/// Esegue un singolo ping alla posizione corrente e ritorna  la distanza
//////////////////////////////////////////////////////////////////////////
int robot_c::LDSPing() {
	return  this->_pSonar->ping_cm();// / US_ROUNDTRIP_CM;

}
/// Ritorna la distanza in cm#endif // SERVO_LDS
#endif


#if OPT_COMPASS

	void robot_c::beginCompass(COMPASS_CLASS *pCompass){
 		this->_pCompass = pCompass;
	}


#endif // SERVO_SONAR



/*
//void Timer2enable()
//{
//	#if defined(TCCR2A)
//		TCCR2A |= _BV(COM2A0);
//	#elif defined(TCCR2)
//		TCCR2 |= _BV(COM20);
//	#endif
//}
//void Timer2setPeriod(unsigned long period)
//{
//    uint8_t pre, top;
//  
//    if ( period == 0) period = 1;
//    period *= clockCyclesPerMicrosecond();
// 
//    period /= 2;            // we work with half-cycles before the toggle 
//    if ( period <= 256) {
//	pre = 1;
//	top = period-1;
//    } else if ( period <= 256L*8) {
//	pre = 2;
//	top = period/8-1;
//    } else if ( period <= 256L*32) {
//	pre = 3;
//	top = period/32-1;
//    } else if ( period <= 256L*64) {
//	pre = 4;
//	top = period/64-1;
//    } else if ( period <= 256L*128) {
//	pre = 5;
//	top = period/128-1;
//    } else if ( period <= 256L*256) {
//	pre = 6;
//	top = period/256-1;
//    } else if ( period <= 256L*1024) {
//	pre = 7;
//	top = period/1024-1;
//    } else {
//	pre = 7;
//	top = 255;
//    }
//
//#if defined(TCCR2A)
//    TCCR2B = 0;
//    TCCR2A = 0;
//    TCNT2 = 0;
//#if defined(ASSR) && defined(AS2)
//    ASSR &= ~_BV(AS2);    // use clock, not T2 pin
//#endif
//    OCR2A = top;
//    TCCR2A = (_BV(WGM21) | ( FrequencyTimer2::enabled ? _BV(COM2A0) : 0));
//    TCCR2B = pre;
//#elif defined(TCCR2)
//    TCCR2 = 0;
//    TCNT2 = 0;
//    ASSR &= ~_BV(AS2);    // use clock, not T2 pin
//    OCR2 = top;
//    TCCR2 = (_BV(WGM21) | ( FrequencyTimer2::enabled ? _BV(COM20) : 0)  | pre);
//#endif
//
//}

*/



void robot_c::_motorCKpulse()		// chiamato da Timer2; frequenza impostata  da _motorSetCKspeed()
{
	noInterrupts();	

//
//#if OPT_ENCODERS 
//	// non fa nulla inquanto interviene la ISR 
//#if DEBUG_ENCODERS
//
//#endif // DEBUG_ENCODERS
//
//
//#else	// se non uso gli encoders conto gli step ad anello aperto
//
//	if (MotCKoutput == HIGH)
//	{
//		// incremento ogni due chiamate -----------
//		status.cmd.stepsDone++;
//		//digitalWriteFast(Pin_ONBOARD_LED, 1); 	//per debug
//	}
//	else{
//		//digitalWriteFast(Pin_ONBOARD_LED, 0); 	//per debug
//	}
//
//	//digitalWriteFast(Pin_ONBOARD_LED, LEDoutput); 	//digitalWrite(Pin_ONBOARD_LED, !LEDoutput); 
//	//LEDoutput = !LEDoutput; //debug
//
//#endif
//

	// Manda fuori l'impulso 
	digitalWriteFast( Pin_MotCK, 0 );//	digitalWriteFast( 13, 1 );
	//delay( 1 );
	delayMicroseconds( 2 );	// almeno 0,5 microsecondi
	status.cmd.stepsDone++;// .IncstepsDone();
	digitalWriteFast( Pin_MotCK, 1 );	//digitalWriteFast( 13,0 );



//	Serial.print( "!" );
//	Serial.print( status.cmd.stepsDone++ );
	interrupts();
}
/// Imposta la velocità del clock motori limitandola tra
/// quella minima ROBOT_MOTOR_CLOCK_microsecondMAX
/// e quella massima ROBOT_MOTOR_CLOCK_microsecondMIN
void robot_c::_motorSetPWMCK(motCk_t clock )
{
	// controlla che il clock sia entro i limiti
		
	if( clock > ROBOT_MOTOR_CLOCK_microsecondMAX) {
		this->status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX;
	}else if( clock < ROBOT_MOTOR_CLOCK_microsecondMIN) {
		this->status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMIN;
	}else{
		this->status.cmd.clock =clock;	//ok > registro il nuovo valore di clock
	};

	
	//dbg( "1,ck:", this->status.cmd.clock );

	// 2/7/15 abbandonato timer 3 perchè interferisce con ISR su pin2 dell'encoder
	//Timer3.initialize( status.cmd.clock );	//Timer3 agisce sui pin 2,3,5
	//analogWrite(Pin_MotCK,128);

	/*	

	// 14-9-2015 così non funziona neppure moveCm() ----------------
	FrequencyTimer2::setPeriod(status.cmd.clock/2);	//FrequencyTimer2::setPeriod((unsigned long)status.cmd.clock);
	FrequencyTimer2::enable();
	analogWrite(Pin_MotCK,128);
*/

	//unsigned long mS;
	//mS = this->status.cmd.clock / 1000;

	// 14-9-2015 compila ma il clock non viene generato
	//FlexiTimer2::stop();
	//FlexiTimer2::set( this->status.cmd.clock, 0.000001, _motorCKpulse ); // chiamo _motorCKpulse every 500 1ms "ticks"
	//FlexiTimer2::start();	//	TCNT2 = tcnt2;   TIMSK2 |= (1<<TOIE2);

	
	// questo funziona ma non so come attaccarci una mia ISR (già usato da Tone)
	//unsigned int Hz;
	// non ha più dato errore di compilazione rimuovendo dal main #include  <FrequencyTimer2\FrequencyTimer2.h>	
	unsigned int Hz = (unsigned int)(1000000.0 / this->status.cmd.clock);
	tone(Pin_MotCK,Hz);//funziona  (usa il timer2)
	//Serial.print( "1," ); Serial.print( Hz ); Serial.println( "Hz;" );
	//pwmWrite( Pin_MotCK, Hz );

};
void robot_c::_motorSetCkPulseDelay( motCk_t clock )
{
	// controlla che il clock sia entro i limiti

	if (clock > ROBOT_MOTOR_CLOCK_microsecondMAX) {
		this->status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX;
	}
	else if (clock < ROBOT_MOTOR_CLOCK_microsecondMIN) {
		this->status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMIN;
	}
	else{
		this->status.cmd.clock = clock;	//ok > registro il nuovo valore di clock
	};




};

/// ///////////////////////////////////////////////////////////////////////
//  calcola e imposta  la velocità this->status.cmd.clock sulla base degli step fatti e sul totale da fare
// parte da 1 e termina quando la velocità ha raggiunto quella desiderata targetSpeed o quando supero la metà degli step da fare
//////////////////////////////////////////////////////////////////////////
void robot_c::_motorAccelerate(){
		//1 determino la fase accelerativa
		// 
		motCk_t _ck = this->status.cmd.clock;
		switch (this->status.cmd.accelPhase) {
			case 1:// accelerazione
				// prima metà? --> rimango in accelerazione
				if(this->status.cmd.stepsDone < (this->status.cmd.targetSteps - this->status.cmd.stepsDone) ){
					this->status.cmd.acceleration = ROBOT_MOTOR_CLOCK_ACCEL;// =(int)(2.0 * this->status.cmd.clock) / (4.0 * this->status.cmd.accelSteps + 1);

					_ck -= this->status.cmd.acceleration ; //riduco gli intervalli in microsecondi
					#if  DEBUG
						dbg1("1,AccPh:");
					#endif
					// limito entro la velocità  target
					if ((_ck  < this->status.cmd.targetCK) || (_ck <= ROBOT_MOTOR_CLOCK_microsecondMIN))
					{	
						// raggiunto o superato velocità target 
						_ck = ROBOT_MOTOR_CLOCK_microsecondMIN;//was  this->status.cmd.targetCK; //sì -> la limito
						this->status.cmd.accelPhase = 0; // passo a fase velocità costante 
						//Serial.print("0,"); Serial.println(this->status.cmd.stepsDone);
						//memorizzo gli step fatti, per stabilire quando iniziare la decelerazione
						this->status.cmd.accelSteps = this->status.cmd.stepsDone;
						#if  DEBUG
							dbg1( "1,ConstPh:" );
						#endif
					}

					this->_motorSetCkPulseDelay( _ck );	// aggiorno la velocità
				}
				else{	//superato la metà > non accelero
					// entro in fase di decelerazione
					this->status.cmd.accelPhase = -1;
					#if  DEBUG
						dbg1( "1,DecelPh:" );
					#endif
					// Serial.print("ACC-1,"); Serial.println(this->status.cmd.stepsDone);
				}
				break;
			case 0:// fase a velocità costante

				// mantengo la velocità o decelero ?
				if( (this->status.cmd.targetSteps - this->status.cmd.stepsDone)<= (uint16_t)this->status.cmd.accelSteps ){
					// decelerazione
					this->status.cmd.accelPhase = -1;
					#if  DEBUG
						dbg1( "1,DecelPh:" );
					#endif

				}
				break;

			case -1:		// decelero
				this->status.cmd.acceleration = ROBOT_MOTOR_CLOCK_DECEL;
				_ck += this->status.cmd.acceleration;
				this->_motorSetCkPulseDelay( _ck );	// aggiorno la velocità
				break;
		}
	}
//Versione che calcola il CK senza assegnarlo
motCk_t robot_c::_motorComputeStepDelay( motCk_t CurrentStepDelay ){
	motCk_t NewStepDelay = 0;
	//1 determino la fase accelerativa
	// 
	static	int resto = 0;	//resto nei calcoli per avere maggiore precisione
 
	switch (this->status.cmd.accelPhase) {
	case 1:// accelerazione
		// prima metà? -->  in accelerazione
		if (this->status.cmd.stepsDone < (this->status.cmd.targetSteps - this->status.cmd.stepsDone)){
			//	this->status.cmd.accelPhase = 1;

			NewStepDelay =CurrentStepDelay - (2 * CurrentStepDelay + resto) / (4 * this->status.cmd.accelSteps + 1);
			resto = (2 * CurrentStepDelay + resto)%( 4 * this->status.cmd.accelSteps + 1 );
			dbg( "1,AccPh;" )

			// limito entro la velocità  target
			if (NewStepDelay  < this->status.cmd.targetCK){// raggiunto o superato velocità target ?
				NewStepDelay = this->status.cmd.targetCK; //sì -> la limito
				this->status.cmd.accelPhase = 0; // passo a fase velocità costante 

				//Serial.print("0,"); Serial.println(this->status.cmd.stepsDone);
				//memorizzo gli step fatti, per stabilire quando iniziare la decelerazione
				this->status.cmd.accelSteps = this->status.cmd.stepsDone;
				dbg( "1,ConstPh;" )
			}

		}
		else{	//superato la metà > non accelero
			// entro in fase di decelerazione
			this->status.cmd.accelPhase = -1;
			dbg( "1,DecelPh;" )
			NewStepDelay = CurrentStepDelay;
 
			// Serial.print("ACC-1,"); Serial.println(this->status.cmd.stepsDone);
		}
		break;
	case 0:// fase a velocità costante

		// mantengo la velocità o decelero ?
		if ((this->status.cmd.targetSteps - this->status.cmd.stepsDone) <= (uint16_t)this->status.cmd.accelSteps){
			// passo alla fase di decelerazione
			this->status.cmd.accelPhase = -1;
			dbg( "1,DecelPh;" )

		}
		NewStepDelay=  CurrentStepDelay;
		break;

	case -1:		// decelero

		NewStepDelay = CurrentStepDelay + 8;// (2 * CurrentStepDelay + resto) / (4 * this->status.cmd.accelSteps + 1);
		resto = (2 * CurrentStepDelay + resto) % (4 * this->status.cmd.accelSteps + 1);
		break;
	}
	return NewStepDelay;	// aggiorno la velocità

	
}
/// ///////////////////////////////////////////////////////////////////////
//  movimento avanti o indietro con feedback da encoders
//  ritorna i cm percorsi
//////////////////////////////////////////////////////////////////////////
int robot_c::moveCm( int cm )
{
	if (cm >=0)
	{	this->status.cmd.commandDir = GOF;}	//usato da ObstacleFree()	
	else	
	{	this->status.cmd.commandDir = GOB;	}

	//	limito la distanza massima- a 5 metri ------------------------
	if (cm > MAX_TARGET_DISTANCE_CM){
		this->status.cmd.targetCm = MAX_TARGET_DISTANCE_CM;
	}
	else
		if (cm < -MAX_TARGET_DISTANCE_CM)
		{
			this->status.cmd.targetCm = -MAX_TARGET_DISTANCE_CM;
		}
		else
		{
			this->status.cmd.targetCm = cm;
		}
	//-------------------------------------------------------------------

	//converto i cm in step da percorrere e li memorizzo in status.cmd.targetCm ----------------
	this->status.cmd.targetSteps =  abs(this->status.cmd.targetCm *ROBOT_MOTOR_STEPS_PER_CM);
	#if  DEBUG 
		dbg("1,Target steps: ",this->status.cmd.targetSteps);
	#endif

	// inizializzo gli step
	this->status.cmd.stepsDone = 0;

	// Controllo assenza ostacolo con IRproxy  -------------------

	if (!this->obstacleFree()) {		//if (0) {		// 
		status.cmd.enL =false;
		status.cmd.enR = false;
		SPEAK_OIOI
		
			
		// disattivo i motori-------------------
		MOTORS_DISABLE
		digitalWriteFast( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );
		
		SERIAL_MSG.println( "1,***command aborted. Obstacle found;" );

		return 0;
	} 
	else ///ok posso muovermi --------------------------
	{

		SERIAL_MSG.println( "\n1,Moving...;" );
	

		//Abilito i motori ---------------------
		status.cmd.enL =true;
		status.cmd.enR = true;
		MOTORS_ENABLE
		

		// inizializzo gli encoders se presenti-----------------
		#if OPT_ENCODERS	

				encL.position=0;
				encR.position=0;

				//Encoder EncL(Pin_EncRa,Pin_EncRb);
				//Encoder EncR(Pin_EncRa,Pin_EncRb);
				//EncL.write(0); EncR.write(0);
				//positionRight = 0; positionLeft = 0;

				this->status.EncR = 0; this->status.EncL = 0;
		#endif
		//int encLprec = 0;
		//int encRprec = 0;


		// imposto il verso di rotazione secondo statuscmd
		if (this->status.cmd.targetCm >0)// imposto la direzione avanti
		{
			status.cmd.cwR = false;
			digitalWriteFast( Pin_MotCWR, !ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = true;	
			digitalWriteFast( Pin_MotCWL, ROBOT_MOTORCW_ACTIVE );

		}
		else	// imposto la direzione indietro
		{
			status.cmd.cwR = true;
			digitalWriteFast( Pin_MotCWR, ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = false;
			digitalWriteFast( Pin_MotCWL, !ROBOT_MOTORCW_ACTIVE );

		}
		//--------------------------------------------------

		// attivo i motori-------------------
		digitalWriteFast(Pin_MotENR, ROBOT_MOTORENABLE_ACTIVE);
		digitalWriteFast(Pin_MotENL, ROBOT_MOTORENABLE_ACTIVE);
		//-------------------------------


		// imposto la velocità iniziale alla velocità minima
		status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX;
		//	int stepDelay = ROBOT_MOTOR_CLOCK_microsecondMAX;		// was	status.cmd.targetCK= robot.speedToCK(status.cmd.targetSpeed);
		// imposto il clock TARGET motori-------------------------------------------------------
		//status.cmd.targetCK=(int)ROBOT_MOTOR_CLOCK_PER_CMs/ status.cmd.targetSpeed;
		status.cmd.targetCK = ROBOT_MOTOR_CLOCK_microsecondMIN;
		// inizializzo  accelerazione-----------
		status.cmd.accelPhase = 1;
		//-----------------------------------------------------------------------------


		dbg2("   target clock:",  status.cmd.targetCK); 
		#if DEBUG_ENCODERS 				
				dbg( "   this->status.EncL", this->status.EncL );
		#endif


		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		// ACCELERATION LOOP finchè step fatti<step programmati o presenza ostacolo
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////

		//---------------------------------------------------------
		// loop di accelerazione
		//------------------------------------------------------
		while ((this->status.cmd.stepsDone <= this->status.cmd.targetSteps) && this->obstacleFree())
		{
			_motorCKpulse();

			// accelera o decelera in base agli step percorsi
			_motorAccelerate();

			//stepDelay = _motorComputeStepDelay( status.cmd.clock );
#if  DEBUG 
			//attenzione!! se attivo viene rallentato sensibilmente  il movimento !!
 			dbg2( "ck:", status.cmd.clock );
#endif	
			//status.cmd.clock = stepDelay;// _motorSetCKspeed( stepDelay );
			delayMicroseconds( status.cmd.clock );
		}// end while
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////

		// step completati oppure ostacolo -----------------
		robot.stop();


		#if DEBUG
			dbg2("End Loop stepsDone:" , status.cmd.stepsDone);
			#if DEBUG_ENCODERS
				//-- visualizzo i risultati
				dbg2( "EncR:", status.EncR );
			#endif
		#endif		

		int cmPercorsi = (int)(status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_CM);


		//-----------------------------------------------------------
		//Serial.print( "steps :" ); Serial.println( status.cmd.stepsDone );
		// ritorno i cm percorsi -----------------------------------------------

		// registro il movimento eseguito
//		pushPathHistory(status.cmd.commandDir, cmPercorsi);
//		updatePose(cmPercorsi, 0);// update Robot position

		return cmPercorsi;
	};
	
}	// end move
/// ///////////////////////////////////////////////////////////////////////
//  movimento rotatorio CW/CCW  con feedback da encoders
//  ritorna angolo percorso in radianti
/// ///////////////////////////////////////////////////////////////////////
long robot_c::rotateSteps(long steps)
{
	if (steps >=0) 	{status.cmd.commandDir = GOR;}	//usato da ObstacleFree()
		else		{status.cmd.commandDir = GOL;}
	
	if (!this->obstacleFree()) {		//if (0) {		// 
		status.cmd.enL = false;
		status.cmd.enR = false;
		SPEAK_OIOI
		// disattivo i motori-------------------
		digitalWriteFast( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );

		
		dbg( "1,***command aborted. Obstacle found;" );
		
		return 0;
	}
	else ///ok posso muovermi --------------------------
	{

		//converto i radianti in step----------------------------
		this->status.cmd.targetSteps = abs(steps);
 
		// to do: Modulo 2PI
		dbg2( "1,Target steps: ", this->status.cmd.targetSteps );
		
		//-------------------------------------------------

		// inizializzo gli step-----------------
		this->status.cmd.stepsDone = 0;

		// inizializzo encoders-----------------
		//int EncLprec = 0;
		//int EncRprec =0;
		#if OPT_ENCODERS
			this->status.EncR=0; this->status.EncL=0;
		#endif
		//--------------------------------------


		// imposto il verso di rotazione secondo statuscmd
		if (steps>0)
		{
			status.cmd.cwR = true;
			digitalWriteFast( Pin_MotCWR, ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = true;
			digitalWriteFast( Pin_MotCWL, ROBOT_MOTORCW_ACTIVE );
		}	
		else
		{
			status.cmd.cwR = false;
			digitalWriteFast( Pin_MotCWR, !ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = false;
			digitalWriteFast( Pin_MotCWL, !ROBOT_MOTORCW_ACTIVE );
		}
		//--------------------------------------------------



		// attivo i motori-------------------
		digitalWriteFast( Pin_MotENR, ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENL, ROBOT_MOTORENABLE_ACTIVE );
		//-------------------------------

		// imposto la velocità iniziale alla velocità minima
		status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX;
		//int stepDelay = ROBOT_MOTOR_CLOCK_microsecondMAX;		// was	status.cmd.targetCK= robot.speedToCK(status.cmd.targetSpeed);
		// imposto il clock TARGET motori-------------------------------------------------------
		//status.cmd.targetCK=(int)ROBOT_MOTOR_CLOCK_PER_CMs/ status.cmd.targetSpeed;
		status.cmd.targetCK = ROBOT_MOTOR_CLOCK_microsecondMIN;
		// inizializzo  accelerazione-----------
		status.cmd.accelPhase = 1;
		//-----------------------------------------------------------------------------



		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		// ACCELERATION LOOP finchè step fatti<step programmati o presenza ostacolo
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		while ((this->status.cmd.stepsDone <= this->status.cmd.targetSteps) && this->obstacleFree()){

			_motorCKpulse();

			// accelera o decelera in base agli step percorsi
			_motorAccelerate();

			//stepDelay = _motorComputeStepDelay( status.cmd.clock );
			#if  DEBUG 
						dbg( "ck:", status.cmd.clock );
			#endif	
			//status.cmd.clock = stepDelay;// _motorSetCKspeed( stepDelay );
			delayMicroseconds( status.cmd.clock );

			//	// memorizzo i valori per il confronto al giro successivo per vedere se sono cambiati
			//	EncLprec =this->status.EncL;
			//	EncRprec =this->status.EncR;
			//
			//
			//	// attendo---------------------------------------------------------------------------
			//	delay(ACCEL_LOOP_MICROS_WAIT);
			//			
			//	// se non uso gli encoder è la procedura che genera il clock motori a incrementare in anello aperto status.cmd.stepsDone
			//	#if OPT_ENCODERS
			//	//lettura encoders---------------------------------------------------------------
			//	noInterrupts();
			//	this->status.EncR = abs( encR.position);//this->status.EncR= abs( EncR.getPosition());	//was  this->status.EncR= abs( EncR.read());
			//	this->status.EncL = abs( encL.position);	//this->status.EncL =  abs( EncL.getPosition());//was	 this->status.EncL =  abs( EncL.read());
			//	interrupts();
			//	#if DEBUG_ENCODERS
			//	// invio solo se i valori sono cambiati
			//	if (this->status.EncL != EncLprec || this->status.EncR != EncRprec) {
			//		dbg( "EncL: %d",this->status.EncL);
			//		dbg(" R: %d",this->status.EncR);
			//	}
			//	#endif
			//	
			//	
			//	// converto in steps---------------------------------------------------------------
			//	this->status.cmd.stepsDone =this->status.EncL* ROBOT_MOTOR_STEPS_PER_ENCODERTICK;// (this->status.EncR + this->status.EncL )/2;
			//	#endif
			//
			//// accelera o decelera---------------------------------------------------------------------------
			//if (hasExpired(previousToggleAccel,ACCEL_LOOP_MICROS_WAIT)){		//10: buona ; anche 5 è buona (con accel.=4)
			//	robot._motorAccelerate();
			//				
			//	#if DEBUG_ENCODERS	 || DEBUG
			//	//	dbg("..stepsDone: ",this->status.cmd.stepsDone);
			//	#endif
			//}
			////---------------------------------------------------------------------------------------------------------




		}// end while

	}
	// step completati oppure ostacolo -----------------
	robot.stop();
 
	dbg2( "End Loop stepsDone:", status.cmd.stepsDone );
	#if DEBUG_ENCODERS
		//-- visualizzo i risultati -----------------------------------
		dbg2("Step percorsi: L:" ,status.EncL)
		dbg2(", R:" ,status.EncR)
	#endif
 	
	//-----------------------------------------------------------

	// ritorno step percorsi ----------------------------------------------- 
	return  (long)status.cmd.stepsDone ;
};


float robot_c::rotateRadiants(float rad)
{
	if (rad>=0)
	{
		status.cmd.commandDir = GOR;	//usato da ObstacleFree()
	}
	else
	{
		status.cmd.commandDir = GOL;
	}
	
	if (!this->obstacleFree()) {		//if (0) {		// 
		status.cmd.enL = false;
		status.cmd.enR = false;
		// disattivo i motori-------------------
		digitalWriteFast( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );

		#if  DEBUG 
				dbg( "1,***command aborted. Obstacle found;", );
		#endif
		return 0;
	}
	else ///ok posso muovermi --------------------------
	{

		//converto i radianti in step----------------------------
		if (rad >0){
			this->status.cmd.targetSteps =  rad *ROBOT_MOTOR_STEPS_PER_RADIANT;
		}
		else{
			this->status.cmd.targetSteps =  -rad *ROBOT_MOTOR_STEPS_PER_RADIANT;
		}
		// to do: Modulo 2PI
		#if  DEBUG 
			dbg( "1,Target steps: ", this->status.cmd.targetSteps );
		#endif
		//-------------------------------------------------

		// inizializzo gli step-----------------
		this->status.cmd.stepsDone = 0;

		// inizializzo encoders-----------------
		//int EncLprec = 0;
		//int EncRprec =0;
		#if OPT_ENCODERS
			this->status.EncR=0; this->status.EncL=0;
		#endif
		//--------------------------------------


		// imposto il verso di rotazione secondo statuscmd
		if (rad>0)
		{
			status.cmd.cwR = true;
			digitalWriteFast( Pin_MotCWR, ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = true;
			digitalWriteFast( Pin_MotCWL, ROBOT_MOTORCW_ACTIVE );
		}	
		else
		{
			status.cmd.cwR = false;
			digitalWriteFast( Pin_MotCWR, !ROBOT_MOTORCW_ACTIVE );	//LOW = CW	(ok)
			status.cmd.cwL = false;
			digitalWriteFast( Pin_MotCWL, !ROBOT_MOTORCW_ACTIVE );
		}
		//--------------------------------------------------



		// attivo i motori-------------------
		digitalWriteFast( Pin_MotENR, ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENL, ROBOT_MOTORENABLE_ACTIVE );
		//-------------------------------

		// imposto la velocità iniziale alla velocità minima
		//int ck = ROBOT_MOTOR_CLOCK_microsecondMAX;// default su velocità minima
		status.cmd.clock = ROBOT_MOTOR_CLOCK_microsecondMAX;
		//int stepDelay = ROBOT_MOTOR_CLOCK_microsecondMAX;		// was	status.cmd.targetCK= robot.speedToCK(status.cmd.targetSpeed);
		// imposto il clock TARGET motori-------------------------------------------------------
		//status.cmd.targetCK=(int)ROBOT_MOTOR_CLOCK_PER_CMs/ status.cmd.targetSpeed;
		status.cmd.targetCK = ROBOT_MOTOR_CLOCK_microsecondMIN;
		// inizializzo  accelerazione-----------
		status.cmd.accelPhase = 1;
		//-----------------------------------------------------------------------------



		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		// ACCELERATION LOOP finchè step fatti<step programmati o presenza ostacolo
		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////
		while ((this->status.cmd.stepsDone <= this->status.cmd.targetSteps) && this->obstacleFree()){

			_motorCKpulse();

			// accelera o decelera in base agli step percorsi
			_motorAccelerate();

			//stepDelay = _motorComputeStepDelay( status.cmd.clock );
			#if  DEBUG 
						dbg( "ck:", status.cmd.clock );
			#endif	
			//status.cmd.clock = stepDelay;// _motorSetCKspeed( stepDelay );
			delayMicroseconds( status.cmd.clock );

			//	// memorizzo i valori per il confronto al giro successivo per vedere se sono cambiati
			//	EncLprec =this->status.EncL;
			//	EncRprec =this->status.EncR;
			//
			//
			//	// attendo---------------------------------------------------------------------------
			//	delay(ACCEL_LOOP_MICROS_WAIT);
			//			
			//	// se non uso gli encoder è la procedura che genera il clock motori a incrementare in anello aperto status.cmd.stepsDone
			//	#if OPT_ENCODERS
			//	//lettura encoders---------------------------------------------------------------
			//	noInterrupts();
			//	this->status.EncR = abs( encR.position);//this->status.EncR= abs( EncR.getPosition());	//was  this->status.EncR= abs( EncR.read());
			//	this->status.EncL = abs( encL.position);	//this->status.EncL =  abs( EncL.getPosition());//was	 this->status.EncL =  abs( EncL.read());
			//	interrupts();
			//	#if DEBUG_ENCODERS
			//	// invio solo se i valori sono cambiati
			//	if (this->status.EncL != EncLprec || this->status.EncR != EncRprec) {
			//		dbg( "EncL: %d",this->status.EncL);
			//		dbg(" R: %d",this->status.EncR);
			//	}
			//	#endif
			//	
			//	
			//	// converto in steps---------------------------------------------------------------
			//	this->status.cmd.stepsDone =this->status.EncL* ROBOT_MOTOR_STEPS_PER_ENCODERTICK;// (this->status.EncR + this->status.EncL )/2;
			//	#endif
			//
			//// accelera o decelera---------------------------------------------------------------------------
			//if (hasExpired(previousToggleAccel,ACCEL_LOOP_MICROS_WAIT)){		//10: buona ; anche 5 è buona (con accel.=4)
			//	robot._motorAccelerate();
			//				
			//	#if DEBUG_ENCODERS	 || DEBUG
			//	//	dbg("..stepsDone: ",this->status.cmd.stepsDone);
			//	#endif
			//}
			////---------------------------------------------------------------------------------------------------------




		}// end while

	}
	// step completati oppure ostacolo -----------------
	robot.stop();
	#if DEBUG
		dbg( "End Loop stepsDone:", status.cmd.stepsDone );
		#if DEBUG_ENCODERS
			//-- visualizzo i risultati -----------------------------------
			Serial.print("Step percorsi: L:" ); Serial.print(status.EncL);
			Serial.print(", R:" ); Serial.println(status.EncR);
		#endif
	#endif	
	//-----------------------------------------------------------

	// ritorno i radianti percorsi -----------------------------------------------

	float radPerformed = (float)status.cmd.stepsDone / ROBOT_MOTOR_STEPS_PER_RADIANT;
	updatePose(0, (int)radPerformed *57.29578); //180/pi=57.29578

	return radPerformed;
}

// deg > 0 Clockwise, deg<0 CCW
int robot_c::rotateDeg( int deg ){

	//float rad = deg * DEG_TO_RAD;
	//int radDone = 0;
	//radDone = rotateRadiants( rad );


	long steps = deg *ROBOT_MOTOR_STEPS_PER_DEG/2;
	long stepsdone = 0;
	stepsdone = rotateSteps(steps);
	return (int)(stepsdone / ROBOT_MOTOR_STEPS_PER_DEG);
	updatePose( 0, (int)(stepsdone / ROBOT_MOTOR_STEPS_PER_DEG));


}
void robot_c::motorsOnOff( bool On ){
	digitalWriteFast( Pin_MotENL, On );
	digitalWriteFast( Pin_MotENR, On );
}
//abilita i motori e il clock via Timer
void robot_c::_go(motCk_t clock)
{	//trasferisce i comandi alle porte di I/O
	//disabilito i motori
	//motorsOnOff( 0 );

	// imposto le direzioni secondo statuscmd
	digitalWriteFast( Pin_MotCWR, status.cmd.cwR );	//LOW = CW	(ok)
	digitalWriteFast( Pin_MotCWL, status.cmd.cwL );

	// clock -----------
	//_motorSetPWMCK( clock );

	digitalWriteFast( Pin_MotENL, status.cmd.enL );
	digitalWriteFast( Pin_MotENR, status.cmd.enR );


	/* così non va bene perchè non accetta i nuovi comandi*/
	while ( this->obstacleFree())
	{ 

		_motorCKpulse();		
		_motorAccelerate();// accelera o decelera in base agli step percorsi
		delayMicroseconds( status.cmd.clock );
	}// end while
	

};

void robot_c::stop(){
		status.cmd.commandDir =STOP;
		status.cmd.enL= 0;
		status.cmd.enR=0;
		digitalWriteFast( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );
		digitalWriteFast( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );

};
/// goFW,goBK,goCCW,goCC non devono essere bloccanti
void robot_c::goFW(motCk_t clock){
//		EncL.write(0);EncR.write(0);// inizializzo gli encoder
		status.cmd.commandDir =GOF;
		status.sensors.irproxy.fw = digitalReadFast(Pin_irproxy_FW); //demandato al metodo robot.readSensors() chiamato da task
		if (this->obstacleFree()==false) {
			status.cmd.enL =false;
			status.cmd.enR =false;
		} else {
			status.cmd.enL =true;	// abilito i driver dei due motori
			status.cmd.enR = true;	//
			status.cmd.cwL =true;	// imposto la direzione
			status.cmd.cwR = false;
			_motorSetPWMCK( clock );
		}
	};
void robot_c::goBK(motCk_t clock){
//			EncL.write(0);EncR.write(0);// inizializzo gli encoder

		status.cmd.commandDir =GOB;
		
		if (this->obstacleFree()==false) {
			status.cmd.enL =false;
			status.cmd.enR =false;
		} else {	
			status.cmd.enL = true;
			status.cmd.enR = true;
			status.cmd.cwL = false;
			status.cmd.cwR = true;

			_motorSetPWMCK( clock );
		}
	};
void robot_c::goCCW(motCk_t clock){
//			EncL.write(0);EncR.write(0);// inizializzo gli encoder

		status.cmd.commandDir =GOL;
		status.cmd.enL = true;
		status.cmd.enR = true;
		status.cmd.cwL = false;
		status.cmd.cwR = false;

		_motorSetPWMCK( clock );
	};
void robot_c::goCW(motCk_t clock){
//			EncL.write(0);EncR.write(0);// inizializzo gli encoder

		status.cmd.commandDir =GOR;
		status.cmd.enL = true;
		status.cmd.enR = true;
		status.cmd.cwL = true;
		status.cmd.cwR = true;

		_motorSetPWMCK( clock );
	};

/// ///////////////////////////////////////////////////////////////////////
//  Esegue la lettura dei sensori :irproxy , pirDome, analog, rele
/// ///////////////////////////////////////////////////////////////////////
void robot_c::readGps() {
	while (SERIAL_GPS.available() > 0)	Gps.encode(SERIAL_GPS.read());
	if (Gps.location.isUpdated()) {
		status.sensors.gps.sats = Gps.satellites.value();
		status.sensors.gps.lat = Gps.location.lat();
		status.sensors.gps.lng = Gps.location.lng();
		status.sensors.gps.alt = Gps.altitude.meters();
		status.sensors.gps.homeDistCm = Gps.distanceBetween(status.sensors.gps.lat, status.sensors.gps.lng, GPSHOME_LAT, GPSHOME_LNG);
		status.sensors.gps.homeAngleDeg =Gps.courseTo(status.sensors.gps.lat, status.sensors.gps.lng, GPSHOME_LAT, GPSHOME_LNG);
	}
}

/// ///////////////////////////////////////////////////////////////////////
// Esegue la lettura di tutti i sensori,
// memorizza i valori in 'status'
// memorizzando i valori precedenti in 'statusOld'
// richiede che i puntatori agli oggetti Sonar, Servo LDS e GPS siano stati impostati
/// ///////////////////////////////////////////////////////////////////////
void robot_c::readSensors(){

	statusOld = status;

	readIrProxySensors();
	readBumpers();
	status.act.laserOn = digitalReadFast(Pin_LaserOn);
	status.sensors.switchTop = digitalReadFast( Pin_SwitchTop );
	status.sensors.pirDome = digitalReadFast( Pin_PIR1 );
	status.act.rele[0]		= getReleStatus(0);
	status.act.rele[1]		= getReleStatus(1);
	status.sensors.analog[Pin_AnaPot1-Pin_AnaBase]	= analogRead(Pin_AnaPot1);	//A0 potenziometro  //116 uS per la lettura
	status.sensors.analog[Pin_AnaVbat-Pin_AnaBase]	= analogRead(Pin_AnaVbat);	// A1 Vbat
	status.sensors.analog[Pin_AnaLight-Pin_AnaBase]= analogRead(Pin_AnaLight);	// A2 Luce
	status.posCurrent.r = readCompassDeg();
	readGps();
//	status.posCurrent.r = readCompassDeg();

#if 	OPT_COMPASS
#endif // 	OPT_COMPASS
//	status.posCurrent.r = readCompassDeg();



	status.ts = millis();
	if (status.sensors.switchTop ==1)
	{
		status.operatingMode = MODE_AUTONOMOUS;
	}
	else
	{
		status.operatingMode = MODE_SLAVE;

	}
};
	//
void robot_c::readIrProxySensors(){
	status.sensors.irproxy.fw = !digitalReadFast( Pin_irproxy_FW );
	status.sensors.irproxy.fwHL = !digitalReadFast( Pin_irproxy_FWHL );
	status.sensors.irproxy.bk = !digitalReadFast( Pin_irproxy_BK );
	status.sensors.irproxy.fr = !digitalReadFast( Pin_irproxy_FR );
	status.sensors.irproxy.fl = !digitalReadFast( Pin_irproxy_FL );


	};
void robot_c::readBumpers(){
	status.sensors.bumper.right = !digitalReadFast(Pin_BumbRight);
	status.sensors.bumper.center = !digitalReadFast(Pin_BumbCenter);
	status.sensors.bumper.left = !digitalReadFast(Pin_BumbLeft);
	};
//int robot_c::ping_cm() {
//	digitalWriteFast( Pin_SonarTrig, LOW );
//	delayMicroseconds( 4);	
//	digitalWriteFast( Pin_SonarTrig, HIGH );
//	delayMicroseconds( 10 );
//	digitalWriteFast( Pin_SonarTrig, LOW );
//
//	//long pulseTime = pulseIn(  Pin_SonarEcho , HIGH );
//
//
//	//while (digitalReadFast( Pin_SonarEcho );                      // Wait for the ping echo.
//	//	if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
//	//return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
//
//	unsigned int echoTime = Sonar.ping();          // Calls the ping method and returns with the ping echo distance in uS.
//	return NewPingConvert( echoTime, US_ROUNDTRIP_CM ); // Convert uS to centimeters.
//
//}

//////////////////////////////////////////////////////////////////////////

/**
 * \Attiva o disattiva uno dei Rele  
 * 
 *  0=Rele1
 *  onoff: tiene conto delle logica negata
 * 
 * \return void
 */
void robot_c::setRele(int16_t releNumber, int16_t onoff){
		switch (releNumber){
			case 1:

				digitalWriteFast( Pin_Rele1, !onoff);
				break;
			case 2:
				digitalWriteFast( Pin_Rele2, !onoff );
				break;
			default:
				break;
			//non fa nulla
		}
	};
/**
 * \ritorna lo stato dei rele
 * 
 * \param releNumber: 1 o 2
 * 
 * \return bool
 */
bool robot_c::getReleStatus(int releNumber){// base  0
		switch (releNumber){
			case 0:
				return !digitalReadFast( Pin_Rele1 );
				break;
			case 1:
				return !digitalReadFast( Pin_Rele2 );
				break;
			default:
				return -1;
				break;
			//non fa nulla
		}
	}

/*
//////////////////////////////////////////////////////////////////////////
/// converte la velocità in cm/s in ampiezza impulso di clock
//////////////////////////////////////////////////////////////////////////
motCk robot_c::speedToCK(float cmPerSecond){
	return (uint32_t)ROBOT_MOTOR_CLOCK_PER_CMs/ cmPerSecond;
	}
*/


///////////////////////////////////////////////////////////////////////////////////////
///  ritorna true se i sensori sono alimentati   
///////////////////////////////////////////////////////////////////////////////////////
bool robot_c::isPowerOn(){
	this->readSensors();
	//dbg("status.analog[1]  %d ",(int)this->status.analog[1]);	
	//dbg("status.sensors.irproxy.fw  %d ",(int)this->status.sensors.irproxy.fw);
	//dbg("status.sensors.irproxy.fwHL  %d ",(int)this->status.sensors.irproxy.fwHL);
	//dbg("status.sensors.irproxy.bk  %d ",(int)this->status.sensors.irproxy.bk);
	//dbg("status.pirDome  %d ",(int)this->status.pirDome);
	if ((this->status.sensors.analog[1]==0)
		&& this->status.sensors.irproxy.fw  
		&& this->status.sensors.irproxy.bk 
		&& this->status.sensors.irproxy.fwHL
		&& !this->status.sensors.pirDome){
		return false;
		}
	else
	{
		dbg2("OK Main Power is ",1)
		return true;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//  Legge gli encoder
/////////////////////////////////////////////////////////////////////////////////////
#if 0
int robot_c::readEncoderL(){
	return EncL.read();
	};
int robot_c::readEncoderR(){
	return EncR.read();
		};
#endif

//////////////////////////////////////////////////////////////////////////
/// ritorna la percentuale di carica della batteria
//////////////////////////////////////////////////////////////////////////	
int robot_c::readBattChargeLevel(){
//	#define ADCconversionFactor 0.004887585532746823069403714565 
//	#define VbatRatio 2.843   //calibrato con Voltmetro ai morsetti della batteria

	#define ADV2Vbat	0.01389540 // =VbatRatio*ADCconversionFactor
	double Vbat;
	this->status.sensors.analog[Pin_AnaVbat-Pin_AnaBase] = analogRead(Pin_AnaVbat);
	//A1_Voltage = ADCValue * ADCconversionFactor ; //Vbat 13.9 >> Va1 = 4,7v >> ADC =1018

	Vbat=  this->status.sensors.analog[Pin_AnaVbat-Pin_AnaBase]  * ADV2Vbat;

	int c;
	c=0;
  if (Vbat > 12.9){ //2.15v  per elemento = 100%
    c=  100 ;
    }else
      if (Vbat > 12.6) { //2.1v  per elemento = 80%
        c=80;
        }else 
          if (Vbat > 12.3) { //2.05v  per elemento = 60%
            c=60;
          }else 
            if (Vbat > 12.18) { //2.03v  per elemento = 50%
              c=50;
             }else 
                if (Vbat > 11.88) { //1.983v  per elemento = 40%
                  c=40;
                  }else 
                    if (Vbat > 11.7) { //1.983v  per elemento = 30%
                      c=30;
                      }else 
                        if (Vbat > 11.46) { //1.913v  per elemento = 20%
                          c=20;
                         }else 
                             if (Vbat > 11.1) { //1.853v  per elemento = 10%
                                c=10;
                             }else 
                                c=0;
	return c;
}

//////////////////////////////////////////////////////////////////////////
/// Imposta la modalità operativa
//////////////////////////////////////////////////////////////////////////	
//void robot_c::SetMode(operatingMode_e mode){
//	this->operatingMode = mode;
//
//	}
//////////////////////////////////////////////////////////////////////////
/// esegue Initial Built In Test facendo una pausa di [delayms] tra ogni test
//////////////////////////////////////////////////////////////////////////	
void robot_c::runIBIT( int delayms )
{

	SPEAK_TEST

	#define TestDist 10
	blinkLed( Pin_LaserOn, 1000 );

	blinkLed( Pin_LED_TOP_R, 800 );
	// accende e spegne i RELE
	this->setRele(1,1);
	delay(delayms);
	this->setRele(1,0);
	delay(delayms);

	this->setRele(2,1);
	delay(delayms);
	this->setRele(2,0);
	delay(delayms);

	//-----------------------------------------
	// Swwep di Test del  Servo
	//----------------------------------------- 
	if (_pServoSonar != 0)
	{
		LASER_ON
		delay( 500 );
		SERIAL_MSG.print( F("1,test Servo 0-180-90;") );
		_pServoSonar->attach( Pin_ServoSonarPWM );

		SPEAK("servo")
		_pServoSonar->write( 0 );	// angolo in gradi se < 360
		delay( 500 );
		_pServoSonar->write( 180 );
		delay( 200 );
		_pServoSonar->write( 90 );
		_pServoSonar->detach();
		delay( 200 );
		int d = 0;
		SERIAL_MSG.println("1,testing Laser Distance sensor...;");
		for (byte i = 0; i < 10; i++)
		{
			d = robot_c::getLDSDistance( );
			SERIAL_MSG.print("  distance: ");SERIAL_MSG.println(d);
			delay( 500 );
		}



		//_pServoSonar->write( status.parameters.sonarStartAngle );	// angolo in gradi se < 360
		//delay( 500 );
		//_pServoSonar->write( status.parameters.sonarStartAngle + status.parameters.sonarScanAngle );
		//delay( 500 );
		//_pServoSonar->write( status.parameters.sonarStartAngle );

		//delay( 2000 );
		//_pServoSonar->write( 90 );	// si riposiziona al centro


		//delay( 2000 );
		//LaserOff;

	}
	else
	{
		SERIAL_MSG.print( "1, Error servo pointer is null!!;" );
	}

	SERIAL_MSG.print( "1,test move FW ;" );
	this->moveCm( TestDist );
	delay(delayms);

	blinkLed( Pin_LED_TOP, 500 );

	SERIAL_MSG.print( "1,test move BK ;" );
	this->moveCm( -TestDist );
	delay(delayms);

	blinkLed( Pin_LED_TOP, 500 );

	//// Ruota a destra e sinistra
	//#define IBIT_ROTATION_DEG 90
	//SERIAL_MSG.print( "1,test rot CW ;" );
	//this->rotateDeg(IBIT_ROTATION_DEG);
	//delay(delayms);

	//blinkLed( Pin_LED_TOP, 500 );

	//SERIAL_MSG.print( "1,test rot CCW ;" );
	//this->rotateDeg( -IBIT_ROTATION_DEG);
	//delay(delayms);
	

	SPEAK_OK
	


}
//////////////////////////////////////////////////////////////////////////
/// controlla che non ci siano ostacoli durante il movimento
//////////////////////////////////////////////////////////////////////////	
//void robot_c::delayUntilObstacles(){
	//while (!this->status.cmd.commandDir= STOP && this->obstacleFree())
	//{
		//delayMicroseconds(1000);
	//}
	//this->stop()
	//
//}

///////////////////////////////////////////////////////////////////////////////////////
/// Legge sensori di prossimità e ritorna true se non ci sono ostacoli nella direzione del moto
///////////////////////////////////////////////////////////////////////////////////////
bool robot_c::obstacleFree()
	{
		bool isFree = true;
		readIrProxySensors();
		readBumpers();
		if (this->status.cmd.commandDir == GOF)
		{
			// movimento in avanti, controllo avanti
			if (this->status.sensors.analog[Pin_AnaLight - Pin_AnaBase] > 500)  //tanta luce
			{
				isFree = (!this->status.sensors.irproxy.fwHL) &&
					(!this->status.sensors.bumper.center) && (!status.sensors.bumper.right) && (!status.sensors.bumper.left);
						
				if(!isFree){SPEAK("o") }
				return isFree;
			}
			else  //poca luce
			{
				isFree = !this->status.sensors.irproxy.fw && !this->status.sensors.irproxy.fr && !this->status.sensors.irproxy.fl &&
					(!this->status.sensors.bumper.center) && (!status.sensors.bumper.right) && (!status.sensors.bumper.left);
				if (!isFree) { SPEAK("o") }
				return isFree;
			}

		}
		else// non sto andando in avanti
		{
			if (this->status.cmd.commandDir == GOB) 
			{
				// movimento indietro, controllo indietro
 				isFree = !this->status.sensors.irproxy.bk;
				if (!isFree) { SPEAK("o") }
				return isFree;

			}
			else 
			{	// CW o CCW
				// in caso di rotazione fermo solo se entrambi i proxy anteriore e posteriore indicano ostacolo
 
				isFree = !(this->status.sensors.irproxy.fw && this->status.sensors.irproxy.bk);
				if (!isFree) { SPEAK("o") }
				return isFree;

			}

		}
}

void robot_c::blinkLed( int port, int mSec )
{
	digitalWriteFast( port, 0 );
	delay( mSec );	// yield(); //	 Sleep for 50 milliseconds.
	digitalWriteFast( port, 255 );
	delay( mSec );	// yield(); //	 Sleep for 50 milliseconds.
}
#if 1	//OPT_COMPASS
#endif
	int robot_c::readCompassDeg() {

		/* Get a new sensor event */
		sensors_event_t event;
		_pCompass->getEvent(&event);

 
		// Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
		// Calculate heading when the magnetometer is level, then correct for signs of axis.
		float heading = atan2(event.magnetic.y, event.magnetic.x);



		// Set declination angle on your location and fix heading
		// You can find your declination on: http://magnetic-declination.com/
		// (+) Positive or (-) for negative
		// For Bytom / Poland declination angle is 4'26E (positive)
		// Formula: (deg + (min / 60.0)) / (180 / M_PI);
		float declinationAngle = (2.0 + (22.0 / 60.0)) / (180 / M_PI); //ok per milano
		heading += declinationAngle;

		// Correct for heading < 0deg and heading > 360deg
		if (heading < 0)
		{
			heading += 2 * PI;
		}

		if (heading > 2 * PI)
		{
			heading -= 2 * PI;
		}

		// Convert to degrees
		float headingDegrees = heading * 180 / M_PI;
		return (int)headingDegrees;
	}
	int robot_c::readCompassDeg(COMPASS_CLASS *pCompass) {
		this->_pCompass = pCompass;
		/* Get a new sensor event */
		sensors_event_t event;
		pCompass->getEvent(&event);


		// Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
		// Calculate heading when the magnetometer is level, then correct for signs of axis.
		float heading = atan2(event.magnetic.y, event.magnetic.x);



		// Set declination angle on your location and fix heading
		// You can find your declination on: http://magnetic-declination.com/
		// (+) Positive or (-) for negative
		// For Bytom / Poland declination angle is 4'26E (positive)
		// Formula: (deg + (min / 60.0)) / (180 / M_PI);
		//float declinationAngle = (2.0 + (0.36666667)) / (180 / M_PI); //ok per milano
		heading += 2.0064;	// declinationAngle;

		// Correct for heading < 0deg and heading > 360deg
		if (heading < 0)
		{
			heading += 2 * PI;
		}

		if (heading > 2 * PI)
		{
			heading -= 2 * PI;
		}

		// Convert to degrees
		float headingDegrees = heading * 180 / M_PI;
		return (int)headingDegrees;

	}

//void robot_c::pushPathHistory(commandDir_e dir, int val) {
//	// push the numbers to the stack.
//	motionHistory_t motionHist;
//	motionHist.cmdDir = dir;
//	motionHist.val = val;
//	stack.push(motionHist);
//
//}
// imposta la posizione e angolo corrente
//void robot_c::setPose(long x, long y, int r) {
//	posCurrent.x = x;
//	posCurrent.y = y;
//	posCurrent.r = r;
//}
// imposta la posizione e angolo corrente in base a distanza e delta angolo 
// chiamata da MoveCm con updatePose(cmPercorsi,0)
// e da rotateDeg con updatePose(0,rotation)
//void robot_c::updatePose(long dist, int alfa) {
//	posCurrent.x += dist*cos(alfa);
//	posCurrent.y += dist*sin(alfa);
//	posCurrent.r += alfa;
//}

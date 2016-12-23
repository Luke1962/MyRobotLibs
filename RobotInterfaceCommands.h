/* Robot Interface Commands */

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
//#include <CmdMessenger2.h>  // CmdMessenger2
#include <CmdMessenger/CmdMessenger.h>
#include <robot/robot.h>
#include "Commands_Enum.h"
extern struct robot_c robot;
//Dichiarazione di funzione che punta all'indirizzo zero
void( *Riavvia )(void) = 0;






//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  C A L L B A C K S				//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
 
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  G E N E R A L E
//////////////////////////////////////////////////////////////////////////
void cmdMsg( String msg ){
//	cmdWiFi.sendCmd( Msg, msg );
	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
}
void OnCmdReboot(){
	//reset software
	cmdMsg( "Riavvio..." );
	Riavvia();
//	software_Reboot();

}
//////////////////////////////////////////////////////////////////////////
/// Modalità operativa : MODE_SLAVE , JOYSTICK , AUTONOMOUS
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotSetMode(){
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	robot.SetMode((operatingMode_e)cmdWiFi.readInt16Arg());
 
	switch (robot.operatingMode)
	{
		case MODE_SLAVE:
			cmdWiFi.sendCmd( CmdRobotSetMode, MODE_SLAVE );
			SPEAK_MODE_SLAVE
			cmdMsg("SetMode MODE_SLAVE"); 
			break;
		case JOYSTICK:
			cmdWiFi.sendCmd( CmdRobotSetMode, JOYSTICK );
			 
			cmdMsg("SetMode JOYSTICK");
			break;
		case AUTONOMOUS:
			cmdWiFi.sendCmd( CmdRobotSetMode, AUTONOMOUS );
			 SPEAK_AUTONOMO
			cmdMsg( "SetMode AUTONOMOUS" );
			break;	
 	
		default:
			cmdWiFi.sendCmd(Msg,"Unrecognised Mode");			 
			break;
	}
}
void OnUnknownCommand()
{
	SPEAK("NON HO CAPITO")
	String s;
	cmdWiFi.readStringArg();
//	cmdWiFi.sendCmd(kError,"\nCommand not recognised");
	cmdWiFi.sendCmdStart(Msg);
	cmdWiFi.sendCmdArg("Cmd [");
	cmdWiFi.sendCmdArg(s );
	cmdWiFi.sendCmdArg("] unkwown");
	cmdWiFi.sendCmdEnd();
}
void OnCmdRobotHello()
{
	cmdMsg("Hello I'm ready");
}
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   M O V I M E N T O
//////////////////////////////////////////////////////////////////////////
// Avanti o indietro di x cm --------------------------------------------
void OnCmdRobotMoveCm()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	String s;
	int cmPercorsi=0;
	int dist = cmdWiFi.readInt16Arg();

	s="Ok OnCmdRobotMoveCm: " + dist ;

	cmdMsg( s );
	cmdWiFi.sendCmd(Msg,dist);

	//eseguo lo spostamento
	cmPercorsi=robot.moveCm(dist);

	// riporto la distanza percorsa-----------
	if(dist>0){
		s = "Moved Forward: ";
	}
	else{
		s = "Moved Back: ";			
	}
 
	s+=   robot.currCommand.stepsDone;
	s+= " of " + robot.currCommand.targetSteps ;
	cmdMsg(s);
	//-----------------------------------------


	// riporto la distanza percorsa-----------
	cmdWiFi.sendCmdStart( kbRotationRad );
	cmdWiFi.sendCmdArg( cmPercorsi );
	cmdWiFi.sendCmdEnd();
	//-----------------------------------------
}
 // ROTAZIONE IN RADIANTI --------------------------------------
void OnCmdRobotRotateRadiants()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }
	float rad = cmdWiFi.readFloatArg();
	String s = "OK rotateRadiants: " ;
	cmdMsg( s );

	float RadPercorsi = 0.0;
	RadPercorsi = robot.rotateRadiants( rad );

	//dtostrf( RadPercorsi, 7, 3, s );

	// Messaggio step percorsi-----------------
	if (RadPercorsi>0){
		s = "Rotated stp CW: ";
	}
	else{
		s = "Rotated stp CCW: ";
	}
	s += robot.currCommand.stepsDone;
	s += " of " + robot.currCommand.targetSteps;
	cmdMsg( s );
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmdWiFi.sendCmdStart( kbRotationRad	);
	cmdWiFi.sendCmdArg( RadPercorsi );
	cmdWiFi.sendCmdEnd();
	//-----------------------------------------
}
void OnCmdRobotRotateDeg()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }
	int deg = cmdWiFi.readInt16Arg();
	String s = "OK rotateRadiants: ";
	cmdMsg(s);

	int DegPercorsi = 0;
	DegPercorsi = robot.rotateDeg(deg);

	//dtostrf( RadPercorsi, 7, 3, s );

	// Messaggio step percorsi-----------------
	if (DegPercorsi>0) {
		s = "Rotated CW stp: ";
	}
	else {
		s = "Rotated CCW stp: ";
	}
	s += robot.currCommand.stepsDone;
	s += " of " + robot.currCommand.targetSteps;
	cmdMsg(s);
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmdWiFi.sendCmdStart(kbRotationRad);
	cmdWiFi.sendCmdArg((float)PI*DegPercorsi/180);
	cmdWiFi.sendCmdEnd();
	//-----------------------------------------
}

void OnCmdRobotMoveCCW()
{
	int ck= cmdWiFi.readInt16Arg();
//	int dist = cmdWiFi.readInt16Arg();

	String s = "RECEIVED COMMAND [CCW]" + ck;
	cmdWiFi.sendCmd(Msg,s);
	robot.goCCW(ck);
}
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  P E R I F E R I C H E
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotRele()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	int16_t rele = cmdWiFi.readInt16Arg();		//numero del rele da attivare/disattivare
	int16_t onoff = cmdWiFi.readInt16Arg();		//numero del rele da attivare/disattivare
	robot.setRele(rele, onoff);
	String s = "Msg Rele " + String(rele) + ":" + String(onoff);
	cmdWiFi.sendCmd(Msg,s);
	// rimanda il medesimo comando indietro come ack
	cmdWiFi.sendCmdStart(CmdRobotRele );
	cmdWiFi.sendCmdArg( rele );
	cmdWiFi.sendCmdArg( onoff );
	cmdWiFi.sendCmdEnd();

}
void OnCmdSetLed()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	int16_t onoff = cmdWiFi.readInt16Arg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_ONBOARD_LED, onoff );
	String s = "Ack CmdSetLaser :" + String( onoff );
	cmdWiFi.sendCmd( Msg, s );
	// rimanda il medesimo comando indietro come ack
	cmdWiFi.sendCmdStart( CmdSetLed );
	cmdWiFi.sendCmdArg( onoff );
	cmdWiFi.sendCmdEnd();

}
void OnCmdSetLaser()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	int16_t onoff = cmdWiFi.readInt16Arg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_LaserOn, onoff );
	String s = "Ack cmdSetLaser :" + String( onoff );
	cmdWiFi.sendCmd( Msg, s );
	// rimanda il medesimo comando indietro come ack
	cmdWiFi.sendCmdStart( CmdSetLaser);
	cmdWiFi.sendCmdArg( digitalReadFast(Pin_LaserOn ));
	cmdWiFi.sendCmdEnd();
}
void OnCmdSetPort()
{
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	int16_t port = cmdWiFi.readInt16Arg();		//numero della porta
	int16_t onoff = cmdWiFi.readInt16Arg();		//valore 0 1

	digitalWriteFast( port, onoff );
	String s = "Ack CmdSetPort " + String( port ) + ":" + String( onoff );
	cmdWiFi.sendCmd( Msg, s );
	// rimanda il medesimo comando indietro come ack
	cmdWiFi.sendCmdStart( CmdSetPort );
	cmdWiFi.sendCmdArg( port );
	cmdWiFi.sendCmdArg( onoff );
	cmdWiFi.sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   A C Q U I S I Z I O N E
//////////////////////////////////////////////////////////////////////////
void OnCmdGetSensorsHRate()  //attenzione al limite dei 9600baud
{
	robot.readSensors();	//IR proxy, Gyro

	cmdWiFi.sendCmdStart(kbGetSensorsHRate); 
	cmdWiFi.sendCmdArg(robot.status.irproxy.fw);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.irproxy.fwHL);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.irproxy.bk);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.pirDome);		// movimento

	cmdWiFi.sendCmdArg(robot.status.analog[0]);	//pot
	cmdWiFi.sendCmdArg(robot.status.analog[1]);	//batteria
	cmdWiFi.sendCmdArg(robot.status.analog[2]);	//light

	cmdWiFi.sendCmdArg(robot.getReleStatus(0));		//rele 1
	cmdWiFi.sendCmdArg(robot.getReleStatus(1));		//rel2

	cmdWiFi.sendCmdArg( digitalReadFast( Pin_MotENR ) );
	cmdWiFi.sendCmdArg( digitalReadFast( Pin_MotENL ) );

	cmdWiFi.sendCmdArg( digitalReadFast( Pin_BtOnOff ) );
	cmdWiFi.sendCmdArg( digitalReadFast( Pin_LaserOn ) );
 	cmdWiFi.sendCmdArg( robot.readBattChargeLevel() );	

	cmdWiFi.sendCmdEnd();

}
void OnCmdGetSensorsLRate()
{
	// Lettura Switch Modalità Autonomo/MODE_SLAVE
	if (robot.status.switchTop != digitalReadFast(Pin_SwitchTop))
	{//cambio modo
		robot.status.switchTop = digitalReadFast(Pin_SwitchTop);

		if (robot.status.switchTop) { 
			robot.SetMode(AUTONOMOUS); 
			SPEAK_AUTONOMO
				dbg("Autonomo")
		}
		else {
			robot.SetMode(MODE_SLAVE);
			SPEAK_MODE_SLAVE	
				dbg("MODE_SLAVE")

		}

	}

	cmdWiFi.sendCmdStart(kbGetSensorsLRate); 
	// Percentuale di carica batteria
	
	cmdWiFi.sendCmdArg( (int)robot.operatingMode );
	cmdWiFi.sendCmdArg( robot.readBattChargeLevel() );	
	cmdWiFi.sendCmdArg( robot.status.switchTop );	


	if (gps.location.isUpdated()) {
		//printf("1,SATS= %l ;", gps.satellites.value());
		//printf("1,LAT= %l ;", gps.location.lat());
		//printf("1,LONG== %l ;", gps.location.lng());
		//printf("1,ALT== %f ;", gps.altitude.meters());
		SERIAL_WIFI.print("1,");
		SERIAL_WIFI.println(gps.satellites.value()); // Number of satellites in use (u32)
		SERIAL_WIFI.print("LAT=");  SERIAL_WIFI.print(gps.location.lat(), 6);
		SERIAL_WIFI.print(", LONG="); SERIAL_WIFI.print(gps.location.lng(), 6);
		SERIAL_WIFI.print(", ALT=");  SERIAL_WIFI.print(gps.altitude.meters());
		SERIAL_WIFI.print(";");
	}

	// IR proxy
	//gps 

	cmdWiFi.sendCmdEnd();
}
void OnCmdReadPort()
{
	int16_t port = cmdWiFi.readInt16Arg();		//numero della porta
	if (robot.operatingMode == MODE_SLAVE) { SPEAK_OK }

	digitalReadFast( port );
	String s = "Ack ReadPort " + String( port ) ;
	cmdWiFi.sendCmd( Msg, s );
	// rimanda il medesimo comando indietro come ack
	cmdWiFi.sendCmdStart( kbReadPort );
	cmdWiFi.sendCmdArg( port );
 
	cmdWiFi.sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// si muove alla  direzione e  velocità impostata
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotStartMoving()
{
	robot.currCommand.commandDir = (commandDir_e)cmdWiFi.readInt16Arg();		//direzione (enum commandDir_t {STOP, GOFW, GOBK, GOCW, GOCCW};)
	int motorCK= cmdWiFi.readInt16Arg();
	//String s = "Mov ck" +  robot.currCommand.commandDir;
	Serial.print( "1,Mov ck" ); Serial.print( motorCK ); Serial.print( ";" );

	
	switch (robot.currCommand.commandDir)
	{
		case GOF:
			robot.goFW(motorCK);
			cmdMsg("goFW"); 
			break;
		case GOB:
			robot.goBK(motorCK);
			cmdMsg( "goBK" );
			break;
		case GOR:
			robot.goCW(motorCK);
			cmdMsg( "goCW" );
			break;	
		case GOL:
			robot.goCCW(motorCK);
			cmdMsg( "goCCW" );
			break;		
		default:
			cmdMsg( "Error on direction" );
			break;
	}
}
void OnCmdRobotStopMoving(){
	cmdMsg( "Stopped" );
	robot.stop();

	}


//////////////////////////////////////////////////////////////////////////
/// C O M A N D I      S O N A R
//////////////////////////////////////////////////////////////////////////

#if OPT_SERVOSONAR

	//////////////////////////////////////////////////////////////
	///		Invia i dati del sonar
	/////////////////////////////////////////////////////////////
	void kbSonarSendData(){
		int alfa;
		int i;
		

		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robot.parameter.sonarScanSteps );
		 
		for (i = 0; i< robot.parameter.sonarScanSteps; i++)
		{
			//kb.printLfCr();
			delay( 20 );
			cmdWiFi.sendCmdStart(kbGetSonarData);
			alfa = robot.parameter.sonarStartAngle + i* robot.parameter.sonarStepAngle;
			cmdWiFi.sendCmdArg( alfa );
			cmdWiFi.sendCmdArg( robot.status.sonarEchos[i] );
			cmdWiFi.sendCmdEnd();
		}
		cmdWiFi.sendCmd( kbSonarDataEnd );

	}


	///////////////////////////////////////////////////////////////////////////////
	// Setup sonar																///
	///////////////////////////////////////////////////////////////////////////////
	void  OnCmdSonarScan()
	{
		SPEAK_OK


		bool blLimited = false;
		String s="";
		robot.parameter.sonarScanSweeps = cmdWiFi.readInt16Arg();
		robot.parameter.sonarScanSpeed = cmdWiFi.readInt16Arg();	//ROBOT_SONAR_SCAN_SPEED_DEFAULT=200;
		robot.parameter.sonarMaxDistance=cmdWiFi.readInt16Arg();	// 500; //in cm
		robot.parameter.sonarStartAngle = cmdWiFi.readInt16Arg();
		robot.parameter.sonarEndAngle = cmdWiFi.readInt16Arg();	// 180; // ampiezza angolo di scansione in gradi
		robot.parameter.sonarStepAngle = cmdWiFi.readInt16Arg();	// 8 ; //  ampiezza step in gradi della scansione (ok anche 10)

		if (robot.parameter.sonarScanSweeps > 1)
		{
			robot.parameter.sonarScanSweeps = 1;
			s += "\nOnCmdSonarSetup ! Limited sonarScanSweeps to 1";

			blLimited = true;
		}
		if ( robot.parameter.sonarEndAngle > 180)
		{
			robot.parameter.sonarEndAngle = 180 ;
			s += "\nOnCmdSonarSetup ! Limited sonarEndAngle to 180";

			blLimited = true;
		}
		
		// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
		if (robot.parameter.sonarScanSpeed < 40)
		{ 
			robot.parameter.sonarScanSpeed = 40;
			s += "\nOnCmdSonarSetup ! Limited sonarScanSpeed to 40";

			blLimited = true;
		}
		//robot.parameter.sonarScanSteps =min( (int)(robot.parameter.sonarScanAngle / robot.parameter.sonarStepAngle) , 255);
		
		if (!blLimited)
		{
			s = "\nOnCmdSonarSetup OK";
		}
		else
		{
			/// Rimando i valori dei parametri limitati

		}
		cmdWiFi.sendCmd(Msg,s);


		cmdMsg( "Scanning...");
		robot.SonarScanBatch( &servoSonar, &Sonar );
		delay( 200 );
		// invia i dati 
		kbSonarSendData();

		//CmdMessenger2 kb = CmdMessenger2( Serial );


	}




	//-------------------------------------------
	// invia i dati SONAR man mano che si sposta
	//-------------------------------------------
	void OnCmdSonarScanSync()
	{  

		String s = "\nStart scanning...";
		cmdWiFi.sendCmd(Msg,s);

		cmdWiFi.sendCmdStart(kbGetSonarData);

		int i=0; 
		int pos = robot.parameter.sonarStartAngle; //int endPos=SONAR_ARRAY_SIZE;

		while(pos < robot.parameter.sonarEndAngle ) // goes from 0 degrees to 180 degrees
		{
			//robot.status.sonarEchos[i] =robot.SonarPingAtPos(pos);



			// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
			unsigned long t1 = millis();
			cmdWiFi.sendCmdArg( robot.status.sonarEchos[i]);
			
			while (millis() - t1 > (unsigned long)robot.parameter.sonarScanSpeed) {
				delay( 1 );
			}

			pos += robot.parameter.sonarStepAngle;
			i++;

		}
		cmdWiFi.sendCmdEnd();
	
		s = "...end scanning";
		cmdWiFi.sendCmd(Msg,s);
	}
#endif


// Called when a received command has no attached function

// Callback function calculates the sum of the two received float values
//void OnCmdRobotMoveFW()
//{
  //
  //// Retreive second parameter as float
  //float b = cmdMessenger.readFloatArg();
  //
  //// Send back the result of the addition
  ////cmdMessenger.sendCmd(kFloatAdditionResult,a + b);
  //cmdMessenger.sendCmdStart(kFloatAdditionResult);
  //cmdMessenger.sendCmdArg(a+b);
  //cmdMessenger.sendCmdArg(a-b);
  //cmdMessenger.sendCmdEnd();
//}





void attachCommandCallbacks()		//va messa in fondo
{
#pragma region  Attach callback methods to WiFi channel 
	cmdWiFi.attach(OnUnknownCommand);
	cmdWiFi.attach(CmdRobotHello, OnCmdRobotHello);
	cmdWiFi.attach(CmdReboot, OnCmdReboot);

	cmdWiFi.attach(CmdRobotStartMoving, OnCmdRobotStartMoving);
	cmdWiFi.attach(CmdRobotStopMoving, OnCmdRobotStopMoving);

	cmdWiFi.attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
	cmdWiFi.attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);

	cmdWiFi.attach(CmdRobotRele, OnCmdRobotRele);
	cmdWiFi.attach(CmdSetLed, OnCmdSetLed);
	cmdWiFi.attach(CmdSetLaser, OnCmdSetLaser);
	cmdWiFi.attach(CmdSetPort, OnCmdSetPort);

	cmdWiFi.attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
	cmdWiFi.attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);
	cmdWiFi.attach(CmdRobotSetMode, OnCmdRobotSetMode);
	#if OPT_SERVOSONAR
	cmdWiFi.attach(CmdSonarScan,OnCmdSonarScan);
	//cmdWiFi.attach(CmdSonarScanBatch, OnCmdSonarScanBatch );
	//cmdWiFi.attach(CmdSonarScanSync, OnCmdSonarScanSync);
	#endif

#pragma endregion
#pragma region  Attach callback methods to Voice Commands channel 
	cmdMMI.attach(OnUnknownCommand);
	cmdMMI.attach(CmdRobotHello, OnCmdRobotHello);
	cmdMMI.attach(CmdReboot, OnCmdReboot);

	cmdMMI.attach(CmdRobotStartMoving, OnCmdRobotStartMoving);
	cmdMMI.attach(CmdRobotStopMoving, OnCmdRobotStopMoving);

	cmdMMI.attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
	cmdMMI.attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);

	cmdMMI.attach(CmdRobotRele, OnCmdRobotRele);
	cmdMMI.attach(CmdSetLed, OnCmdSetLed);
	cmdMMI.attach(CmdSetLaser, OnCmdSetLaser);
	cmdMMI.attach(CmdSetPort, OnCmdSetPort);

	cmdMMI.attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
	cmdMMI.attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);
	cmdMMI.attach(CmdRobotSetMode, OnCmdRobotSetMode);
	#if OPT_SERVOSONAR
	cmdMMI.attach(CmdSonarScan,OnCmdSonarScan);
		//cmdWiFi.attach(CmdSonarScanBatch, OnCmdSonarScanBatch );
		//cmdWiFi.attach(CmdSonarScanSync, OnCmdSonarScanSync);
	#endif

#pragma endregion
}

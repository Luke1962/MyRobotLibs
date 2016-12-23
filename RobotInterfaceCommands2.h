/* Robot Interface Commands */

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
//#include <CmdMessenger2.h>  // CmdMessenger2
#include "robot.h"
#include "Commands_Enum.h"
#include <CmdMessenger2/CmdMessenger2.h>
//extern struct robot_c robot;
//Dichiarazione di funzione che punta all'indirizzo zero
void( *Riavvia )(void) = 0;
#define MESSAGEMAXLEN 50
char strBuff[MESSAGEMAXLEN];

/// ///////////////////////////////////////////////////////////////////////////////////
// ////////////////////////  C A L L B A C K S				//////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////////
 
/// ///////////////////////////////////////////////////////////////////////
//  C O M A N D I   D I   I M P O S T A Z I O N E  G E N E R A L E
/// ///////////////////////////////////////////////////////////////////////
void cmdMsg(const char msg[]){
//	cmd->sendCmd( Msg, msg );
 	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
 	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");

}
//void cmdMsg(const __FlashStringHelper* msg[]){
////	cmd->sendCmd( Msg, msg );
// 	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
// 	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.print(";");
//	//ser.print("1,"); ser.print(msg); ser.print(";");
//
//}

void cmdMsg( const char msg[], int v) {
 	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(v); SERIAL_MSG.print(";");
 	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.print(v);SERIAL_MMI.print(";");
 }

//void cmdMsg(CmdMessenger2 *cmd,  char *msg, int v) {
// 	cmd->sendCmd(Msg);
//	for (int i = 0; i < sizeof(msg); i++)
//	{
//		cmd->sendCmdArg(msg[i]);
//	}
// 	cmd->sendCmdArg(v);
//	cmd->sendCmdEnd();
//}
//void cmdMsg(CmdMessenger2 *cmd,  char msg[], int v) {
// 	cmd->sendCmd(Msg);
//	for (int i = 0; i < sizeof(msg); i++)
//	{
//		cmd->sendCmdArg(msg[i]);
//	}
// 	cmd->sendCmdArg(v);
//	cmd->sendCmdEnd();
//}
//void cmdMsg(CmdMessenger2 *cmd,  String msg, int v) {
// 	cmd->sendCmd(Msg);
//	cmd->sendCmdArg(msg);
// 	cmd->sendCmdArg(v);
//	cmd->sendCmdEnd();
//}
//void cmdMsg(CmdMessenger2 *cmd, char msg[], int v) {
//	//	cmd->sendCmd( Msg, msg );
//	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
//	//ser.print("1,"); ser.print(msg); ser.print(";");
//	cmd->sendCmd(Msg);
//	cmd->sendCmdArg(msg);
//	cmd->sendCmdArg(v);
//	cmd->sendCmdEnd();
//}
 
void OnCmdReboot(CmdMessenger2 *cmd){
	//reset software
	cmdMsg( "Riavvio..." );
	Riavvia();
//	software_Reboot();

}
/// ///////////////////////////////////////////////////////////////////////
//  Modalità operativa : SLAVE , JOYSTICK , AUTONOMOUS
/// ///////////////////////////////////////////////////////////////////////
void OnCmdRobotSetMode(CmdMessenger2 *cmd){
	if (robot.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

	robot.SetMode((operatingMode_e)cmd->readInt16Arg());
 
	switch (robot.status.operatingMode)
	{
		case MODE_SLAVE:
			cmd->sendCmd( CmdRobotSetMode, MODE_SLAVE);
			cmdMsg("SetMode SLAVE");
			break;
		case MODE_JOYSTICK:
			cmd->sendCmd( CmdRobotSetMode, MODE_JOYSTICK );
			 
			cmdMsg("SetMode JOYSTICK");
			break;
		case MODE_AUTONOMOUS:
			cmd->sendCmd( CmdRobotSetMode, MODE_AUTONOMOUS );
			 SPEAK_AUTONOMO
			cmdMsg( "SetMode AUTONOMOUS" );
			break;	
 	
		default:
			cmd->sendCmd(Msg,"Unrecognised Mode");			 
			break;
	}
}
void OnUnknownCommand(CmdMessenger2 *cmd)
{

	cmd->sendCmd(kError,"\nCommand not recognised");

	cmdMsg("unkwownCmd :");
	cmd->reset();
}
void OnCmdRobotHello(CmdMessenger2 *cmd)
{
	cmdMsg("Hello I'm ready");
}
/// ///////////////////////////////////////////////////////////////////////
//  C O M A N D I   D I   M O V I M E N T O
//////////////////////////////////////////////////////////////////////////
// Avanti o indietro di x cm --------------------------------------------
void OnCmdRobotMoveCm(CmdMessenger2 *cmd)
{

 
	//String s;
	int cmPercorsi=0;
	int dist = cmd->readInt16Arg();
	
	cmdMsg( "MoveCm: ", dist);

	//cmd->sendCmd(Msg,dist);

	//eseguo lo spostamento
	cmPercorsi=robot.moveCm(dist);

	// riporto la distanza percorsa-----------
	if(dist>0){
		cmdMsg( "Moved steps Forward: ", robot.status.cmd.stepsDone);
	}
	else{
		cmdMsg( "Moved steps Back: ", robot.status.cmd.stepsDone);
	}
 
	cmdMsg( "..of targetSteps:", robot.status.cmd.targetSteps);
	//-----------------------------------------

 
	// riporto la distanza percorsa-----------
	cmd->sendCmdStart( kbMovedCm );
	cmd->sendCmdArg( cmPercorsi );
	cmd->sendCmdEnd();
	//-----------------------------------------
}
 // ROTAZIONE IN RADIANTI --------------------------------------
void OnCmdRobotRotateRadiants(CmdMessenger2 *cmd)
{
	float rad = cmd->readFloatArg();
	cmdMsg(  "OK rotateRadiants: "  );

	float RadPercorsi = 0.0;
	RadPercorsi = robot.rotateRadiants( rad );

	//dtostrf( RadPercorsi, 7, 3, s );

	// Messaggio step percorsi-----------------
	if (RadPercorsi>0){
		cmdMsg("Rotated stp CW: ", RadPercorsi);
	}
	else{
		cmdMsg("Rotated stp CCW: ", -RadPercorsi);
	}
	cmdMsg("of : ", robot.status.cmd.targetSteps);
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmd->sendCmdStart( kbRotationRad	);
	cmd->sendCmdArg( RadPercorsi );
	cmd->sendCmdEnd();
	//-----------------------------------------
}
void OnCmdRobotRotateDeg(CmdMessenger2 *cmd)
{
	if (robot.status.operatingMode == MODE_SLAVE) { SPEAK_OK }
	int deg = cmd->readInt16Arg();
	//String s = "OK rotateRadiants: ";
	cmdMsg( "OK rotateRadiants: ");

	int DegPercorsi = 0;
 	DegPercorsi = robot.rotateDeg(deg);
 
	//dtostrf( RadPercorsi, 7, 3, s );

	// Messaggio step percorsi-----------------
	if (DegPercorsi > 0) {
		// Messaggio step percorsi-----------------
		cmdMsg( "Rotated stp CW: ", DegPercorsi);
	}
	else {
		cmdMsg( "Rotated stp CCW: ", -DegPercorsi);
	}
	cmdMsg( "of : ", deg);
	//-----------------------------------------
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmd->sendCmdStart(kbRotationDeg);
	cmd->sendCmdArg(DegPercorsi);
	cmd->sendCmdEnd();
	//-----------------------------------------
}

void OnCmdRobotMoveCCW(CmdMessenger2 *cmd)
{
	int ck= cmd->readInt16Arg();
 
 	cmdMsg( "RECEIVED COMMAND [MoveCCW]", ck);
 	robot.goCCW(ck);
}
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  P E R I F E R I C H E
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotRele(CmdMessenger2 *cmd)
{


	int16_t rele = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	robot.setRele(rele, onoff);

	char m[] = "set Rele :";
	cmdMsg(m , rele);

	// rimanda il medesimo comando indietro come ack


}
void OnCmdSetLed(CmdMessenger2 *cmd)
{

	int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_ONBOARD_LED, onoff );


	cmdMsg( "Ack CmdSetLed :", onoff);

	//// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetLed );
	//cmd->sendCmdArg( onoff );
	//cmd->sendCmdEnd();

}
void OnCmdSetLaser(CmdMessenger2 *cmd)
{

	int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_LaserOn, onoff );
	cmdMsg( "Ack CmdSetLaser :", onoff);

	//// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetLaser);
	//cmd->sendCmdArg( digitalReadFast(Pin_LaserOn ));
	//cmd->sendCmdEnd();
}
void OnCmdSetPort(CmdMessenger2 *cmd)
{


	int16_t port = cmd->readInt16Arg();		//numero della porta
	int16_t onoff = cmd->readInt16Arg();		//valore 0 1

	digitalWriteFast( port, onoff );
	//String s = "Ack CmdSetPort " + String( port ) + ":" + String( onoff );
	cmdMsg( "Ack CmdSetPort :", port);

 //	// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetPort );
	//cmd->sendCmdArg( port );
	//cmd->sendCmdArg( onoff );
	//cmd->sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   A C Q U I S I Z I O N E
//////////////////////////////////////////////////////////////////////////
void OnCmdGetSensorsHRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud su canale WiFi
{
	robot.readSensors();	//IR proxy, Gyro

	cmd->sendCmdStart(kbGetSensorsHRate); 
	cmd->sendCmdArg(millis());

	cmd->sendCmdArg(robot.status.tictac);

	cmd->sendCmdArg(robot.status.posCurrent.x);	//robot position X
	cmd->sendCmdArg(robot.status.posCurrent.y);	//robot position y
	cmd->sendCmdArg(robot.status.posCurrent.r);	//robot position alfa gradi

	cmd->sendCmdArg(robot.status.sensors.irproxy.fw);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fwHL);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.bk);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.pirDome);		// movimento

	cmd->sendCmdArg(robot.status.sensors.analog[0]);	//pot
	cmd->sendCmdArg(robot.status.sensors.analog[1]);	//batteria
	cmd->sendCmdArg(robot.status.sensors.analog[2]);	//light

	cmd->sendCmdArg(robot.getReleStatus(0));		//rele 1
	cmd->sendCmdArg(robot.getReleStatus(1));		//rel2

	//cmd->sendCmdArg( digitalReadFast( Pin_MotENR ) );
	//cmd->sendCmdArg( digitalReadFast( Pin_MotENL ) );

	cmd->sendCmdArg(robot.status.sensors.switchTop); // status switch modo Autonomo/slave
	cmd->sendCmdArg(robot.status.act.laserOn); // laser

	cmd->sendCmdArg(robot.readBattChargeLevel()); //0-100

	cmd->sendCmdArg(robot.status.sensors.gps.sats);		//gps
	cmd->sendCmdArg(robot.status.sensors.gps.lat);
	cmd->sendCmdArg(robot.status.sensors.gps.lng);
	cmd->sendCmdEnd();

}
void OnCmdGetSensorsLRate(CmdMessenger2 *cmd)
{
  
	cmd->sendCmdStart(kbGetSensorsLRate); 
	// Percentuale di carica batteria
	
	cmd->sendCmdArg( (int)robot.status.operatingMode );
	cmd->sendCmdArg( robot.readBattChargeLevel() );	
	cmd->sendCmdArg( robot.status.sensors.switchTop );	
	cmd->sendCmdArg(robot.status.sensors.gps.sats);
	cmd->sendCmdArg(robot.status.sensors.gps.lat);
	cmd->sendCmdArg(robot.status.sensors.gps.lng);
	cmd->sendCmdArg(robot.status.sensors.gps.alt);

	cmd->sendCmdEnd();
}
void OnCmdReadPort(CmdMessenger2 *cmd)
{
	int16_t port = cmd->readInt16Arg();		//numero della porta
	if (robot.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

	digitalReadFast( port );
	String s = "Ack ReadPort " + String( port ) ;
	cmd->sendCmd( Msg, s );
	// rimanda il medesimo comando indietro come ack
	cmd->sendCmdStart( kbReadPort );
	cmd->sendCmdArg( port );
 
	cmd->sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// si muove alla  direzione e  velocità impostata
//////////////////////////////////////////////////////////////////////////
/*
void OnCmdRobotStartMoving(CmdMessenger2 *cmd)
{
	robot.status.cmd.commandDir = (commandDir_e)cmd->readInt16Arg();		//direzione (enum commandDir_t {STOP, GOFW, GOBK, GOCW, GOCCW};)
	int motorCK= cmd->readInt16Arg();
	//String s = "Mov ck" +  robot.currCommand.commandDir;
	Serial.print( "1,Mov ck" ); Serial.print( motorCK ); Serial.print( ";" );

	
	switch (robot.status.cmd.commandDir)
	{
		case GOF:
			robot.goFW(motorCK);
			cmdMsg("goFW");
			break;
		case GOB:
			robot.goBK(motorCK);
			cmdMsg("goBK" );
			break;
		case GOR:
			robot.goCW(motorCK);
			cmdMsg( "goCW" );
			break;	
		case GOL:
			robot.goCCW(motorCK);
			cmdMsg("goCCW" );
			break;		
		default:
			cmdMsg( "Error on direction" );
			break;
	}
}
void OnCmdRobotStopMoving(CmdMessenger2 *cmd){
	robot.stop();
	cmdMsg( "Stopped" );

	}
*/

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I      S O N A R
//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////
	///		Invia i dati del sonar
	/////////////////////////////////////////////////////////////
	void kbSonarSendData(CmdMessenger2 *cmd){
		int alfa;
		int i;
		

		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robot.status.parameters.sonarScanSteps );
		 
		for (i = 0; i< robot.status.parameters.sonarScanSteps; i++)
		{
			//kb.printLfCr();
			//delay( 20 );
			cmd->sendCmdStart(kbGetSonarData);
			alfa = robot.status.parameters.sonarStartAngle + i* robot.status.parameters.sonarStepAngle;
			cmd->sendCmdArg( alfa );
			cmd->sendCmdArg( robot.status.sensors.sonarEchos[i] );
			cmd->sendCmdEnd();
		}
		cmd->sendCmd( kbSonarDataEnd );

	}

	void  OnCmdSonarScanDefault(CmdMessenger2 *cmd) 
	{
 
 
		cmdMsg( "Scanning...");
		robot.LDSScanBatch();
		//delay( 200 );
		// invia i dati 
		cmdMsg( "Sending scan data...");
		//kbSonarSendData(cmd);
		int alfa;
		int i;


		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robot.status.parameters.sonarScanSteps );
		dbg2("sonarScanSteps: ", robot.status.parameters.sonarScanSteps)
		for (i = 0; i< robot.status.parameters.sonarScanSteps; i++)
		{
			//kb.printLfCr();
			//delay( 20 );
			cmd->sendCmdStart(kbGetSonarData);
			alfa = robot.status.parameters.sonarStartAngle + i* robot.status.parameters.sonarStepAngle;
			cmd->sendCmdArg(alfa);
			cmd->sendCmdArg(robot.status.sensors.sonarEchos[i]);
			cmd->sendCmdEnd();
		}
		cmd->sendCmd(kbSonarDataEnd);

		//CmdMessenger2 kb = CmdMessenger2( Serial );


	}

	///////////////////////////////////////////////////////////////////////////////
	// Setup sonar																///
	///////////////////////////////////////////////////////////////////////////////
	void  OnCmdSonarScan(CmdMessenger2 *cmd) 
	{
		// es. di comando 2 passate
		// 25,2,200,500,45,135,10;

		bool blLimited = false;
		String s="";
		robot.status.parameters.sonarScanSweeps = cmd->readInt16Arg();
		robot.status.parameters.sonarScanSpeed = cmd->readInt16Arg();	//ROBOT_SONAR_SCAN_SPEED_DEFAULT=200;
		robot.status.parameters.sonarMaxDistance=cmd->readInt16Arg();	// 500; //in cm
		robot.status.parameters.sonarStartAngle = cmd->readInt16Arg();
		robot.status.parameters.sonarEndAngle = cmd->readInt16Arg();	// 180; // ampiezza angolo di scansione in gradi
		robot.status.parameters.sonarStepAngle = cmd->readInt16Arg();	// 8 ; //  ampiezza step in gradi della scansione (ok anche 10)

		if (robot.status.parameters.sonarScanSweeps > 1)
		{
			robot.status.parameters.sonarScanSweeps = 1;
			s += "\nOnCmdSonarSetup ! Limited sonarScanSweeps to 1";

			blLimited = true;
		}
		if ( robot.status.parameters.sonarEndAngle > 180)
		{
			robot.status.parameters.sonarEndAngle = 180 ;
			s += "\nOnCmdSonarSetup ! Limited sonarEndAngle to 180";

			blLimited = true;
		}
		
		// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
		if (robot.status.parameters.sonarScanSpeed < 40)
		{ 
			robot.status.parameters.sonarScanSpeed = 40;
			s += "\nOnCmdSonarSetup ! Limited sonarScanSpeed to 40";

			blLimited = true;
		}
		//robot.status.parameters.sonarScanSteps =min( (int)(robot.status.parameters.sonarScanAngle / robot.status.parameters.sonarStepAngle) , 255);
		
		if (!blLimited)
		{
			cmdMsg("\nOnCmdSonarSetup OK");
		}
		else
		{
			cmdMsg("\nVALORI LIMITATI !!");
			/// Rimando i valori dei parametri limitati

		}


		cmdMsg( "Scanning...");
		robot.LDSScanBatch();
		delay( 200 );
		// invia i dati 
		kbSonarSendData(cmd);

		//CmdMessenger2 kb = CmdMessenger2( Serial );


	}




	/// //////////////////////////////////////////////////
	// invia i dati SONAR man mano che si sposta
	/// //////////////////////////////////////////////////
	void OnCmdSonarScanSync(CmdMessenger2 *cmd)
	{  

		cmdMsg("\nStart scanning...");

		cmd->sendCmdStart(kbGetSonarData);

		int i=0; 
		int pos = robot.status.parameters.sonarStartAngle; //int endPos=SONAR_ARRAY_SIZE;

		while(pos < robot.status.parameters.sonarEndAngle ) // goes from 0 degrees to 180 degrees
		{
			robot.status.sensors.sonarEchos[i] =robot.SonarPingAtPos(pos);
			cmd->sendCmdArg( robot.status.sensors.sonarEchos[i]);
			chThdSleepMilliseconds( robot.status.parameters.sonarScanSpeed);
			

			pos += robot.status.parameters.sonarStepAngle;
			i++;

		}
		cmd->sendCmdEnd();
	
 		cmdMsg("...end scanning");
	}



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





void attachCommandCallbacks(CmdMessenger2 *cmd)		//va messa in fondo
{
#pragma region  Attach callback methods to WiFi channel 

	cmd->attach(OnUnknownCommand);
	cmd->attach(CmdRobotHello, OnCmdRobotHello);
	cmd->attach(CmdReboot, OnCmdReboot);

	//cmd->attach(CmdRobotStartMoving, OnCmdRobotStartMoving);
	//cmd->attach(CmdRobotStopMoving, OnCmdRobotStopMoving);

	cmd->attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
	cmd->attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);
	cmd->attach(CmdRobotRotateDeg, OnCmdRobotRotateDeg);

	cmd->attach(CmdRobotRele, OnCmdRobotRele);
	cmd->attach(CmdSetLed, OnCmdSetLed);
	cmd->attach(CmdSetLaser, OnCmdSetLaser);
	cmd->attach(CmdSetPort, OnCmdSetPort);

	cmd->attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
	cmd->attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);
	cmd->attach(CmdRobotSetMode, OnCmdRobotSetMode);
	cmd->attach(CmdSonarScan,OnCmdSonarScan);

#pragma endregion
}

/* Robot Interface Commands */
#ifndef __RobotInterfaceCommandsCore__
#define __RobotInterfaceCommandsCore__

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
//#include <CmdMessenger2.h>  // CmdMessenger2
#include "robot.h"
#include <MyRobotLibs\dbg.h>
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

 
void onCmdReboot(CmdMessenger2 *cmd){
	//reset software
	MSG( "Riavvio..." );
	Riavvia();
//	software_Reboot();

}
/// ///////////////////////////////////////////////////////////////////////
//  Modalità operativa : SLAVE , JOYSTICK , AUTONOMOUS
/// ///////////////////////////////////////////////////////////////////////
void onCmdRobotSetMode(CmdMessenger2 *cmd){
 
	robot.status.operatingMode =(operatingMode_e)cmd->readInt16Arg();
	switch (robot.status.operatingMode)
	{
		case MODE_SLAVE:
			cmd->sendCmd( CmdRobotSetMode, MODE_SLAVE);
			MSG("OK SLAVE");
			break;
		case MODE_JOYSTICK:
			cmd->sendCmd( CmdRobotSetMode, MODE_JOYSTICK );
			 
			MSG("OK JOYSTICK");
			break;
		case MODE_AUTONOMOUS:
			cmd->sendCmd( CmdRobotSetMode, MODE_AUTONOMOUS );
			SPEAK_AUTONOMO
			MSG( "OK AUTONOMOUS" );
			break;	
	
		default:
			MSG("Unrecognised Mode");
			break;
	}
}


void OnUnknownCommand(CmdMessenger2 *cmd)
{

	MSG("Command not recognised");

	//MSG("unkwownCmd :");
	cmd->reset();
}
/// ///////////////////////////////////////////////////////////////////////
//  C O M A N D I   D I   M O V I M E N T O
/// ///////////////////////////////////////////////////////////////////////
// Avanti o indietro di x cm --------------------------------------------
void onCmdRobotMoveCm(CmdMessenger2 *cmd)
{

 
	//String s;
	int cmPercorsi=0;
	int dist = cmd->readInt16Arg();
	

	//cmd->sendCmd(Msg,dist);

	//eseguo lo spostamento
	cmPercorsi=robot.moveCm(dist);

	// riporto la distanza percorsa-----------
	if(dist>0){
		MSG2( "Moved steps Forward: ", robot.status.cmd.stepsDone);
	}
	else{
		MSG2( "Moved steps Back: ", robot.status.cmd.stepsDone);
	}
 
	MSG2( "..of targetSteps:", robot.status.cmd.targetSteps);
	//-----------------------------------------

 
	// riporto la distanza percorsa-----------
	cmd->sendCmdStart( kbMovedCm );
	cmd->sendCmdArg( cmPercorsi );
	cmd->sendCmdEnd();
	MSG2( "Cm done: ", cmPercorsi);
	//-----------------------------------------
}
 // ROTAZIONE IN RADIANTI --------------------------------------
void onCmdRobotRotateRadiants(CmdMessenger2 *cmd)
{
	float rad = cmd->readFloatArg();

	float RadPercorsi = 0.0;
	RadPercorsi = robot.rotateRadiants( rad );

	//dtostrf( RadPercorsi, 7, 3, s );

	// Messaggio step percorsi-----------------
	if (RadPercorsi>0){
		MSG2("Rotated stp CW: ", RadPercorsi);
	}
	else{
		MSG2("Rotated stp CCW: ", -RadPercorsi);
	}
	MSG2("of : ", robot.status.cmd.targetSteps);
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmd->sendCmdStart( kbRotationRad	);
	cmd->sendCmdArg( RadPercorsi );
	cmd->sendCmdEnd();
	MSG2(  "rot done Rad: " , RadPercorsi);
	//-----------------------------------------
}
void onCmdRobotRotateDeg(CmdMessenger2 *cmd)
{
	if (robot.status.operatingMode == MODE_SLAVE) { SPEAK_OK }
	int deg = cmd->readInt16Arg();
	//String s = "OK rotateRadiants: ";

	int DegPercorsi = 0;
	DegPercorsi = robot.rotateDeg(deg);
 


	// Messaggio step percorsi-----------------
	if (DegPercorsi > 0) {
		// Messaggio step percorsi-----------------
		MSG2( "Rotated Deg CW: ", DegPercorsi);
	}
	else {
		MSG2("Rotated Deg CCW: ", -DegPercorsi);
	}
	MSG2( "of : ", deg);
	//-----------------------------------------
	//-----------------------------------------



	// riporto la distanza percorsa-----------
	cmd->sendCmdStart(kbRotationDeg);
	cmd->sendCmdArg(DegPercorsi);
	cmd->sendCmdEnd();
	MSG2( "Rotated Deg: ", DegPercorsi);
	//-----------------------------------------
}

void onCmdRobotMoveCCW(CmdMessenger2 *cmd)
{
	int ck= cmd->readInt16Arg();
 
	MSG2( "OK MoveCCW: ", ck);
	robot.goCCW(ck);
}
// Avvia il robot secondo direzione e velocità passata dagli argomenti
void onCmdRobotGo(CmdMessenger2 *cmd)
{
	commandDir_e dir= (commandDir_e)cmd->readInt16Arg();
	robotSpeed_e speed= (robotSpeed_e)cmd->readInt16Arg();	 

	MSG("OK Move: ");
	robot.go(dir,speed);
}
// FERMA IMMEDIATAMENTE
void onCmdRobotStop(CmdMessenger2 *cmd)
{
	robot.stop();
}

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  P E R I F E R I C H E
//////////////////////////////////////////////////////////////////////////
void onCmdRobotRele(CmdMessenger2 *cmd)
{

	int16_t rele = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	robot.setRele(rele, onoff);

	MSG2(  "set Rele :", rele);

	// rimanda il medesimo comando indietro come ack

}
void onCmdSetLed(CmdMessenger2 *cmd)
{

	int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_ONBOARD_LED, onoff );


	MSG2( "Ack CmdSetLed :", onoff);

	//// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetLed );
	//cmd->sendCmdArg( onoff );
	//cmd->sendCmdEnd();

}
void onCmdSetLaser(CmdMessenger2 *cmd)
{

	bool onoff = cmd->readBoolArg();		//numero del rele da attivare/disattivare
	digitalWriteFast( Pin_LaserOn, onoff );
	MSG2( "Ack CmdSetLaser :", onoff);

	//// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetLaser);
	//cmd->sendCmdArg( digitalReadFast(Pin_LaserOn ));
	//cmd->sendCmdEnd();
}
void onCmdSetPort(CmdMessenger2 *cmd)
{


	int16_t port = cmd->readInt16Arg();		//numero della porta
	int16_t onoff = cmd->readInt16Arg();		//valore 0 1

	digitalWriteFast( port, onoff );
	//String s = "Ack CmdSetPort " + String( port ) + ":" + String( onoff );
	MSG2( "Ack CmdSetPort :", port);

 //	// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetPort );
	//cmd->sendCmdArg( port );
	//cmd->sendCmdArg( onoff );
	//cmd->sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   A C Q U I S I Z I O N E
//////////////////////////////////////////////////////////////////////////
void onCmdGetSensorsHRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud su canale WiFi
{
	//osalSysDisable();

	//robot.readSensorsHR();	//IR proxy, Gyro


	cmd->sendCmdStart(kbGetSensorsHRate); 
	// param 1
	cmd->sendCmdArg(millis());

	// param 2
	cmd->sendCmdArg(robot.status.tictac);

	// param 3,4,5
	cmd->sendCmdArg(robot.status.posCurrent.x);	//robot position X
	cmd->sendCmdArg(robot.status.posCurrent.y);	//robot position y
	cmd->sendCmdArg(robot.status.posCurrent.r);	//robot position alfa gradi

	// param 6,7,8,9,10
	cmd->sendCmdArg(robot.status.sensors.irproxy.fw);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fwHL);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fr);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fl);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.bk);	// IR proxy

	// param 11
	cmd->sendCmdArg(robot.status.sensors.pirDome);		// movimento

	// param 12,13,14
	cmd->sendCmdArg(robot.status.sensors.analog[0]);	//pot
	cmd->sendCmdArg(robot.status.sensors.analog[1]);	//batteria
	cmd->sendCmdArg(robot.status.sensors.analog[2]);	//light


	// param 15,16
	cmd->sendCmdArg(robot.getReleStatus(0));		//rele 1
	cmd->sendCmdArg(robot.getReleStatus(1));		//rel2

	//cmd->sendCmdArg( digitalReadFast( Pin_MotENR ) );
	//cmd->sendCmdArg( digitalReadFast( Pin_MotENL ) );

	// param 17,18
	cmd->sendCmdArg(robot.status.sensors.switchTop); // status switch modo Autonomo/slave
	cmd->sendCmdArg(robot.status.act.laserOn); // laser

	//cmd->sendCmdArg(robot.readBattChargeLevel()); //0-100

	//cmd->sendCmdArg(robot.status.sensors.gps.sats);		//gps
	//cmd->sendCmdArg(robot.status.sensors.gps.lat);
	//cmd->sendCmdArg(robot.status.sensors.gps.lng);
	cmd->sendCmdEnd();

	//osalSysEnable();

}
void onCmdGetSensorsLRate(CmdMessenger2 *cmd)
{
	robot.readSensorsLR();	//IR proxy, Gyro

	cmd->sendCmdStart(kbGetSensorsLRate); 
	// Percentuale di carica batteria
	
	cmd->sendCmdArg( robot.readBattChargeLevel() );
	cmd->sendCmdArg( robot.status.sensors.switchTop );	

	cmd->sendCmdArg(robot.status.sensors.gps.lat);
	cmd->sendCmdArg(robot.status.sensors.gps.lng);
	cmd->sendCmdArg(robot.status.sensors.gps.sats);

	cmd->sendCmdArg( (int)robot.status.operatingMode );

	//cmd->sendCmdArg(robot.status.sensors.gps.alt);

	cmd->sendCmdEnd();
}
void onCmdGetProxy(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud su canale WiFi
{
	//osalSysDisable();

	//robot.readSensorsHR();	//IR proxy, Gyro


	cmd->sendCmdStart(kbProxy); 
	cmd->sendCmdArg(robot.status.sensors.irproxy.fw);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fwHL);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fr);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.fl);	// IR proxy
	cmd->sendCmdArg(robot.status.sensors.irproxy.bk);	// IR proxy

	cmd->sendCmdArg(robot.status.sensors.pirDome);		// movimento
	cmd->sendCmdEnd();

	//osalSysEnable();

}

void onCmdGetPose(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud su canale WiFi
{

	cmd->sendCmdStart(kbGetPose); 
	cmd->sendCmdArg(millis());


	cmd->sendCmdArg(robot.status.posCurrent.x);	//robot position X
	cmd->sendCmdArg(robot.status.posCurrent.y);	//robot position y
	cmd->sendCmdArg(robot.status.posCurrent.r);	//robot position alfa gradi

	cmd->sendCmdEnd();

}


void onCmdReadPort(CmdMessenger2 *cmd)
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
 
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I      S O N A R
//////////////////////////////////////////////////////////////////////////
#pragma region SONAR


	// ////////////////////////////////////////////////////////////
	//		Invia un set di coppie singole di dati (alfa, Distance)
	// alfa è l'angolo del servo (0° dritto, >0 a destra)
	// ///////////////////////////////////////////////////////////
	void kbSonarSendData(CmdMessenger2 *cmd) {
		int alfa;
		int i;


		// invio quanti dati ho da trasmettere
		// kb.sendCmdArg( robot.status.parameters.sonarScanSteps );

		for (i = 0; i < robot.status.parameters.sonarScanSteps; i++)
		{

			cmd->sendCmdStart(kbGetSonarData);
			alfa = robot.status.parameters.sonarStartAngle + i* robot.status.parameters.sonarStepAngle;
			cmd->sendCmdArg(alfa);
			cmd->sendCmdArg(robot.status.sensors.sonarEchos[i]);
			cmd->sendCmdEnd();
		}
		cmd->sendCmd(kbSonarDataEnd);

	}

	void  onCmdSonarScanDefault(CmdMessenger2 *cmd)
	{


		MSG("Scanning...");
		robot.LDSScanBatch();
		//delay( 200 );
		// invia i dati 
		MSG("Sending scan data...");
		//kbSonarSendData(cmd);
		int alfa;
		int i;


		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robot.status.parameters.sonarScanSteps );
		dbg2("sonarScanSteps: ", robot.status.parameters.sonarScanSteps)
			for (i = 0; i < robot.status.parameters.sonarScanSteps; i++)
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
	void  onCmdSonarScan(CmdMessenger2 *cmd)
	{
		// es. di comando 2 passate
		// 25,2,200,500,45,135,10;

		bool blLimited = false;
		String s = "";
		robot.status.parameters.sonarScanSweeps = cmd->readInt16Arg();
		robot.status.parameters.sonarScanSpeed = cmd->readInt16Arg();	//ROBOT_SONAR_SCAN_SPEED_DEFAULT=200;
		robot.status.parameters.sonarMaxDistance = cmd->readInt16Arg();	// 500; //in cm
		robot.status.parameters.sonarStartAngle = cmd->readInt16Arg();
		robot.status.parameters.sonarEndAngle = cmd->readInt16Arg();	// 180; // ampiezza angolo di scansione in gradi
		robot.status.parameters.sonarStepAngle = cmd->readInt16Arg();	// 8 ; //  ampiezza step in gradi della scansione (ok anche 10)

		if (robot.status.parameters.sonarScanSweeps > 1)
		{
			robot.status.parameters.sonarScanSweeps = 1;
			s += "\nonCmdSonarSetup ! Limited sonarScanSweeps to 1";

			blLimited = true;
		}
		if (robot.status.parameters.sonarEndAngle > 180)
		{
			robot.status.parameters.sonarEndAngle = 180;
			s += "\nonCmdSonarSetup ! Limited sonarEndAngle to 180";

			blLimited = true;
		}

		// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
		if (robot.status.parameters.sonarScanSpeed < 40)
		{
			robot.status.parameters.sonarScanSpeed = 40;
			s += "\nonCmdSonarSetup ! Limited sonarScanSpeed to 40";

			blLimited = true;
		}
		//robot.status.parameters.sonarScanSteps =min( (int)(robot.status.parameters.sonarScanAngle / robot.status.parameters.sonarStepAngle) , 255);

		if (!blLimited)
		{
			MSG("onCmdSonarSetup OK");
		}
		else
		{
			MSG("VALORI LIMITATI !!");
			/// Rimando i valori dei parametri limitati

		}


		MSG("Scanning...");
		robot.LDSScanBatch();
		delay(200);
		// invia i dati 
		kbSonarSendData(cmd);

		//CmdMessenger2 kb = CmdMessenger2( Serial );


	}

	/// //////////////////////////////////////////////////
	// invia i dati SONAR man mano che si sposta
	/// //////////////////////////////////////////////////
	void onCmdSonarScanSync(CmdMessenger2 *cmd)
	{  

		MSG("Start scanning...");
		

		cmd->sendCmdStart(kbGetSonarData);

		int i=0; 
		int pos = robot.status.parameters.sonarStartAngle; //int endPos=SONAR_ARRAY_SIZE;

		while(pos < robot.status.parameters.sonarEndAngle ) // goes from 0 degrees to 180 degrees
		{
			robot.status.sensors.sonarEchos[i] =robot.sonarPingAtPos(pos);
			cmd->sendCmdArg( robot.status.sensors.sonarEchos[i]);
			chThdSleepMilliseconds( robot.status.parameters.sonarScanSpeed);
			

			pos += robot.status.parameters.sonarStepAngle;
			i++;

		}
		cmd->sendCmdEnd();
	
		MSG("...end scanning");
	}

	void onCmdServoPos(CmdMessenger2 *cmd)
	{
		int alfa;
		alfa =cmd->readInt16Arg();
		if ((alfa >= 0) && (alfa <= 180)) {
			robot._pServoSonar->write(alfa);
		}
		MSG3("Servo at: ",alfa,"°");

	}

#pragma endregion




 


void attachCommandCallbacks(CmdMessenger2 *cmd)		//va messa in fondo
{
#pragma region  Attach callback methods 

	cmd->attach(OnUnknownCommand);

	cmd->attach(CmdRobotSetMode, onCmdRobotSetMode);
	cmd->attach(CmdRobotSonarScan,onCmdSonarScan);
	cmd->attach(CmdRobotGo, onCmdRobotGo);
	cmd->attach(CmdRobotStop, onCmdRobotStop);

	cmd->attach(CmdRobotMoveCm, onCmdRobotMoveCm);
	cmd->attach(CmdRobotRotateDeg, onCmdRobotRotateDeg);

	cmd->attach(CmdSetRele, onCmdRobotRele);
	cmd->attach(CmdSetLed, onCmdSetLed);
	cmd->attach(CmdSetLaser, onCmdSetLaser);
	cmd->attach(CmdSetPort, onCmdSetPort);

	cmd->attach(CmdGetSensorsHRate, onCmdGetSensorsHRate);
	cmd->attach(CmdGetSensorsLRate, onCmdGetSensorsLRate);
	cmd->attach(CmdGetPose,onCmdGetPose);
	cmd->attach(CmdReboot, onCmdReboot);
	cmd->attach(CmdServoPos, onCmdServoPos);
	cmd->attach(CmdGetProxy, onCmdGetProxy);

#pragma endregion
}
#endif // !__RobotInterfaceCommandsCore__

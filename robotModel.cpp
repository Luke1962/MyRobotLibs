/////////////////////////////////////////////////////////////////////////////////////////////
// CONTIENE LE STESSE FUNZIONI DI ROBOT.CPP MA I COMANDI LI INVIA TRAMITE SERIALE AL CORE  //
/////////////////////////////////////////////////////////////////////////////////////////////

#include "robotModel.h"
	
///----------------------------------------------------------
/// CONSTRUCTOR
///---------------------------------------------------------
robotBaseModel_c::robotBaseModel_c(){

	begin(operatingMode_e::MODE_SLAVE);
}

///----------------------------------------------------------
/// INIZIALIZZAZIONE
///---------------------------------------------------------
void robotBaseModel_c::begin(operatingMode_e mode){
	status.operatingMode = mode;
	setPose(0, 0, 90);
	status.cmd.commandDir = STOP;


	//-----------------------------------------
	/// Setup Sensori
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

	//-----------------------------------------
	/// Inizializzazione eventi
	//-----------------------------------------
	resetEvents();



	//-----------------------------------------
	/// Inizializzazione posizione
	//-----------------------------------------
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
	status.isMoving = false;
		


	//-----------------------------------------
	// Setup parametri Sonar
	//-----------------------------------------
	status.parameters.sonarScanSpeed = 100;  // ms di attesa tra due posizioni
	status.parameters.sonarMaxDistance=500; //in cm
	status.parameters.sonarScanSweeps =  1;

	status.parameters.sonarStartAngle =  0;
	status.parameters.sonarEndAngle  = 180; // ampiezza angolo di scansione in gradi
	status.parameters.sonarStepAngle  =  5; //  ampiezza step in gradi della scansione (ok anche 10)
//		status.parameters.sonarScanSteps = (int)(status.parameters.sonarScanAngle / status.parameters.sonarStepAngle);
	status.parameters.sonarMedianSamples = 10;
	// inizializzo l'array delle echo
	for (int i = 0; i < 180; i++) 
	{
		status.sensors.sonarEchos[i] = 0;
	}

	statusOld = status;
};


// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool robotBaseModel_c::hasExpired(unsigned long &prevTime, unsigned long interval)
{
	if (  millis() - prevTime > interval ) 
	{
		prevTime = millis();
		return true;
	} else
	return false;
}

/////////////////////////////////////////////////////////////
// solleva i flag degli eventi (in modo cumulativo)
/////////////////////////////////////////////////////////////
bool robotBaseModel_c::raiseEvents()
{
	bool event = false;
	if ((status.posCurrent.x != statusOld.posCurrent.x)
		||(status.posCurrent.y != statusOld.posCurrent.y)
		||(status.posCurrent.r != statusOld.posCurrent.r)
		)
	{
		status.pendingEvents.posCurrent = true;
		event = true;
	}
	// bumpers
	if (status.sensors.bumper.right != statusOld.sensors.bumper.right)
	{
		status.pendingEvents.bumperR = true;
		event = true;
	}
	if (status.sensors.bumper.center != statusOld.sensors.bumper.center)
	{
		status.pendingEvents.bumperF = true;
		event = true;
	}
	if (status.sensors.bumper.left != statusOld.sensors.bumper.left)
	{
		status.pendingEvents.bumperL = true;
		event = true;
	}
	// ir proxy
	if (status.sensors.irproxy.fw != statusOld.sensors.irproxy.fw)
	{
		status.pendingEvents.irproxyF = true;
		event = true;
	}
	if (status.sensors.irproxy.bk != statusOld.sensors.irproxy.bk)
	{
		status.pendingEvents.irproxyB = true;
		event = true;
	}
	if (status.sensors.irproxy.fr != statusOld.sensors.irproxy.fr)
	{
		status.pendingEvents.irproxyR = true;
		event = true;
	}
	if (status.sensors.irproxy.fl != statusOld.sensors.irproxy.fl)
	{
		status.pendingEvents.irproxyL = true;
		event = true;
	}
	// pir
	if (status.sensors.pirDome != statusOld.sensors.pirDome)
	{
		status.pendingEvents.pirDome = true;
		event = true;
	}
	//analog
	if (status.sensors.analog[2] != statusOld.sensors.analog[2])
		{
			status.pendingEvents.light = true;
			event = true;
	}

	
	//rele
	if (status.act.rele[0] != statusOld.act.rele[0])
	{
		status.pendingEvents.rele0 = true;
		event = true;
	}
	if (status.act.rele[1] != statusOld.act.rele[1])
	{
		status.pendingEvents.rele1 = true;
		event = true;
	}
	//livello di carica batteria
	if (status.sensors.batCharge != statusOld.sensors.batCharge)
	{
		status.pendingEvents.batCharge = true;
		event = true;
	}
	status.pendingEvents.EventFlag = event;
	return status.pendingEvents.EventFlag;
}
void robotBaseModel_c::resetEvents()
{
		status.pendingEvents.posCurrent = false;
		status.pendingEvents.bumperR = false;
		status.pendingEvents.bumperF = false;
		status.pendingEvents.bumperL = false;
		status.pendingEvents.irproxyF = false;
		status.pendingEvents.irproxyFH = false;
		status.pendingEvents.irproxyB = false;
		status.pendingEvents.irproxyR = false;
		status.pendingEvents.irproxyL = false;
		status.pendingEvents.pirDome = false;
		status.pendingEvents.light = false;
		status.pendingEvents.rele0 = false;
		status.pendingEvents.rele1 = false;
		status.pendingEvents.batCharge = false;
		status.pendingEvents.EncL = false;
		status.pendingEvents.EncR = false;
		status.pendingEvents.gps = false;
		for (size_t i = 0; i < 4; i++)
		{
			status.pendingEvents.analog[i] = false;
		}
		status.pendingEvents.light = false;
		status.pendingEvents.operatingMode = false;
		status.pendingEvents.switchTop = false;
		status.pendingEvents.ts = 0;

 }

String robotBaseModel_c::getOperatingModeStr() {
	switch (status.operatingMode)
	{
	case MODE_UNKNOWN:		return "UNKNOWN"; break;
	case MODE_SLAVE:		return "SLAVE"; break;
	case MODE_JOYSTICK:	return "JOYSTICK"; break;
	case MODE_AUTONOMOUS:return "AUTONOMOUS"; break;
	default: return "SLAVE"; break;;
	}
}
char* robotBaseModel_c::getOperatingModeChar( ) {

	char* modeStr= new char[10]; 
	switch (status.operatingMode)
	{

		case MODE_UNKNOWN:	strcpy(modeStr, "MODE_UNKNOWN"); break;
		case MODE_SLAVE:		 strcpy(modeStr, "SLAVE"); break;
		case MODE_JOYSTICK:	 strcpy(modeStr, "JOYSTICK");  break;
		case MODE_AUTONOMOUS: strcpy(modeStr, "AUTONOMOUS");  break;
		default:   strcpy(modeStr, "SLAVE"); break;
	}
	return modeStr;



}
void robotBaseModel_c::pushPathHistory(commandDir_e dir, int val) {
	// push the numbers to the stack.
	motionHist.cmdDir = dir;
	motionHist.val = val;
	stackMotion.push(motionHist);

}
// imposta la posizione e angolo corrente
void robotBaseModel_c::setPose(long x, long y, int r) {
	status.posCurrent.x = x;
	status.posCurrent.y = y;
	status.posCurrent.r = r;
}
// imposta la posizione e angolo corrente in base a distanza e delta angolo 
// chiamata da MoveCm con updatePose(cmPercorsi,0)
// e da rotateDeg con updatePose(0,rotation)
void robotBaseModel_c::updatePose(long dist, int alfa) {
	status.posCurrent.x += dist*cos(alfa);
	status.posCurrent.y += dist*sin(alfa);
	status.posCurrent.r += alfa;
}




#pragma region robotVirtualModel_c

void robotVirtualModel_c::begin(operatingMode_e mode, CmdMessenger2* cmd) {
	this->cmd2Robot = cmd;
	robotBaseModel_c::begin(mode);

}

//////////////////////////////////////////////////////////////////////////
/// Gestione modalità operativa
//////////////////////////////////////////////////////////////////////////	
void robotVirtualModel_c::cmdSetMode(operatingMode_e mode){
	status.pendingEvents.operatingMode = true;
	status.operatingMode = mode;
	cmd2Robot->sendCmdStart(CmdRobotSetMode);
	cmd2Robot->sendCmdArg(mode);
	cmd2Robot->sendCmdEnd();
	//if (robotModel.status.operatingMode != robotModel.statusOld.operatingMode) {
	//}
}

//////////////////////////////////////////////////////////////////////////
// Invia il comando di una scansione di 180 ° e memorizza l'angolo in gradi con la maggiore distanza
//////////////////////////////////////////////////////////////////////////
//void robot_c::LDSScanBatch( Servo *_pServoSonar, VL53L0X *_pDistanceSensor){
void robotVirtualModel_c::cmdLDSScanBatch() {

	cmd2Robot->sendCmdStart(CmdRobotSonarScan);
	cmd2Robot->sendCmdEnd();

}

//////////////////////////////////////////////////////////////////////////
/// Gestione movimento
//////////////////////////////////////////////////////////////////////////	
int robotVirtualModel_c::cmdMoveCm(int cm) {
	int cmDone = 0;
	cmd2Robot->sendCmdStart(CmdRobotMoveCm);
	cmd2Robot->sendCmdArg(cm);
	cmd2Robot->sendCmdEnd();

	//-Attesa callback ----------------------------------------
	while (!SERIAL_ROBOT.available())
	{
		chThdSleepMilliseconds(500);//	chThdYield();
	}
	cmDone = cmd2Robot->readInt32Arg();
	return cmDone;

}
int robotVirtualModel_c::cmdRotateDeg(int deg) {
	cmd2Robot->sendCmdStart(CmdRobotRotateDeg);
	cmd2Robot->sendCmdArg(deg);
	cmd2Robot->sendCmdEnd();
	return 0;
}
int robotVirtualModel_c::cmdRotateRad(double rad) {
	cmd2Robot->sendCmdStart(CmdRobotRotateRadiants);
	cmd2Robot->sendCmdArg(rad);
	cmd2Robot->sendCmdEnd();
	return 0;

}

int robotVirtualModel_c::cmdSetRele(uint8_t rele, uint8_t onoff) {
	///robotModel.setRele(rele, onoff);

	// rimanda il medesimo comando indietro come ack
	cmd2Robot->sendCmdStart(CmdSetRele);
	cmd2Robot->sendCmdArg(rele);
	cmd2Robot->sendCmdArg(onoff);
	cmd2Robot->sendCmdEnd();
	return 0;
}

void  robotVirtualModel_c::cmdReadAllSensors() {
	cmd2Robot->sendCmdStart(CmdGetSensorsHRate);
	cmd2Robot->sendCmdEnd();
	cmd2Robot->sendCmdStart(CmdGetSensorsLRate);
	cmd2Robot->sendCmdEnd();

}
void  robotVirtualModel_c::cmdGetSensorsHR() {
	cmd2Robot->sendCmdStart(CmdGetSensorsHRate);
	cmd2Robot->sendCmdEnd();
}
void  robotVirtualModel_c::cmdGetSensorsLR() {
	cmd2Robot->sendCmdStart(CmdGetSensorsLRate);
	cmd2Robot->sendCmdEnd();

}
void  robotVirtualModel_c::cmdSetLaser(bool blOn) {
	cmd2Robot->sendCmdStart(CmdSetLaser);
	cmd2Robot->sendCmdArg(blOn);
	cmd2Robot->sendCmdEnd();

}
void  robotVirtualModel_c::cmdGo(commandDir_e dir, motCk_t speed) {
	cmd2Robot->sendCmdStart(CmdRobotGo);
	cmd2Robot->sendCmdArg(dir);
	cmd2Robot->sendCmdArg(speed);
	cmd2Robot->sendCmdEnd();

}
void  robotVirtualModel_c::cmdStop() {
	cmd2Robot->sendCmdStart(CmdRobotStop);
	cmd2Robot->sendCmdEnd();

}
void  robotVirtualModel_c::cmdRiavvia() {
	cmd2Robot->sendCmdStart(CmdReboot);
	cmd2Robot->sendCmdEnd();

}

// invia il comando di posizionare il servo del sonar all'angolo alfa
void  robotVirtualModel_c::cmdServoPos(int alfa) {
	cmd2Robot->sendCmdStart(CmdServoPos);
	cmd2Robot->sendCmdArg(alfa);
	cmd2Robot->sendCmdEnd();

}



#pragma endregion

/////////////////////////////////////////////////////////////////////////////////////////////
// CONTIENE LE STESSE FUNZIONI DI ROBOT.CPP MA I COMANDI LI INVIA TRAMITE SERIALE AL CORE  //
/////////////////////////////////////////////////////////////////////////////////////////////

#include "robotModel.h"
	
///----------------------------------------------------------
/// CONSTRUCTOR
///---------------------------------------------------------
robotModel_c::robotModel_c (){

	//this->begin(operatingMode_e::MODE_SLAVE);
}

///----------------------------------------------------------
/// INIZIALIZZAZIONE
///---------------------------------------------------------
void robotModel_c::begin(operatingMode_e mode){
	status.operatingMode = mode;
	setPose(0, 0, 90);
	status.cmd.commandDir = STOP;


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
void robotModel_c::begin(operatingMode_e mode, CmdMessenger2* cmd) {
	this->cmd2Robot = cmd;
	this->begin(mode);

}


// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool robotModel_c::hasExpired(unsigned long &prevTime, unsigned long interval)
{
	if (  millis() - prevTime > interval ) 
	{
		prevTime = millis();
		return true;
	} else
	return false;
}

//////////////////////////////////////////////////////////////////////////
/// Gestione modalità operativa
//////////////////////////////////////////////////////////////////////////	
void robotModel_c::SetMode(operatingMode_e mode){
	 status.operatingMode = mode;
	}
String robotModel_c::getOperatingModeStr() {
	switch (status.operatingMode)
	{
	case MODE_UNKNOWN:		return "UNKNOWN"; break;
	case MODE_SLAVE:		return "SLAVE"; break;
	case MODE_JOYSTICK:	return "JOYSTICK"; break;
	case MODE_AUTONOMOUS:return "AUTONOMOUS"; break;
	default: return "SLAVE"; break;;
	}
}
char* robotModel_c::getOperatingModeChar( ) {

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


void robotModel_c::pushPathHistory(commandDir_e dir, int val) {
	// push the numbers to the stack.
	motionHist.cmdDir = dir;
	motionHist.val = val;
	stackMotion.push(motionHist);

}
// imposta la posizione e angolo corrente
void robotModel_c::setPose(long x, long y, int r) {
	status.posCurrent.x = x;
	status.posCurrent.y = y;
	status.posCurrent.r = r;
}
// imposta la posizione e angolo corrente in base a distanza e delta angolo 
// chiamata da MoveCm con updatePose(cmPercorsi,0)
// e da rotateDeg con updatePose(0,rotation)
void robotModel_c::updatePose(long dist, int alfa) {
	status.posCurrent.x += dist*cos(alfa);
	status.posCurrent.y += dist*sin(alfa);
	status.posCurrent.r += alfa;
}

//////////////////////////////////////////////////////////////////////////
// Invia il comando di una scansione di 180 ° e memorizza l'angolo in gradi con la maggiore distanza
//////////////////////////////////////////////////////////////////////////
//void robot_c::LDSScanBatch( Servo *_pServoSonar, VL53L0X *_pDistanceSensor){
void robotModel_c::LDSScanBatch() {


}

//////////////////////////////////////////////////////////////////////////
/// Gestione movimento
//////////////////////////////////////////////////////////////////////////	
int robotModel_c::moveCm(int cm){
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
int robotModel_c::rotateDeg(int deg) {
 	cmd2Robot->sendCmdStart(CmdRobotRotateDeg);
	cmd2Robot->sendCmdArg(deg);
	cmd2Robot->sendCmdEnd();

}

int robotModel_c::setRele(uint8_t rele, uint8_t onoff) {
	///robotModel.setRele(rele, onoff);
	
	// rimanda il medesimo comando indietro come ack
	cmd2Robot->sendCmdStart(CmdRobotRele );
	cmd2Robot->sendCmdArg( rele );
	cmd2Robot->sendCmdArg( onoff );
	cmd2Robot->sendCmdEnd();

}

void  robotModel_c::readSensors() {
	cmd2Robot->sendCmdStart(CmdGetSensorsHRate);

}


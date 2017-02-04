#if !defined(__COMMANDS_ENUM_H__)
#define __COMMANDS_ENUM_H__
///////////////////////////////////////////////////////////////////
// SE MODIFICHI L'ELENCO DEVI RICOMPILARE ENTRAMBI MMI E CORE !!!!!
///////////////////////////////////////////////////////////////////
enum commands_e		// Commands da mantenere allineati con robot.cs
{		//prefisso Cmd per i comandi da remoto, cb per i callback
	Nil,
	Msg,// Command to acknowledge that cmd was received
	CmdRobotHello,// comando da inviare per verificare se  il robot  è in ascolto
	CmdRobotSetMode,  // imposta la modalità MODE_MODE_SLAVE | Autonoma |ByJoystick
	kError,// Command to report errors
	CmdSetLed, // Command to request led to be set in specific state
	cmdSetLedFrequency,	//not used
	CmdReboot,
	Status,
	// MPU-------------
	DataMPU,
	StatusMPU,
	ErrorMPU,
	CmdMPUReset,
	CmdMPUSetRate,
	CmdMPUStart,
	CmdMPUStop,
	CmdMPUCalibrate,

	// MOVIMENTO ------------
	CmdRobotStartMoving,
	CmdRobotStop,
	CmdRobotMoveCm,
	CmdRobotRotateRadiants,
	CmdRobotRotateDeg,

	// SENSORI --------------
	CmdGetSensorsHRate,
	CmdGetSensorsLRate,
	CmdGetPose,
	CmdGetProxy,
	// ATTUATORI ------------
	CmdSetRele,

	// sonar-----------
	CmdRobotSonarScan,
	CmdSonarScanBatch,
	CmdSonarScanSync,

	cmdSpeech,
	// callback dal robot al PC o MMI------------------------------
	kbMovedCm,
	kbRotationRad,
	kbRotationDeg,
	kbGetSensorsHRate,
	kbGetSensorsLRate,
	kbGetSonarData,
	kbSonarDataEnd,
	CmdSetLaser,
	CmdSetPort,
	kbReadPort,
	kbGetPose,
	kbProxy,
	CmdRobotGo,
	CmdServoPos
} ;
#endif
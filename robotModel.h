#if !defined(__ROBOTMODEL_H__)
#define __ROBOTMODEL_H__
// dichiarare nel main robot_c robot;
// dichiarare nel main robot_c robot;
//#define dbg2(msg, v)   Serial.print(F(msg)); Serial.println(v);
//#define dbg(msg)  Serial.println(F(msg));
//#define dbg(fmt, ...) (#if DEBUG_ENABLED (Debug.log(fmt, ## __VA_ARGS__); #endif)

////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//// INCLUDE
////////////////////////////////////////////////////////////////////////////
#include <stdint.h>
#include <Arduino.h>
#include <hwMMI_config.h>

#pragma region  path recorder
// registra i movimenti fatti

// include stack library header.
#include <StackArray\StackArray.h>
#include <CmdMessenger2/CmdMessenger2.h>
#include <ChibiOS_AVR/ChibiOS_AVR.h>
#include <Commands_Enum.h>
#pragma endregion


//////////////////////////////////////////////////////////////////////////
// COSTANTI
//////////////////////////////////////////////////////////////////////////
									// parametri meccanici-------------------
#define ROBOT_WEEL_DISTANCE	350						// distanza in mm tra le due ruote
#define ROBOT_ROTATION_CIRCUMPHERENCE_CM =	109.956		// circoferenza = pi*ROBOT_WEEL_DISTANCE=109,956cm
#define ROBOT_MOTOR_STEPS_PER_CM	39.0f	//	42.44f				//	valore di 78 trovato sperimentalmente impostando 50cm di distanza)
#define ROBOT_MOTOR_STEPS_PER_RADIANT 2333				// con 1375 fa un po' piu di 180° // 371.36155//=ROBOT_MOTOR_STEPS_PER_ROTATION /2*pi
// NON USATO #define ROBOT_MOTOR_MM_PER_STEP  0.1282f			// mm percorsi con ogni step (inverso del precedente) (old value 0.4712389f	)	
//#define ROBOT_MOTOR_CM_PER_ROTATION 109.9557 //pI *
//#define ROBOT_MOTOR_STEPS_PER_ROTATION	2333.333	// =ROBOT_MOTOR_STEPS_PER_CM *ROBOT_MOTOR_CM_PER_ROTATION
#define ROBOT_MOTOR_STEPS_PER_DEG	24		// 24 calcolato empiricamente e va bene  (era	23.8237 )


#define SONAR_ARRAY_SIZE 180		// una cell per grado
#define ANALOG_PORTS 5
#define ROBOT_MOTORENABLE_ACTIVE 0		// 1 = enable a logica positiva 0 = enable a logica negata (DRV8825)
#define ROBOT_MOTORCW_ACTIVE 0		// 1 = CW a logica positiva 0 = CW a logica negata (DRV8825)

#define ROBOT_MOTOR_CLOCK_microsecondMIN 2500	//was 1500 1300 = 769 Step/s 1300 ck  velocità max
#define ROBOT_MOTOR_CLOCK_microsecondMAX 6000	//was 3000 4000 = 250 Step/s	 12400 ck velocità min 
#define MAX_TARGET_DISTANCE_CM 500	//max distanza da percorrere per un singolo comando
#define ROBOT_SONAR_SCAN_SPEED_DEFAULT 200
#define ROBOT_MOTOR_CLOCK_PER_CMs	23562.677f // distanza IMPULSI in uSec per velocità di 1cm/s

									// parametri dinamici-------------------
#define ACCEL_LOOP_MICROS_WAIT 5					// Valori ok 5, 10 
#define ROBOT_MOTOR_CLOCK_ACCEL 4					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok
#define ROBOT_MOTOR_CLOCK_DECEL 20					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok


#define ROBOT_MOTOR_STEPS_PER_ENCODERTICK 8.33f			//  = 200 steps/giro su  e 24 tick dell'encoder per giro


#define LDS_MAX_DISTANCE_CM 300
#define LDS_TIMEOUT_DISTANCE_CM 10000


enum commandDir_e {STOP, GOF, GOB, GOR, GOL};
enum commandStatus_e {pending, completed };


struct motionHistory_t {
	commandDir_e cmdDir;
	int val;
};


//posizione home (sala dietro la mia sedia)
#define GPSHOME_LAT  45.471633
#define GPSHOME_LNG  9.124051
// CHIAVARI  LAT 44.326953  LNG 9.289679


typedef unsigned long motCk_t; // was typedef uint32_t motCk; 
typedef int int16_t;

enum operatingMode_e
{
	MODE_UNKNOWN,
	MODE_SLAVE,
	MODE_JOYSTICK,
	MODE_AUTONOMOUS,
};

struct position_t{
	double x; // posizione verso est in mm ripetto a Home
	double y; // posizione a nord in mm ripetto a Home
	int r;	//angolo di rotazione rispetto al nord in gradi 
};
struct cmd_t {
	motCk_t	clock; // clock motori in microsecondi, limitato tra ROBOT_MOTOR_CLOCK_microsecondMIN/MAX
	bool	enL;		//enable
	bool	enR;
	bool	cwL;		// direzione 1 = CW
	bool	cwR;
	int acceleration;	// valore dell'acceleraz. in incrementi di CK in microsecondi
	int accelPhase;		// 1= in acc. , 0=velocità costante -1, in decel.
	int accelSteps;	// ampiezza della fase di accel/decel. in step
	volatile	uint16_t stepsDone;		//step percorsi, aggiornato dalla lettura encoders
	uint16_t targetSteps;	//step da percorrere 
	int targetSpeed;		// velocità finale in cm/s
	motCk_t targetCK;	//velocità finale in CK
	int targetCm;

	commandStatus_e commandStatus; //comando in corso di esecuzione o terminato
	commandDir_e commandDir;
	position_t poseTarget;	// posizione obiettivo nelle coordinate World
};
struct irproxy_t{
	unsigned fw: 1;	//Forward
	unsigned fwHL: 1;	//Forward High Light
	unsigned fr: 1; //Forward right
	unsigned fl: 1;	//Forward left
	unsigned bk: 1; //back

};
struct bumper_t{
	unsigned right: 1;	//Forward
	unsigned center: 1;	//Forward High Light
	unsigned left: 1;	//Forward left
};
struct gps_t {
	uint8_t sats;
	double  lat;
	double  lng;
	double alt;
	int homeDistCm;
	int homeAngleDeg;
};
struct sensors_t{
	unsigned long ts; // sensor data timestamp in milliseconds
	irproxy_t	irproxy;
	bumper_t bumper;
	uint16_t sonarEchos[SONAR_ARRAY_SIZE];	//distanza rilevata in funzione dell'angolo max 180
	int light;
	bool switchTop;
	bool pirDome;
	bool pir2;
	bool ignoreIR; // impostare a true per non usare i sensori di prossimità agli infrarossi 

	long int EncL;	// contatore  posizione encoder Left
	long int EncR;	// contatore  posizione encoder RIGHT};
	long int EncLprec;	// vecchio valore contatore  posizione encoder Left
	long int EncRprec;	// vecchio valore  contatore  posizione encoder RIGHT};
	long	analog[5]; //0..1023
	unsigned int batCharge;	// livello batteria 0-100%
	gps_t gps;
	//#if OPT_ENCODERS
	//
	//	Encoder_state_t  encL;		// statico perchè deve essere visibile all'ISR?
	//	Encoder_state_t  encR;

	//#endif
};
struct dim_t{	//dimensioni fisiche
	float BodyRadius;
	float WeelRadius;
};
struct parameter_t{
//		long stepsPerCm	;
		int maxspeed_ck;	//clock col valore piu basso
		int minspeed_ck;	//clock col valore piu alto
		int sonarScanSpeed;
		int sonarMaxDistance;	// distanza massima SONAR in cm = MAX_DISTANCE
		int sonarEndAngle;		// ampiezza scansione da 1 a 180
		int sonarStepAngle;		// risoluzione es 5
		int sonarStartAngle;
		int sonarScanSweeps;	// numero di passate
		int sonarScanSteps;		// step = sonarScanAngle/sonarStepAngle// contatore del numero di echo da inviare
		int sonarMedianSamples;		// 
		int SonarMaxDistAngle;		// angolo in cui nella scansione trova la distanza massima
};
struct actuators_t {
	bool laserOn;
	bool MotENR;
	bool MotENL;
	bool blueToothOn;
	unsigned int rele[10];

};

// Flag degli eventi da segnalare
struct pendingEvents_t {
	bool EventFlag; // true quando almenu un evento è stato sollevato
	bool operatingMode;//segnala un cambio nel modo operativo
	bool irproxyF;//segnala un cambio nei sensori di prossimità
	bool irproxyFH;//segnala un cambio nei sensori di prossimità
	bool irproxyB;//segnala un cambio nei sensori di prossimità
	bool irproxyR;//segnala un cambio nei sensori di prossimità
	bool irproxyL;//segnala un cambio nei sensori di prossimità
	bool bumperF;
	bool bumperR;
	bool bumperL;
	bool light;//segnala un cambio nel'intensità di luce
	bool switchTop;//segnala un cambio nello switch top
	bool pirDome;//segnala un cambio nel sensore di movimento
	bool pir2;
	bool EncL;	//segnala un cambio nell' encoder Left
	bool EncR;	//segnala un cambio nell' encoder RIGHT};
	bool analog[5]; //segnala un cambio negli ingressi analogici
	bool batCharge;	// segnala un cambio del livello batteria 0-100%
	bool gps;	// segnala un aggiornamento nel GPS
	bool posCurrent; //segnala un cambio nella posizione corrente
	bool rele0;	//segnala un cambio nel rele 0
	bool rele1;	//segnala un cambio nel rele 1
	unsigned long	ts;	//timestamp dell'evento piu recente
};
// Stato del Robot
struct robotStatus_t {
	bool	tictac;	// periodicamente switchato onoff
	operatingMode_e operatingMode;
	sensors_t		sensors;
	actuators_t		act;
	parameter_t		parameters;	// parametri operativi
	cmd_t			cmd;
	position_t		posCurrent; //posizione corrente in [mm] e [deg]
	pendingEvents_t pendingEvents;
	bool			isMoving;
	unsigned long	ts;	//timestamp
};

//#if OPT_ENCODERS
//	typedef struct {
//		volatile IO_REG_TYPE * pin1_register;
//		volatile IO_REG_TYPE * pin2_register;
//		IO_REG_TYPE            pin1_bitmask;
//		IO_REG_TYPE            pin2_bitmask;
//		uint8_t                state;
//		int32_t                position;
//	} Encoder_state_t;
//#endif
enum robotSpeed_e  //FREQUENZE DI CLOCK CORRISPONDENTI 
{
	MIN= ROBOT_MOTOR_CLOCK_microsecondMAX,
	SLOW= 4000,
	MEDIUM= 3500,
	FAST= 3000,
	MAX = ROBOT_MOTOR_CLOCK_microsecondMIN
};


// parte della classe robot usata sia dal progetto "robot" sia da "MMI"
// Contiene solo le proprietà 
// e metodi che non interagiscono con attuatori o sensori del robot
class robotBaseModel_c{
	//////////////////////////////////////////////////////////////////////////
	/// Proprietà Pubbliche
	//////////////////////////////////////////////////////////////////////////
	public:		
		robotBaseModel_c();
		enum		direction_t {fw, bk, left, right };
		dim_t			dim;	// dimensions
		position_t		posHome;		// posizione iniziale
		robotStatus_t	status;		// status readings
		robotStatus_t	statusOld;		// previous status readings
		motionHistory_t motionHist;
		int cmdSettingDefaultMoveCm;
		int cmdSettingDefaultRotateDeg;

		// create a stack of numbers.
		StackArray <motionHistory_t> stackMotion;

	//////////////////////////////////////////////////////////////////////////
	/// Metodi Pubblici
	//////////////////////////////////////////////////////////////////////////
	public:		
		void begin(operatingMode_e mode );
		//void runIBIT(int delayms);	// esegue Initial Built In Test
		void setPose(long x, long y, int r);
		void updatePose(long dist, int alfa);
		bool hasExpired(unsigned long &prevTime, unsigned long interval);
		bool raiseEvents(); //confronta statusOld con status e solleva i flag dei pendingEvent
		void resetEvents();


		String getOperatingModeStr();
		char* getOperatingModeChar(); //thread safe

		void pushPathHistory(commandDir_e dir, int val);

} ;
// classe di robot virtuale che include i metodi di comando al robot fisico via cmdMessanger (seriale)
class robotVirtualModel_c : public robotBaseModel_c {
public:
		void begin(operatingMode_e mode, CmdMessenger2* cmd);
		void cmdSetMode(operatingMode_e mode);
		int cmdMoveCm(int cm);
		int cmdRotateDeg(int deg);
		int cmdRotateRad(double rad);
		void cmdGo(commandDir_e dir, motCk_t speed);
		void cmdStop();

		void cmdReadAllSensors();
		void cmdLDSScanBatch(); // LDSScanBatch();
		int cmdSetRele(uint8_t rele, uint8_t onoff);
		void cmdSetLaser(bool blOn);
		void cmdGetSensorsHR();
		void cmdGetSensorsLR();
		void cmdRiavvia();
		void cmdServoPos(int alfa);

		CmdMessenger2* cmd2Robot;

};

#endif 
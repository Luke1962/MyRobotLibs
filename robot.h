#if !defined(__ROBOT_H__)
#define __ROBOT_H__
// dichiarare nel main robot_c robot;
// dichiarare nel main robot_c robot;
//#define dbg2(msg, v)   Serial.print(F(msg)); Serial.println(v);
//#define dbg(msg)  Serial.println(F(msg));
//#define dbg(fmt, ...) (#if DEBUG_ENABLED (Debug.log(fmt, ## __VA_ARGS__); #endif)
#include <ChibiOS_AVR\ChibiOS_AVR.h>
//////////////////////////////////////////////////////////////////////////
// OPZIONI DI DEBUG
//////////////////////////////////////////////////////////////////////////
//#define DEBUG_ON
#define DEBUG_ENCODERS 0
#define IRSENSORS_NOT_PRESENT 0	//per debugging senza i sensori OvbstacleFree() ritorna sempre true
#include <dbg.h>

//////////////////////////////////////////////////////////////////////////
//																		//
//						 O P Z I O N I   R O B O T						//
//																		//
//////////////////////////////////////////////////////////////////////////

#define OPT_SERVOSONAR 1	//INCLUDI FUNZIONALITA' SERVO-SONAR
#define OPT_ENCODERS  0	//INCLUDI ENCODERS
#define OPT_BT 0	//bluetooth
#define OPT_SPEECH 0
#define OPT_GPS 1
#define OPT_LDS 1	//Laser Distance sensor
#define OPT_COMPASS 1

//////////////////////////////////////////////////////////////////////////
// INCLUDE
//////////////////////////////////////////////////////////////////////////
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <arduino.h>
#include "robotModel.h"
#include "systemConfig.h"
#include "hw_config.h"
#include "speakCore.h"
//#include <FrequencyTimer2\FrequencyTimer2.h>
#include <digitalWriteFast\digitalWriteFast.h>
#include <PWM/PWM.h>

#if  OPT_SERVOSONAR
	#include <Average\Average.h>
	#define SONAR_MAX_SAMPLES 20

	#include <Newping/NewPing.h>
	#include <Servo/src/Servo.h>
#endif

#if	OPT_ENCODERS
//	#include <Encoder/Encoder.h>
	#include <../Encoder/utility/direct_pin_read.h>
#endif

#include <Wire/src/Wire.h>
#include <VL53L0X\VL53L0X.h>
#include <Adafruit_HMC5883_U\Adafruit_HMC5883_U.h>
#define COMPASS_CLASS Adafruit_HMC5883_Unified

#define LASER_ON digitalWriteFast(Pin_LaserOn,1);
#define LASER_OFF digitalWriteFast(Pin_LaserOn,0);
#define RELE1_ON robot.setRele(1, 1);
#define RELE1_OFF robot.setRele(1, 0);
#define WEBCAM_ON robot.setRele(2, 1);
#define WEBCAM_OFF robot.setRele(2, 0);
#define LEDTOP_R_ON digitalWriteFast(Pin_LED_TOP_R, 1);
#define LEDTOP_R_OFF digitalWriteFast(Pin_LED_TOP_R, 0);
#define LEDTOP_G_ON digitalWriteFast(Pin_LED_TOP_G, 1);
#define LEDTOP_G_OFF digitalWriteFast(Pin_LED_TOP_G, 0);
#define LEDTOP_B_ON digitalWriteFast(Pin_LED_TOP_B, 1);
#define LEDTOP_B_OFF digitalWriteFast(Pin_LED_TOP_B, 0);
#define TOGGLEPIN(P) digitalWriteFast(P,!digitalReadFast(P))
/**/
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
#define ROBOT_MOTOR_CLOCK_microsecondMAX 4000	//was 3000 4000 = 250 Step/s	 12400 ck velocità min 
#define MAX_TARGET_DISTANCE_CM 500	//max distanza da percorrere per un singolo comando
#define ROBOT_SONAR_SCAN_SPEED_DEFAULT 200
#define ROBOT_MOTOR_CLOCK_PER_CMs	23562.677f // distanza IMPULSI in uSec per velocità di 1cm/s

									// parametri dinamici-------------------
#define ACCEL_LOOP_MICROS_WAIT 5					// Valori ok 5, 10 
#define ROBOT_MOTOR_CLOCK_ACCEL 4					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok
#define ROBOT_MOTOR_CLOCK_DECEL 20					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok


#define ROBOT_MOTOR_STEPS_PER_ENCODERTICK 8.33f			//  = 200 steps/giro su  24 tick dell'encoder per giro
///enum commandDir_e {STOP, GOF, GOB, GOR, GOL};
///enum commandStatus_e {pending, completed };

#define LDS_MAX_DISTANCE_CM 300
#define LDS_TIMEOUT_DISTANCE_CM 10000
/////posizione home (sala dietro la mia sedia)
///#define GPSHOME_LAT  45.471633
///#define GPSHOME_LNG  9.124051


///typedef unsigned long motCk_t; // was typedef uint32_t motCk; 
///typedef int int16_t;

#if OPT_ENCODERS
	typedef struct {
		volatile IO_REG_TYPE * pin1_register;
		volatile IO_REG_TYPE * pin2_register;
		IO_REG_TYPE            pin1_bitmask;
		IO_REG_TYPE            pin2_bitmask;
		uint8_t                state;
		int32_t                position;
	} Encoder_state_t;
#endif

class  robot_c : public robotModel_c {
	//////////////////////////////////////////////////////////////////////////
	/// Metodi Pubblici
	//////////////////////////////////////////////////////////////////////////
	public:	
		robot_c();		//constructor
		void begin(operatingMode_e mode);
		#if OPT_COMPASS && OPT_SERVOSONAR && OPT_LDS && OPT_COMPASS
			void begin( Servo *_pServoSonar, NewPing *_pSonar , VL53L0X *_pDistanceSensor, COMPASS_CLASS *_compass);
		#endif
		#if OPT_COMPASS
			COMPASS_CLASS *_pCompass;
			void beginCompass( COMPASS_CLASS *_compass);
			int readCompassDeg();
			int readCompassDeg(COMPASS_CLASS *pCompass);
		#endif // OPT_COMPASS
		#if OPT_ENCODERS
			Encoder_state_t  encL;		// statico perchè deve essere visibile all'ISR?
			Encoder_state_t  encR;
			int encRead(Encoder_state_t  encoder);
			void encWrite(Encoder_state_t encoder, int32_t p);
			static void ISRencL( void );
			static 	void ISRencR( void );
		#endif
		#if  OPT_SERVOSONAR
			// includere le librerie nel main e instanziare l'oggetto			
			Servo *_pServoSonar;//pointer del servo sonar			
			NewPing	*_pSonar;//pointer all'oggetto sonar//	sonar(int trigPin,int echoPin);

			void beginServosonar( Servo *pServoSonar, NewPing *pSonar  );
			void SonarScanBatch( Servo *pServoSonar, NewPing *pSonar ); // SonarScanBatch();
			int SonarPingAtPos(int pos);
			int SonarPingAtPosAvg( int pos ,  float *maxStdDev,int samples= SONAR_MAX_SAMPLES);

			int SonarPing();
			//int SonarFindMaxDistAngle( Servo *_pServoSonar, NewPing *_pSonar );

		#endif
#if OPT_LDS
			VL53L0X *_pLDS;

			void robot_c::beginLDS(VL53L0X *pDistanceSensor);
			void LDSScanBatch(); // LDSScanBatch();
			int getLDSDistance(int pos);
			int LDSPing();
			int getLDSDistance();


#endif // SERVO_LDS

		//void SetMode(operatingMode_e mode);	
		int moveCm(int cm);
		long rotateSteps( long steps );
		float rotateRadiants(float rad);
		int rotateDeg( int deg );

		void motorsOnOff( bool On );
		void stop();
		void goFW(motCk_t clock);
		void goBK(motCk_t clock);
		void goCW(motCk_t clock);
		void goCCW(motCk_t clock);
		void readSensors();
		void readIrProxySensors();
		void readBumpers();
		void setRele(int16_t releNumber, int16_t onoff);
		bool getReleStatus(int i);
		bool isPowerOn();
		int readBattChargeLevel();
		bool obstacleFree();
		void blinkLed( int port, int mSec );
		//String getOperatingModeStr();
		void readGps();
		void runIBIT( int delayms );	// esegue Initial Built In Test



	private:
		void _begin();

		//bool hasExpired(unsigned long &prevTime, unsigned long interval);
		void _motorCKpulse();
		void _go(motCk_t clock);
		void _motorSetCkPulseDelay( motCk_t clock ); //usato da MoveCm
		void _motorSetPWMCK( motCk_t clock ); //usato da goFW
		void _motorAccelerate();
		motCk_t _motorComputeStepDelay( motCk_t stepDelay );

} ;

extern struct robot_c robot;	//was extern struct robot_c robot;

//ISR(TIMER3_COMPB_vect)
//{
//	robot.currCommand.stepsDone++;
//}


#endif // __ROBOTVAR_H__
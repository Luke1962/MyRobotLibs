#if !defined(__SYSTEMCONFIG_H__)
#define __SYSTEMCONFIG_H__

// Connessioni HW
// Serial  
// Serial1 MMI
#include <Arduino.h>

#define ROBOT_STARTING_MODE  MODE_MODE_SLAVE  //AUTONOMOUS	// 

#define SERIAL_MSG	Serial
#define SERIAL_MSG_BAUD_RATE 115200

#define SERIAL_MMI Serial1			//  MMI > Serial3  Test > Serial
#define SERIAL_MMI_BAUD_RATE 115200


 

// Serial3 GPS
#define SERIAL_GPS Serial3
#define SERIAL_GPS_BAUD_RATE 9600



#endif // 0

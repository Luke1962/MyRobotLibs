#if !defined(__DBGCORE_H__)
	#define __DBGCORE_H__
	#include <Arduino.h>

	#include <systemConfig.h>


//void cmdMsg(const char*   msg) {
//	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.println(";");
//	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.println(";");
//}


//void cmdMsg1(const __FlashStringHelper*   msg) {
//	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.println(";");
//	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.println(";");
//}
//void cmdMsg2(const __FlashStringHelper*   msg, int v) {
//	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(v); SERIAL_MSG.println(";");
//	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.print(v); SERIAL_MMI.println(";");
//}
//void cmdMsg3(const __FlashStringHelper*   msg, int v, const __FlashStringHelper*   msg2) {
//	SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(v); SERIAL_MSG.print(msg2); SERIAL_MSG.println(";");
//	SERIAL_MMI.print("1,"); SERIAL_MMI.print(msg); SERIAL_MMI.print(v); SERIAL_MMI.print(msg2); SERIAL_MMI.println(";");
//}
//#define MSG(s)  	cmdMsg1( F(s));
//#define MSG2(s,v)  	cmdMsg2( F(s),v);
//#define MSG3(s,v,t) cmdMsg3( F(s),v, F(t));

#define MSG(s)  	SERIAL_MSG.print(F("1,")); SERIAL_MSG.print(F(s)); SERIAL_MSG.println(F(";"));SERIAL_MMI.print(F("1,")); SERIAL_MMI.print(F(s)); SERIAL_MMI.println(F(";"));
#define MSGSTR(s)  	SERIAL_MSG.print(F("1,")); SERIAL_MSG.print(s); SERIAL_MSG.println(F(";"));SERIAL_MMI.print(F("1,")); SERIAL_MMI.print(s); SERIAL_MMI.println(F(";"));
#define MSG2(s,v)  	SERIAL_MSG.print(F("1,")); SERIAL_MSG.print(F(s)); SERIAL_MSG.print(v); SERIAL_MSG.println(F(";")); SERIAL_MMI.print(F("1,")); SERIAL_MMI.print(s); SERIAL_MMI.print(v); SERIAL_MMI.println(F(";"));

#define MSG3(s,v,t) SERIAL_MSG.print(F("1,")); SERIAL_MSG.print(F(s)); SERIAL_MSG.print(v); SERIAL_MSG.print(F(t)); SERIAL_MSG.println(";");SERIAL_MMI.print("1,"); SERIAL_MMI.print(F(s)); SERIAL_MMI.print(v); SERIAL_MMI.print(F(t)); SERIAL_MMI.println(";");

#endif // 0

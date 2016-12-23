#include <SPFD5408\SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408\SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408\SPFD5408_TouchScreen.h>     // Touch library

#define SERIAL_IN Serial
#define SERIAL_BAUD_RATE 115200		//115200

// connessioni HW dell'LCD------
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

// caratteristiche fisiche LCD
#define LCD_X_WIDTH 240
#define LCD_Y_HEIGHT 320
#define LCD_TEXT_HEIGHT 16 // Height of text to be printed and scrolled
#define LCD_TEXT_WIDTH 10

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// caratteristiche del Terminale
#define LCD_TEXTCOLOR MAGENTA
#define LCD_BACKGROUND BLACK
#define LCD_TEXTSIZE 2		//20 righe di 24 caratteri (19righe utili se uso la prima come stato con size =1)
#define LCD_CHAR_COLUMNS 24
// The scrolling area must be a integral multiple of LCD_TEXT_HEIGHT
#define LCD_BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define LCD_TOP_FIXED_AREA 24 // 16Number of lines in top fixed area (lines counted from top of screen)
# define LCD_SCROLL_ROWS 5



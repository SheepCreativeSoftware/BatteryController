
#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

uint8_t displayAdress = 0;
bool serialIsSent = 0;
bool displaySenden = 0;
bool blinkPulse;

#ifndef	SerialUSB									//Definiere USB Port, wenn noch nicht definiert
	#define SerialUSB SERIAL_PORT_MONITOR
#endif

void displayStart();
void displayAnzeigen();
uint8_t i2cScan();

#define SCREEN_WIDTH 128 // OLED Display width in pixels
#define SCREEN_HEIGHT 64 // OLED Display height in pixels
// Declaration for a SSD1306 Display on I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if the Arduino shares the Reset Pin with the display)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#endif
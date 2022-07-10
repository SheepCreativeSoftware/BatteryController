/*
 * Battery Controller v0.0.1
 * Date: 10.07.2022 | 10:26
 * <Motorcontroller um einen Regler mit Brushlessmotor anzusteuern und per Tastendruck die Drehzahl zu verändern>
 * Copyright (C) 2020 Marina Egner <info@sheepindustries.de>
 *
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. 
 * If not, see <https://www.gnu.org/licenses/>. 
 * [In Deutsch]: <https://www.gnu.de/documents/gpl-3.0.de.html>
 */

/************************************
 * Programm Konfiguration
 ************************************/
//Ändere diesen Wert für unterschiedliche Debuging level
#define DEBUGLEVEL 3										//0 = Aus | 1 = N/A | >2 = Serial Monitor

/************************************
 * Definition IO Pins
 ************************************/
/* Pinout Arduino Nano:
 * Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
 * Servo Lib deaktiviert PWM funktionalität für pin 9 und 10
 */

//Inputs
//Pin 0+1 Reserved for serial communication via USB!

#define inBattVolts A0					// Messure Battery Volts
#define inBattCurrent A1				// Messure Battery Current

//Outputs
#define outDisplay 2					// Pin to activate Display 
#define statusLED 13					//Pin mit Status LED

/************************************
 * Zusätzliche Dateien einbinden
 ************************************/
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/************************************
 * Definition und Initialiierung
 * Globale Variablen, Klassen und Funktionen
 **********************************/
//Globale Programm Variablen
bool serialIsSent = 0;
bool displaySenden = 0;
bool readSensor = 0;
uint32_t spannungUmgerechnet = 0;
uint32_t spannungVoltDEC0 = 0;
uint32_t spannungVoltDEC1 = 0;
bool blinkPulse;

uint32_t letzteZeit = 0;
uint32_t letzteZeit2 = 0;
int16_t inCurrentRAW = 0;
int16_t inCurrentNorm = 0;
int16_t inCurrentVolts = 2500;
int16_t inCurrentRAWTemp = 512;
const int16_t filterFactorRead = 50;
const int16_t filterFactorCurrent = 10;
const int16_t correctionVolts = 17;


uint8_t displayAdress = 0;

#ifndef	SerialUSB									//Definiere USB Port, wenn noch nicht definiert
	#define SerialUSB SERIAL_PORT_MONITOR
#endif

//Funktionen
void displayStart();
void displayAnzeigen();
uint8_t i2cScan();
int16_t rawToVolts(int16_t);
int16_t voltsToCurrent(int16_t);
int16_t filterT1(int16_t, int16_t, int16_t);

#define SCREEN_WIDTH 128 // OLED Display Breite, in pixels
#define SCREEN_HEIGHT 64 // OLED Display Höhe, in pixels
// Deklaration für ein SSD1306 Display an I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (oder -1 wenn der Arduino sich den Reset Pin teilt)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
	
	pinMode(inBattVolts, INPUT);
	pinMode(inBattCurrent, INPUT);
	pinMode(outDisplay, OUTPUT);
	#if (DEBUGLEVEL >=1)									// Bedingte Kompilierung
		pinMode(statusLED, OUTPUT);							// Status LED definieren zur Anzeige des Status
	#endif		
	#if (DEBUGLEVEL >=2)									// Bedingte Kompilierung
		SerialUSB.begin(9600);  							// Starte Serielle Kommunikation per USB
	#endif
	digitalWrite(outDisplay, HIGH);
	displayStart();										// Überprüfe zum Start die Adresse des Displays und starte Display Library
	delay(2000);


}

void loop() {
	int16_t inBattCurrentTemp = analogRead(inBattCurrent) * 10;
	inCurrentRAWTemp = filterT1(inBattCurrentTemp, inCurrentRAWTemp, filterFactorRead);
	inCurrentRAW = inCurrentRAWTemp / 10;
	inCurrentVolts = rawToVolts(inCurrentRAW) + correctionVolts;
	if((millis()%500 >= 250) && (readSensor == false)) {	// Führe nur in bestimmten Zeit Abstand aus
		readSensor = true;
		inCurrentNorm = filterT1(voltsToCurrent(inCurrentVolts), inCurrentNorm, filterFactorCurrent);
	} else if((millis()%500 < 250) && (readSensor == true)) {
		readSensor = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
	}
	displayAnzeigen();									// Darstellung des Display laden
}

int16_t rawToVolts(int16_t raw) {
	return map(raw, 0, 1023, 0, 5000);
}

int16_t voltsToCurrent(int16_t volts) {
	return map(volts, 500, 4500, -20000, 20000);
}


int16_t filterT1(int16_t inValue, int16_t currentValue, int16_t filterFactor) {
// 	Aktueller Messwert=(Neuer Messwert - Aktueller Messwert) / Glättungsfaktor Aktueller Messwert 
// Aktueller Messwert=(14,3 - 10,0) / 10 10,0 = 4,3 / 10 10,0 = 10,4 °C 
	return (inValue - currentValue) / filterFactor + currentValue;
}

void displayStart() {										// Überprüfe zum Start die Adresse des Displays und starte Display Library
	
	Wire.begin();
	displayAdress = EEPROM.read(0);							// lese Display Adresse von EEPROM Adresse 0
	#if (DEBUGLEVEL >=3)									// Bedingte Kompilierung
		SerialUSB.print(F("EEPROM Wert: 0x"));
		if (displayAdress<16) {
				SerialUSB.print(F("0"));
			}
		SerialUSB.println(displayAdress,HEX);
	#endif
	
	uint8_t error;
	if(displayAdress >= 127) {
		displayAdress = i2cScan();
		EEPROM.update(0, displayAdress);				// Speichere Adresse auf EEPROM
	} else {
		Wire.beginTransmission(displayAdress);
		error = Wire.endTransmission();						// Wenn Rückgabewert ungleich 0, dann ist kein Gerät mit der Adresse auf dem Bus
		if(error != 0) {
			#if (DEBUGLEVEL >=3)							// Bedingte Kompilierung
				SerialUSB.println(F("Adresse aus EMPROM nicht ansprechbar"));
			#endif
			displayAdress = i2cScan();
			EEPROM.update(0, displayAdress);				// Speichere Adresse auf EEPROM
		} else {
			#if (DEBUGLEVEL >=3)							//Bedingte Kompilierung
				SerialUSB.println(F("Adresse aus EEPROM ansprechbar!"));
			#endif
		}
	}
	// Starte Display, falls nicht möglich setze Fehler
	if(!display.begin(SSD1306_SWITCHCAPVCC, displayAdress, true, false)) { // Address 0x3D for 128x64
		#if (DEBUGLEVEL >=3)
		Serial.println(F("SSD1306 Zuweisung gescheitert"));
		#endif
		for(;;); // Nicht weiter machen, Dauerschleife
	}

	// Lösche und aktualisiere Display
	display.clearDisplay();
	display.display();
}

void displayAnzeigen() {									// Darstellung des Display laden
	if((millis()%500 >= 250) && (displaySenden == false)) {	// Führe nur in bestimmten Zeit Abstand aus
		blinkPulse = !blinkPulse;
		display.clearDisplay();
		display.setCursor(0,0);             				// Start at top-left corner
		display.setTextSize(2);             				// Normal 1:1 pixel scale
		display.setTextColor(SSD1306_WHITE);        		// Male weißen text
		if(spannungVoltDEC0 < 10) {
			display.print(F(" "));
		}
		display.print(spannungVoltDEC0);
		display.print(F("."));
		display.print(spannungVoltDEC1);
		display.println(F("V"));		
        display.fillRect(65, 4, 4, 6, SSD1306_WHITE); 		// Batterie Pluspol | x,y,width,height,color    
		display.drawRect(69, 0, 58, 15, SSD1306_WHITE); 	// Batterie Rahmen | x,y,width,height,color
		
		if(spannungUmgerechnet >= 20000) {
			display.fillRect(71, 3, 12, 9, SSD1306_WHITE); 	// Batterie 4/4 voll | x,y,width,height,color
		} else {
			
		}
		if(spannungUmgerechnet >= 19000) {
			display.fillRect(85, 3, 12, 9, SSD1306_WHITE); 	// Batterie 3/4 voll | x,y,width,height,color
		} else {
			display.drawLine(84, 3, 84, 11, SSD1306_WHITE); // Batterie Trennstrich | x,y,width,height,color
		}
		if(spannungUmgerechnet >= 18000) {
			display.fillRect(99, 3, 12, 9, SSD1306_WHITE); 	// Batterie 4/4 voll | x,y,width,height,color
		} else {
			display.drawLine(98, 3, 98, 11, SSD1306_WHITE); // Batterie Trennstrich | x,y,width,height,color
		}
		if(spannungUmgerechnet >= 17000) {
			display.fillRect(113, 3, 12, 9, SSD1306_WHITE); // Batterie 1/4 voll | x,y,width,height,color
		} else {
			if(blinkPulse) {
				display.fillRect(113, 3, 12, 9, SSD1306_WHITE); 	//Batterie 1/4 voll | x,y,width,height,color	
			} else {
				display.drawLine(112, 3, 112, 11, SSD1306_WHITE); //Batterie Trennstrich | x,y,width,height,color
			}
		}

		display.setCursor(8,20); // Start at top-left corner
		display.setTextSize(1);  
		display.println(F("RAW"));
		display.setCursor(8,30); // Start at top-left corner
		display.setTextSize(1);
		display.println(inCurrentRAW);
		display.setCursor(64,20); // Start at top-left corner
		display.setTextSize(1);  
		display.println(F("AMPS"));
		display.setCursor(64,30); // Start at top-left corner
		display.setTextSize(1);
		display.println(inCurrentNorm);
		display.setCursor(8,50); // Start at top-left corner
		display.setTextSize(1);
		display.println(inCurrentVolts);
		display.setCursor(64,50); // Start at top-left corner
		display.println(F("mV"));
		display.display();
		displaySenden = true;
	} else if((millis()%500 < 250) && (displaySenden == true)) {
		displaySenden = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
	}
	
}

uint8_t i2cScan() {											// Scanne I2C Bus, um Adresse des Display herauszufinden
	uint8_t error, address, outValue;
	uint8_t nDevices = 0;
	#if (DEBUGLEVEL >= 3)
	SerialUSB.println(F("--Scane I2C Bus--"));
	#endif
	for(address = 1; address <=127 ; address++ ) {
	// der Scan verwendet den Rückgabewert von Write.endTransmisstion 
	// um zu sehen ob ein Gerät angewortet hat
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		#if (DEBUGLEVEL >= 3)
			SerialUSB.print(F("Prüfe I2C Adresse: 0x"));
			if (address<16) {
				SerialUSB.print(F("0"));
			} 
			SerialUSB.println(address, HEX);
		#endif
		if (error == 0) {
			nDevices++;
			outValue = address;
			address = 127;
		}
	}
	if (nDevices == 0) {
		outValue = 0;
		#if (DEBUGLEVEL >= 3)
			SerialUSB.println(F("--Kein Gerät gefunden--"));
		#endif
	} else {
		#if (DEBUGLEVEL >= 3)
			SerialUSB.print(F("Gerät gefunden auf Adresse: 0x"));
			if (address<16) {
				SerialUSB.print(F("0"));
			} 
			SerialUSB.println(address, HEX);
		#endif
	}
		
	return outValue;
}
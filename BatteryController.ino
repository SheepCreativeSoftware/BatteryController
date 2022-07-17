/*
 * Battery Controller v0.0.1
 * Date: 10.07.2022 | 10:26
 * <Controller to measure energy consumption and estimate State of Charge>
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
 * Program Configuration
 ************************************/
//Change this for different Debuging levels
#define DEBUGLEVEL 3										//0 = Off | 1 = N/A | >2 = Serial Monitor

/************************************
 * Definition IO Pins
 ************************************/
/* Pinout Arduino Nano:
 * Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
 */

//Inputs
//Pin 0+1 Reserved for serial communication via USB!

#define inBattVolts A0					// Messurement Battery Volts
#define inBattCurrent A1				// Messurement Battery Current

//Outputs
#define outDisplay 2					// Pin to activate Display 
#define statusLED 13					// Pin with internal Status LED

/************************************
 * Include nessecary libs
 ************************************/
#include "Display.h"

/************************************
 * Global Vars
 ************************************/
//Global vars

bool readSensor = 0;

uint32_t letzteZeit = 0;
uint32_t letzteZeit2 = 0;
int16_t currentRawFiltered = 512;
const int16_t filterFactorRead = 50;
const int16_t filterFactorCurrent = 10;
const int16_t correctionVolts = 17;
#define SENSOR_MID_VOLTAGE 2500;
#define SENSOR_FACTOR_PER_AMP 66; //66mV/A
int16_t currentMilliAmps = 0;
int16_t currentNorm = 0;
int32_t currentMicrosAmpHours = 0;


//Funktionen
int16_t rawToVolts(int16_t);
int16_t voltsToCurrent(int16_t);
int16_t filterT1(int16_t, int16_t, int16_t);
int32_t voltsToAmps(int32_t);
int32_t calcMilliAmpHours(int32_t, int32_t);
uint16_t readBattVolts(uint8_t);

#ifndef	SerialUSB									//Definiere USB Port, wenn noch nicht definiert
	#define SerialUSB SERIAL_PORT_MONITOR
#endif

void setup() {
	pinMode(inBattVolts, INPUT);
	pinMode(inBattCurrent, INPUT);
	#if (DEBUGLEVEL >=1)									// Bedingte Kompilierung
		pinMode(statusLED, OUTPUT);							// Status LED definieren zur Anzeige des Status
	#endif		
	#if (DEBUGLEVEL >=2)									// Bedingte Kompilierung
		SerialUSB.begin(115200);  							// Starte Serielle Kommunikation per USB
	#endif
	digitalWrite(outDisplay, HIGH);
	displayStart();										// Überprüfe zum Start die Adresse des Displays und starte Display Library
	uint16_t inBattVolts = readBattVolts(inBattVolts);
	delay(2000);
}



void loop() {
	int16_t inBattCurrentTemp = analogRead(inBattCurrent) * 10;
	currentRawFiltered = filterT1(inBattCurrentTemp, currentRawFiltered, filterFactorRead);
	int16_t tempCurrentRaw = currentRawFiltered / 10;
	int16_t currentInVolts = rawToVolts(tempCurrentRaw) + correctionVolts;
	int32_t currentMicrosAmps = voltsToAmps(currentInVolts);
	currentMilliAmps = currentMicrosAmps / 1000;

	if((millis()%500 >= 250) && (readSensor == false)) {	// Führe nur in bestimmten Zeit Abstand aus
		readSensor = true;
		currentNorm = filterT1(currentMilliAmps, currentNorm, filterFactorCurrent);
		currentMicrosAmpHours = calcMilliAmpHours(currentMicrosAmps, currentMicrosAmpHours);
		int16_t currentMilliAmpHours = currentMicrosAmpHours / 1000;
	} else if((millis()%500 < 250) && (readSensor == true)) {
		readSensor = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
	}
	displayAnzeigen();									// Darstellung des Display laden

}

#define DAC_MIN_VAL 0
#define DAC_MAX_VAL 1023
#define RESISTOR_R1 30000.0								//Widerstand gegen V+ für Spannungsmessung in Ohm (Wichtig Kommastelle(bzw. Punkt:'.') muss vorhanden sein)
#define RESISTOR_R2 7500.0								//Widerstand gegen GND für Spannungsmessung in Ohm (Wichtig Kommastelle(bzw. Punkt:'.') muss vorhanden sein)
#define VOLTS_CORRECTION 0								//Korrektur der Spannung in mV
//Funktion zum Berechnen der Spannung (Vout Ausgang Spannungsteiler/Eingang Arduino | Vin Eingang Spannungteiler/Akku)
//Vout = Vin * (R2 / (R1 + R2))
//Vin = Vout / (R2 / (R1 + R2))
#define VOLTS_OUT_MAX 5000
#define VOLTS_OUT_MIN 0
#define VOLTS_IN_MAX VOLTS_OUT_MAX / (RESISTOR_R2 / (RESISTOR_R2 + RESISTOR_R1))
#define VOLTS_IN_MIN 0
#define VOLTS_IN_MAX_CALC (VOLTS_IN_MAX - VOLTS_CORRECTION)
#define SPANNUNG_IN_BEREICH (VOLTS_IN_MAX_CALC - VOLTS_IN_MIN)


uint16_t readBattVolts(uint8_t batteryPin) {
	uint16_t inBattRaw = analogRead(batteryPin);
	return map(inBattRaw, DAC_MIN_VAL, DAC_MAX_VAL, VOLTS_IN_MIN, VOLTS_IN_MAX_CALC);
}

int32_t calcMilliAmpHours(int32_t microsAmps, int32_t microsAmpHours) {
	int32_t tempMicrosAmpHours = microsAmps / 7200;
	return microsAmpHours + tempMicrosAmpHours;
}

int32_t voltsToAmps(int16_t volts) {
	int32_t tempCurrents = volts - SENSOR_MID_VOLTAGE;
	tempCurrents *= 1000;
	return tempCurrents / SENSOR_FACTOR_PER_AMP;
}

int16_t rawToVolts(int16_t raw) {
	return map(raw, DAC_MIN_VAL, DAC_MAX_VAL, VOLTS_OUT_MIN, VOLTS_OUT_MAX);
}

int16_t voltsToCurrent(int16_t volts) {
	return map(volts, 500, 4500, -5000, 5000);
}


int16_t filterT1(int16_t inValue, int16_t currentValue, int16_t filterFactor) {
// 	Aktueller Messwert=(Neuer Messwert - Aktueller Messwert) / Glättungsfaktor Aktueller Messwert 
// Aktueller Messwert=(14,3 - 10,0) / 10 10,0 = 4,3 / 10 10,0 = 10,4 °C 
	return (inValue - currentValue) / filterFactor + currentValue;
}


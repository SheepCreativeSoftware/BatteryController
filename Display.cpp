#include "Display.h"

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
		// if(spannungVoltDEC0 < 10) {
		// 	display.print(F(" "));
		// }
		// display.print(spannungVoltDEC0);
		// display.print(F("."));
		// display.print(spannungVoltDEC1);
		// display.println(F("V"));		
        // display.fillRect(65, 4, 4, 6, SSD1306_WHITE); 		// Batterie Pluspol | x,y,width,height,color    
		// display.drawRect(69, 0, 58, 15, SSD1306_WHITE); 	// Batterie Rahmen | x,y,width,height,color
		
		// if(spannungUmgerechnet >= 20000) {
		// 	display.fillRect(71, 3, 12, 9, SSD1306_WHITE); 	// Batterie 4/4 voll | x,y,width,height,color
		// } else {
			
		// }
		// if(spannungUmgerechnet >= 19000) {
		// 	display.fillRect(85, 3, 12, 9, SSD1306_WHITE); 	// Batterie 3/4 voll | x,y,width,height,color
		// } else {
		// 	display.drawLine(84, 3, 84, 11, SSD1306_WHITE); // Batterie Trennstrich | x,y,width,height,color
		// }
		// if(spannungUmgerechnet >= 18000) {
		// 	display.fillRect(99, 3, 12, 9, SSD1306_WHITE); 	// Batterie 4/4 voll | x,y,width,height,color
		// } else {
		// 	display.drawLine(98, 3, 98, 11, SSD1306_WHITE); // Batterie Trennstrich | x,y,width,height,color
		// }
		// if(spannungUmgerechnet >= 17000) {
		// 	display.fillRect(113, 3, 12, 9, SSD1306_WHITE); // Batterie 1/4 voll | x,y,width,height,color
		// } else {
		// 	if(blinkPulse) {
		// 		display.fillRect(113, 3, 12, 9, SSD1306_WHITE); 	//Batterie 1/4 voll | x,y,width,height,color	
		// 	} else {
		// 		display.drawLine(112, 3, 112, 11, SSD1306_WHITE); //Batterie Trennstrich | x,y,width,height,color
		// 	}
		// }

		display.setCursor(8,20); // Start at top-left corner
		display.setTextSize(1);  
		display.println(F("RAW"));
		display.setCursor(8,30); // Start at top-left corner
		display.setTextSize(1);
		//display.println(inCurrentRAW);
		display.setCursor(64,20); // Start at top-left corner
		display.setTextSize(1);  
		display.println(F("AMPS"));
		display.setCursor(64,30); // Start at top-left corner
		display.setTextSize(1);
		//display.println(inCurrentNorm);
		display.setCursor(8,50); // Start at top-left corner
		display.setTextSize(1);
		//display.println(inCurrentVolts);
		display.setCursor(64,50); // Start at top-left corner
		// display.println(currentNorm);
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
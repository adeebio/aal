// Include the Arduino Library - to access the standard
// types and constants of the Arduino language.

#include <Arduino.h>
#include <LiquidCrystal.h>

// Include the AAL header file where the class methods
// and properties are declared.

#include <AAL.h>

// Constructor method.

AALClass::AALClass () {
	// Initialise the LiquidCrystal Library with the numbers of
	// the interface pins.
	//LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
	
	// Set up the LCD's number of columns and rows: 
	//lcd.begin(16, 2);
	//lcd.print("hello, world!");
}

// Initialize an AAL object.

AALClass AAL;
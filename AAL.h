// If statement to make sure the AAL Library is not
// included more than once.

#ifndef AAL_h
#define AAL_h

// Include the Arduino Library - to access the standard
// types and constants of the Arduino language.

#include <Arduino.h>
#include <LiquidCrystal.h>

// Declare (and spec out) the AALClass class.
class AALClass {
	// Public methods and properties.
	public:
		// Constructor method.
		AALClass();
		
		// Public methods.
		
		// Public properties.
	
	// Private methods and properties.
	private:
		// Private methods.
		
		// Private properties.
};

// Code to remove the need to initialize an AAL
// object in sketch.

extern AALClass AAL;

#endif
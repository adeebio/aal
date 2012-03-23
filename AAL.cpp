// Include the Arduino Library - to access the standard
// types and constants of the Arduino language.

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>	
#include <ACTLab.h>

// Include the AAL header file where the class methods
// and properties are declared.

#include <AAL.h>

// Declare objects in use.
LiquidCrystal LCD(2, 3, 4, 5, 6, 7);

// Constructor method.

AALClass::AALClass () {
	// Set private properties.
		// Calibration:
		_analogInputUpperBound		= 1023;
		_PWMOutputUpperBound		= 255;
		
		// Voltage safety limits:
		_motorVoltageCutOff			= 8.5; // /V
		
		// Rig Connections:
		_rotor1MotorPWMPin			= 10;
		_rotor2MotorPWMPin			= 11;
		_rotor1TachoAnalogPin		= 0;
		_rotor2TachoAnalogPin		= 1;
		
		// Button pins:
		_btnInternetPin				= 9; // Digital
		_btnRigPin					= 8; // Digital
		_btnPreviousPin				= 2; // Analog
		_btnNextPin					= 3; // Analog
		_btnGoPin					= 4; // Analog
		
		// Data submissions:
		_submitSerial				= 0;
		_submitServer				= 0;
		
		// Others:
		_baud						= 9600;
		_serial						= 0;
		
	// Specify whether digital pins are inputs or outputs.
	pinMode(_rotor1MotorPWMPin,OUTPUT);
	pinMode(_rotor2MotorPWMPin,OUTPUT);
	pinMode(_btnInternetPin,INPUT);
	pinMode(_btnRigPin,INPUT);
	
	// Specify the size of the LCD.
	LCD.begin(16,2);
	
	// Configure server access.
	ACTLab.rig("tr");
	ACTLab.MAC(0x90,0xA2,0xDA,0x00,0x7F,0xAB);
}

// AAL.setup()

void AALClass::setup () {
	// If either _serial or _submitSerial = 1 then the Serial needs to be turned on.
	if ((_serial==1)||(_submitSerial==1)) {Serial.begin(_baud);};
	
	// If data is being sent to the server the ethernet shield needs to be started.
	if (_submitServer==1) {ACTLab.startEthernet();};
}

// AAL.R1M()

bool AALClass::R1M (double voltage) {
	// Voltage, plus 10, divide by 20, times by _PWMOutputUpperBound.
	double value = (((voltage+10)/20)*_PWMOutputUpperBound);
	Serial.println(value);
	analogWrite(_rotor1MotorPWMPin,value);
}

// AAL.R2M()

bool AALClass::R2M (double voltage) {
	// Voltage, plus 10, divide by 20, times by _PWMOutputUpperBound.
	double value = (((voltage+10)/20)*_PWMOutputUpperBound);
	analogWrite(_rotor2MotorPWMPin,value);
}

// AAL.R1T()

double AALClass::R1T () {
	// Analog Read, divide by _analogInputUpperBound, times 20, minus 10.
	double value = (((analogRead(_rotor1TachoAnalogPin)/_analogInputUpperBound)*20)-10);
	return (value);
}

// AAL.R2T()

double AALClass::R2T () {
	// Analog Read, divide by _analogInputUpperBound, times 20, minus 10.
	double value = (((analogRead(_rotor2TachoAnalogPin)/_analogInputUpperBound)*20)-10);
	return (value);
}

// AAL.btnInternet()

bool AALClass::btnInternet () {
	if (digitalRead(_btnInternetPin)==HIGH) {return false;}
	else {return true;};
}

// AAL.btnRig()

bool AALClass::btnRig () {
	if (digitalRead(_btnRigPin)==HIGH) {return false;}
	else {return true;};
}

// AAL.btnPrevious()

bool AALClass::btnPrevious () {
	if (analogRead(_btnPreviousPin)>128) {return false;}
	else {return true;};
}

// AAL.btnNext()

bool AALClass::btnNext () {
	if (analogRead(_btnNextPin)>128) {return false;}
	else {return true;};
}

// AAL.btnGo()

bool AALClass::btnGo () {
	if (analogRead(_btnGoPin)>128) {return false;}
	else {return true;};
}

// AAL.serial()

void AALClass::serial (int arg) {
	if (arg==0||arg==1) {_serial = arg;};
}

// AAL.serialPrint()

void AALClass::serialPrint (char str[]) {
	if (_serial) {Serial.print(str);};
}

// AAL.serialPrintln()

void AALClass::serialPrintln (char str[]) {
	if (_serial) {Serial.println(str);};
}

// AAL.LCDPrint()

void AALClass::LCDPrint (char str[]) {
	LCD.print(str);
}

// AAL.submit()

void AALClass::submit (double time, double input, double output) {
	if (_submitSerial) {submitSerial(time,input,output);};
	if (_submitServer) {submitServer(time,input,output);};
}

// AAL.submitSerialSet()

void AALClass::submitSerialSet (int arg) {
	if (arg==0||arg==1) {_submitSerial = arg;};
}

// AAL.submitServerSet()

void AALClass::submitServerSet (int arg) {
	if (arg==0||arg==1) {_submitServer = arg;};
}

// AAL.submitSerial()

void AALClass::submitSerial (double time, double input, double output) {
	Serial.print(time);
	Serial.print(";");
	Serial.print(input);
	Serial.print(";");
	Serial.println(output);
}

// AAL.submitServer()

void AALClass::submitServer (double time, double input, double output) {
	ACTLab.submitData(time,input,output);
}

// ======================================================================
// ================================================== Experiments : Start
// ======================================================================



// ======================================================================
// ==================================================== Experiments : End
// ======================================================================

// Initialize an AAL object.

AALClass AAL;
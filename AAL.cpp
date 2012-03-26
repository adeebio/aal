// Include the Arduino Library - to access the standard
// types and constants of the Arduino language. Also include
// any other libraries needed by this one.

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>	
#include <ACTLab.h>

// Include the AAL header file where the class methods
// and properties are declared.

#include <AAL.h>

// Declare the LCD object with the correct pin configurations.
LiquidCrystal LCD(2, 3, 13, 5, 6, 7);

// ======================================================================
// =================================================== Constructor Method

AALClass::AALClass () {
	// Set private properties.
		// Calibration:
		_analogInputUpperBound		= 1023;
		_PWMOutputUpperBound		= 255;
		
		// Voltage safety limits:
		_motorVoltageCutOff			= 8.5; // /V
		
		// Rig Connections:
		_rotor1MotorPWMPin			= 12;
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
		_submitSerial				= 1;
		_submitServer				= 1;
		
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

// ======================================================================
// ============================================================ AAL Setup

void AALClass::setup () {
	// If either _serial or _submitSerial = 1 then the Serial needs to be turned on.
	if ((_serial==1)||(_submitSerial==1)) {Serial.begin(_baud);};
	
	// Settle the rig.
	R1M(0); R2M(0);
	
	// If data is being sent to the server the ethernet shield needs to be started.
	if (_submitServer==1) {
		serialPrintln("Starting ethernet (setup).");
		ACTLab.startEthernet();
		serialPrintln("Ethernet started (setup).");
	};
}

// ======================================================================
// =============================================== Rig Inputs and Outputs

bool AALClass::R1M (double voltage) {
	// Variables
	double _voltage;
	bool _within;
	double _value;
	
	// Check if requested voltage is within range.
	if (voltage < ( - _motorVoltageCutOff)) {
		_voltage = ( - _motorVoltageCutOff);
		_within = false;
	} else if (voltage > _motorVoltageCutOff) {
		_voltage = _motorVoltageCutOff;
		_within = false;
	} else {
		_voltage = voltage;
		_within = true;
	};
	
	// Calculate voltage PWM value:
	// Voltage, plus 10, divide by 20, times by _PWMOutputUpperBound.
	_value = (((_voltage+((double)10))/((double)20))*_PWMOutputUpperBound);
	
	// Send PWM value to rig.
	analogWrite(_rotor1MotorPWMPin,_value);
	
	return _within;
}

// AAL.R2M()

bool AALClass::R2M (double voltage) {
	// Variables
	double _voltage;
	bool _within;
	double _value;
	
	// Check if requested voltage is within range.
	if (voltage < ( - _motorVoltageCutOff)) {
		_voltage = ( - _motorVoltageCutOff);
		_within = false;
	} else if (voltage > _motorVoltageCutOff) {
		_voltage = _motorVoltageCutOff;
		_within = false;
	} else {
		_voltage = voltage;
		_within = true;
	};
	
	// Calculate voltage PWM value:
	// Voltage, plus 10, divide by 20, times by _PWMOutputUpperBound.
	_value = (((_voltage+((double)10))/((double)20))*_PWMOutputUpperBound);
	
	// Send PWM value to rig.
	analogWrite(_rotor2MotorPWMPin,_value);
	
	return _within;
}

// AAL.R1T()

double AALClass::R1T () {
	// Analog Read, divide by _analogInputUpperBound, times 20, minus 10.
	double value = (((analogRead(_rotor1TachoAnalogPin)/_analogInputUpperBound)*((double)20))-((double)10));
	return (value);
}

// AAL.R2T()

double AALClass::R2T () {
	// Analog Read, divide by _analogInputUpperBound, times 20, minus 10.
	double value = (((analogRead(_rotor2TachoAnalogPin)/_analogInputUpperBound)*((double)20))-((double)10));
	return (value);
}

// ======================================================================
// ============================================================== Buttons

bool AALClass::btnInternet () {
	if (digitalRead(_btnInternetPin)==HIGH) {return false;} else {return true;}; }

bool AALClass::btnRig () {
	if (digitalRead(_btnRigPin)==HIGH) {return false;} else {return true;}; }

bool AALClass::btnPrevious () {
	if (analogRead(_btnPreviousPin)>128) {return false;} else {return true;}; }

bool AALClass::btnNext () {
	if (analogRead(_btnNextPin)>128) {return false;} else {return true;}; }

bool AALClass::btnGo () {
	if (analogRead(_btnGoPin)>128) {return false;} else {return true;}; }

// ======================================================================
// ====================================================== Serial Messages

// AAL.serial()

void AALClass::serial (int arg) { if ( arg==0||arg==1) {_serial = arg;}; }

// AAL.serialPrint()

void AALClass::serialPrint (char str[]) { if (_serial) {Serial.print(str);}; }

// AAL.serialPrintln()

void AALClass::serialPrintln (char str[]) { if (_serial) {Serial.println(str);}; }

// ======================================================================
// ================================================================== LCD

// AAL.LCDPrint()

void AALClass::LCDPrint (char str[]) {
	LCD.print(str);
}

// ======================================================================
// ===================================================== Data Submissions

// AAL.submit()

void AALClass::submit (double time, double reference, double input, double output) {
	if (_submitSerial) {submitSerial(time, reference, input, output);};
	if (_submitServer) {submitServer(time, reference, input, output);};
}

// AAL.submitSerialSet()

void AALClass::submitSerialSet (int arg) { if (arg==0||arg==1) {_submitSerial = arg;}; }

// AAL.submitServerSet()

void AALClass::submitServerSet (int arg) { if (arg==0||arg==1) {_submitServer = arg;}; }

// AAL.submitSerial()

void AALClass::submitSerial (double time, double reference, double input, double output) {
	Serial.print(time);
	Serial.print(";");
	Serial.print(reference);
	Serial.print(";");
	Serial.print(input);
	Serial.print(";");
	Serial.println(output);
}

// AAL.submitServer()

void AALClass::submitServer (double time, double reference, double input, double output) {
	ACTLab.submitData(time,input,output);
}

// ======================================================================
// ==================================================== Reference Signals

// AAL.ref_step()

double AALClass::ref_step (double time, double start, double end, double amplitude) {
	if ((time>start)&&(time<end)) {return amplitude;}
	else {return (double)0;};
}

// AAL.ref_sine()

double AALClass::ref_sine (double time, double start, double end, double amplitude, double frequency) {
	if ((time>start)&&(time<end)) {
		double timePassed = time - start;
		return (amplitude*(sin(timePassed*frequency)));
	} else {return (double)0;};
}

// AAL.ref_scale()

double AALClass::ref_scale (double time, double start, double end, double bias, double amplitude,
								double frequencyStart, double frequencyEnd) {
}

// ======================================================================
// =================================================== Experiment Methods

// AAL.exp_step()

bool AALClass::exp_step (double start, double end, double amplitude) {
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	for (float time = 0; time <= end; time = time + 0.1) {
		float input =  ref_step(time,start,end,amplitude);
		R1M(input);
		submit(time, input, input, R1T());
		delay(100);
	};
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	return true;
}

// AAL.exp_scale()

bool AALClass::exp_scale (double start, double end, double bias, double amplitude, double frequencyStart, double frequencyEnd) {
	// Variables
	float _time_total		= end;
	float _time_increment	= 0.1;
	float _time_current		= 0;
	float _steps			= _time_total/_time_increment;
	
	float _freq_start		= frequencyStart;
	float _freq_end			= frequencyEnd;
	float _freq_current		= frequencyStart;
	float _freq_increment	= (_freq_end - _freq_start)/_steps;
	
	float _amplitude		= amplitude;
	float _bias				= bias;
	float _currentPhase		= 0;

	// Settle the rig.
	R1M(_bias);R2M(0);delay(5000);
	
	// Control loop.
	for (float i = 0; i <= _steps; i++) {
		float temp_time = _time_current;
		float temp_input = _bias + ((sin(_freq_current*_currentPhase))*_amplitude);
		float temp_output = R1T();
		
		R1M(temp_input);
		submit(temp_time,temp_input,temp_input,temp_output);
		
		_time_current = _time_current + _time_increment;
		_currentPhase = ((_freq_current * _currentPhase)/(_freq_current + _freq_increment)) + _time_increment;
		_freq_current = _freq_current + _freq_increment;
		
		delay(_time_increment*1000);
	};
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	return true;
}

// AAL.exp_p_step()

bool AALClass::exp_p_step (double start, double end, double amplitude, double Kp, double delta) {
	// Variables.
	float _Kp = Kp;			// 8.0289
	float _delta = delta;	// 0.02 s
	float _tLoopStart;
	float _tLoop;
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	// Control loop.
	for (float time = 0; time <= end; time = time + _delta) {
		
		Serial.print("_delta: ");
		Serial.print(_delta,5);
		
		// Get time reference at the start of loop.
		_tLoopStart = millis()/((float)1000);
		
		// Determine error, e(k).
		//float e_k_ = (ref_step(time,start,end,amplitude)) - R1T();
		
		// Send input to rig.
		//R1M((_Kp*e_k_));
		
		// Submit data.
		//submit (time, (_Kp*e_k_), (_Kp*e_k_), R1T());
		submit(5, 5, 5, 5);
		
		Serial.print("_tLoopStart: ");
		Serial.print(_tLoopStart,5);
		
		// Get time since start of loop.
		_tLoop = (millis()/((float)1000)) - _tLoopStart;
		
		Serial.print("     _tLoop: ");
		Serial.print(_tLoop,5);
		
		// Check loop length and act accordingly.
		if (_tLoop < delta) {
			delay((delta - _tLoop)*1000);
			
			Serial.print("     delay: ");
			Serial.print((delta - _tLoop)*1000,5);
			
			_delta = delta;
		} else {
			_delta = _tLoop;
		};
		
		Serial.println("");
	};
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	return true;
}

// ======================================================================

// Initialize an AAL object.

AALClass AAL;
// Include the AAL header file where the class methods
// and properties are declared.

#include <AAL.h>

// Declare the LCD object with the correct pin configurations.
LiquidCrystal LCD(2, 3, 13, 5, 6, 7);

// ======================================================================
// =================================================== Constructor Method

AALClass::AALClass () {
	// Set private properties.
		// Core method properties:
		_LCD_initialModeSet			= false;
		_LCD_modeInternet			= btnInternet();
		_currentExpNum				= 1;
		_btnPreviousPressed			= false;
		_btnNextPressed				= false;
		_btnGoPressed				= false;
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

// ======================================================================
// ===================================== AAL Start and Other Core Methods

// AAL.setup()

void AALClass::setup () {
	// Begin the serial connection with a 9600 baud.
	Serial.begin(9600);
	
	// Configure ACTLab Library.
	ACTLab.rig("tr");
	ACTLab.MAC(0x90,0xA2,0xDA,0x00,0x7F,0xAB);
	ACTLab.SDBuffer(0);              // 1 = Intending to use SD Buffer. [Default: 0]
	ACTLab.serialMessages(1);        // 1 = Print ACTLab progress messages to serial. [Default: 0]
	
	// Start ACTLab Library.
	ACTLab.start();
	
	// Change any AAL settings if needed.
	AAL.submitSerialSet(1);          // 1 = Send data to serial. [Default: 1]
	AAL.submitServerSet(0);          // 1 = Send data to server. [Default: 0]
	AAL.serialMessages(1);           // 1 = Print AAL progress messages to serial. [Default: 0]
}

// AAL.start()

void AALClass::start (int setupRequested) {
	// Settle the rig first.
	R1M(0); R2M(0);
	
	// Set up the LCD's number of columns and rows.
	LCD.begin(16, 2);
	
	// Initialising message.
	LCDPrint("Initialising...","");
	
	// Check to see if setup is needed.
	if (setupRequested==1) {setup();};
	
	// Welcome message.
	LCDPrint("     Hello!","I'm Rotorduino!!");
	delay(3000);
	
	// Enter the rigLoop.
	rigLoop();
}

// AAL.rigLoop()

void AALClass::rigLoop () {
	while (true) {
		if (btnInternet()) {
			// Internet Mode.
			modeInternet();
		} else {
			// Rig Mode.
			modeRig();
		};
	};
}

// AAL.modeInternet()

void AALClass::modeInternet () {
	// Check if LCD needs to be changed.
	if ((!_LCD_modeInternet)||(!_LCD_initialModeSet)) {
		_LCD_modeInternet=true;
		_LCD_initialModeSet=true;
		LCDPrint("Mode: Internet","");
	};
}

// AAL.modeRig()

void AALClass::modeRig () {
	// Set _numOfExperiments.
	_numOfExperiments = modeRigExperiments(1);
	
	// Check if LCD needs to be changed (if so, also reset rig experiment number).
	if (_LCD_modeInternet||(!_LCD_initialModeSet)) {
		_LCD_modeInternet=false;
		_LCD_initialModeSet=true;
		_currentExpNum = 1;
		modeRigSetLCD();
	};
	
	// If previous button is pushed.
	if (btnPrevious()) {
		// Check to see if it isn't the same press.
		if (!_btnPreviousPressed) {
			_btnPreviousPressed = true;
			
			// Set _currentExpNum and update LCD message.
			if (_currentExpNum == 1) {_currentExpNum = _numOfExperiments;}
			else {_currentExpNum --;};
			modeRigSetLCD();
		};
	};
	
	// If next button is pushed.
	if (btnNext()) {
		// Check to see if it isn't the same press.
		if (!_btnNextPressed) {
			_btnNextPressed = true;
			
			// Set _currentExpNum.
			if (_currentExpNum == _numOfExperiments) {_currentExpNum = 1;}
			else {_currentExpNum ++;};
			modeRigSetLCD();
		};
	};
	
	// If go button is pushed.
	if (btnGo()) {
		// Check to see if it isn't the same press.
		if (!_btnGoPressed) {
			_btnGoPressed = true;
			
			// Run selected experiment
			modeRigExperiments();
		};
	};
	
	// Reset button pressed states.
	if (!btnPrevious())	{_btnPreviousPressed	= false;};
	if (!btnNext())		{_btnNextPressed		= false;};
	if (!btnGo())		{_btnGoPressed			= false;};
}

// AAL.modeRigSetLCD()

void AALClass::modeRigSetLCD () {
	// Buffers
	char LCDMessageBuffer[16] = {};
	char expNumBuffer[3] = {};
	
	// LCD print.
	itoa(_currentExpNum, expNumBuffer, 10);
	strcat(LCDMessageBuffer,"Experiment: ");
	strcat(LCDMessageBuffer,expNumBuffer);
	LCDPrint("Mode: Rig",LCDMessageBuffer);
}

// AAL.modeRigExperiments()

int AALClass::modeRigExperiments (int numRequested) {
	// Number of experiments.
	int numberOfExperiments = 9;
	
	// If an experiment has been requested.
	if (numRequested == 0) {
		switch (_currentExpNum) {
			case 1: // Experiment 1
				serialPrintln("Experiment 1 GO!");
				exp_step (5, 30, 3);
				break;
				
			case 2: // Experiment 2
				serialPrintln("Experiment 2 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.1, 0.1, 0.05);
				break;
				
			case 3: // Experiment 3
				serialPrintln("Experiment 3 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.2, 0.1, 0.05);
				break;
				
			case 4: // Experiment 4
				serialPrintln("Experiment 4 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.3, 0.1, 0.05);
				break;
				
			case 5: // Experiment 5
				serialPrintln("Experiment 5 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.4, 0.1, 0.05);
				break;
				
			case 6: // Experiment 6
				serialPrintln("Experiment 6 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.5, 0.1, 0.05);
				break;
				
			case 7: // Experiment 7
				serialPrintln("Experiment 7 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.6, 0.1, 0.05);
				break;
				
			case 8: // Experiment 8
				serialPrintln("Experiment 8 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.7, 0.1, 0.05);
				break;
				
			case 9: // Experiment 9
				serialPrintln("Experiment 9 GO!");
				exp_step_c(5, 30, 3.0, "pi", 0.5, 0.8, 0.1, 0.05);
				break;
				
			default:
				break;
		};
		
		// Return something.
		return numberOfExperiments;
	};
	
	// If number of experiments requested.
	if (numRequested != 0) {
		return numberOfExperiments;
	};
}

// ======================================================================
// =============================================== Rig Inputs and Outputs

// AAL.R1M()

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

// AAL.serialMessages()

void AALClass::serialMessages (int arg) { if ( arg==0||arg==1) {_serial = arg;}; }

// AAL.serialPrint()

void AALClass::serialPrint (char str[]) { if (_serial) {Serial.print(str);}; }

// AAL.serialPrintln()

void AALClass::serialPrintln (char str[]) { if (_serial) {Serial.println(str);}; }

// ======================================================================
// ================================================================== LCD

// AAL.LCDPrint()

void AALClass::LCDPrint (char row1[], char row2[]) {
	// Clear the LCD.
	LCD.clear();
	// Print row 1.
	LCD.print(row1);
	// Print row 2.
	LCD.setCursor(0, 1);
	LCD.print(row2);
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
	ACTLab.submitData(time,reference,input,output);
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
// ============================================ Control Assistive Methods

// AAL.cProperties_reset()

void AALClass::cProperties_reset () {
	_cm	= "";
	_Kp	= 0;
	_Ki	= 0;
	_Kd	= 0;
	_u	= 0;
	_u1	= 0;
	_y	= 0;
	_y1	= 0;
	_e	= 0;
	_e1	= 0;
	_d	= 0;
	_d1 = 0;
	_tLoopStart	= 0;
	_tLoop		= 0;
}

// AAL.cProperties_set()

void AALClass::cProperties_set (String cm, float Kp, float Ki, float Kd, float d) {
	_cm = cm;
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_d	= d;
	_d1	= d;
}

// AAL.cProperties_pushYE()

void AALClass::cProperties_pushYE (float y, float e) {
	_y1	= _y;
	_e1	= _e;
	_y	= y;
	_e	= e;
}

// AAL.cProperties_pushU()

void AALClass::cProperties_pushU (float u) {
	_u1	= _u;
	_u	= u;
}

// AAL.cLoop_start()

void AALClass::cLoop_start () {
	// Get time reference at the start of loop.
	_tLoopStart = millis()/((float)1000);
}

// AAL.cLoop_end()

void AALClass::cLoop_end () {
	// Get time since start of loop.
	_tLoop = (millis()/((float)1000)) - _tLoopStart;
	
	// Check loop length and act accordingly.
	if (_tLoop < _d) {
		delay((_d - _tLoop)*1000);
		_d1 = _d;
	} else {
		_d1 = _tLoop;
	};
}

// ======================================================================
// ====================================================== Control Methods

// AAL.c_p()

double	AALClass::c_p () {
	return (_Kp*_e);
}

// AAL.c_pd()

double	AALClass::c_pd () {
	return ((_Kp*_e)+((_Kd/_d1)*(_e - _e1)));
}

// AAL.c_pi()

double	AALClass::c_pi () {
	return (_u1+(_Kp*_e)-((_Kp-(_Ki*_d1))*_e1));
}

// AAL.c_pdfb()

double	AALClass::c_pdfb () {
	return ((_Kp*_e)-((_Kp*_Kd)*((_y-_y1)/_d1)));
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
	float _Kp = Kp;			// 0.40
	float _delta = delta;	// 0.05 s
	float _tLoopStart;
	float _tLoop;
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	// Control loop.
	for (float time = 0; time <= end; time = time + _delta) {
		
		// Get time reference at the start of loop.
		_tLoopStart = millis()/((float)1000);
		
		// Determine error, e(k).
		//float e_k_ = (ref_step(time,start,end,amplitude)) - R1T();
		
		// Send input to rig.
		//R1M((_Kp*e_k_));
		
		// Submit data.
		//submit (time, (_Kp*e_k_), (_Kp*e_k_), R1T());
		
		// Get time since start of loop.
		_tLoop = (millis()/((float)1000)) - _tLoopStart;
		
		// Check loop length and act accordingly.
		if (_tLoop < delta) {
			delay((delta - _tLoop)*1000);
			_delta = delta;
		} else {
			_delta = _tLoop;
		};
	};
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	return true;
}

// AAL.exp_step_c()

bool AALClass::exp_step_c (double start, double end, double amplitude, String cm, double Kp, double Ki, double Kd, double delta) {
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	// Reset control properties.
	cProperties_reset();
	
	// Set control properties.
	cProperties_set(cm, Kp, Ki, Kd, delta);
	
	// Control loop.
	for (float time = 0; time <= end; time = time + _d1) {
		// Start loop actions.
		cLoop_start();
		
		// Variables.
		float	r;
		float	u;
		float	y;
		float	e;
		
		// Get reference.
		r = ref_step(time,start,end,amplitude);
		// Get output.
		y = R1T();
		// Calculate error.
		e = r - y;
		// Push y and e.
		cProperties_pushYE(y, e);
		// Calculate input.
		if		(cm == "p")		{u = c_p();}
		else if	(cm == "pd")	{u = c_pd();}
		else if	(cm == "pi")	{u = c_pi();}
		else if	(cm == "pdfb")	{u = c_pdfb();}
		else	{return false;};
		// Push u.
		cProperties_pushU(u);
		// Send u to rig.
		R1M(u);
		
		// Submit data.
		submit (time, r, u, y);
		
		// End loop actions.
		cLoop_end();
	};
	
	// Settle the rig.
	R1M(0);R2M(0);delay(5000);
	
	return true;
}

// ======================================================================

// Initialize an AAL object.

AALClass AAL;
// If statement to make sure the AAL Library is not
// included more than once.

#ifndef AAL_h
#define AAL_h

// Include the Arduino Library - to access the standard
// types and constants of the Arduino language.

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Ethernet.h>	
#include <ACTLab.h>

// Declare (and spec out) the AALClass class.
class AALClass {
	// Public methods and properties.
	public:
		// Constructor method.
		AALClass();
		
	// Public methods.
		// AAL Setup:
		void	setup();
		// Rig inputs and outputs:
		bool	R1M(double voltage);	// true = value given and safe; false = value not given as unsafe.
		bool	R2M(double voltage);	// true = value given and safe; false = value not given as unsafe.
		double	R1T();
		double	R2T();
		// Buttons:
		bool	btnInternet();
		bool	btnRig();
		bool	btnPrevious();
		bool	btnNext();
		bool	btnGo();
		// Serial messages:
		void	serial(int arg);
		void	serialPrint(char str[]);
		void	serialPrintln(char str[]);
		// LCD:
		void	LCDPrint(char str[]);		// !!!!!
		// Data submissions:
		void	submit(double time, double input, double output);
		void	submitSerialSet(int arg);
		void	submitServerSet(int arg);
		void	submitSerial(double time, double input, double output);
		void	submitServer(double time, double input, double output);
		// Experiment methods.
			
		
		// Public properties.
	
	// Private methods and properties.
	private:
		// Private methods.
		
		// Private properties.
		// Calibaration:
		int		_analogInputUpperBound;
		int		_PWMOutputUpperBound;
		// Voltage safety limits:
		float	_motorVoltageCutOff;
		// Rig connections:
		int		_rotor1MotorPWMPin;
		int		_rotor2MotorPWMPin;
		int		_rotor1TachoAnalogPin;
		int		_rotor2TachoAnalogPin;
		// Button pins:
		int		_btnInternetPin;
		int		_btnRigPin;
		int		_btnPreviousPin;
		int		_btnNextPin;
		int		_btnGoPin;
		// Data submissions:
		int		_submitSerial;	// 0 = OFF; 1 = ON; Default - 0.
		int		_submitServer;	// 0 = OFF; 1 = ON; Default - 0.
		// Others:
		int		_baud;			// Default - 9600.
		int		_serial;		// 0 = OFF; 1 = ON; Default - 0.
};

// Code to remove the need to initialize an AAL
// object in sketch.

extern ACTLabClass ACTLab;
extern AALClass AAL;

#endif
// If statement to make sure the AAL Library is not
// included more than once.

#ifndef AAL_h
#define AAL_h

// Include the Arduino Library - to access the standard
// types and constants of the Arduino language. Also include
// any other libraries needed by this one.

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
		bool	R1M(double voltage);	// true = value given and safe;
										// false = value not given as unsafe.
		bool	R2M(double voltage);	// true = value given and safe;
										// false = value not given as unsafe.
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
		void	submit(double time, double reference, double input, double output);
		void	submitSerialSet(int arg);
		void	submitServerSet(int arg);
		void	submitSerial(double time, double reference, double input, double output);
		void	submitServer(double time, double reference, double input, double output);        // !!!!!
		// Reference Signals:
		double	ref_step(double time, double start, double end, double amplitude);
		double	ref_sine(double time, double start, double end, double amplitude, double frequency);
		double	ref_scale(double time, double start, double end, double bias,
							double amplitude, double frequencyStart, double frequencyEnd);
		// Experiment methods:
		bool	exp_step(double start, double end, double amplitude);
		bool	exp_scale(double start, double end, double bias, double amplitude,
							double frequencyStart, double frequencyEnd);
		bool	exp_p_step(double start, double end, double amplitude, double Kp, double delta);
		
		// Public properties.
	
	// Private methods and properties.
	private:
		// Private methods.
		
		// Private properties.
		// Calibaration:
		double	_analogInputUpperBound;
		double	_PWMOutputUpperBound;
		// Voltage safety limits:
		double	_motorVoltageCutOff;
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
		int		_submitSerial;	// 0 = OFF; 1 = ON; Default - 1.
		int		_submitServer;	// 0 = OFF; 1 = ON; Default - 1.
		// Others:
		int		_baud;			// Default - 9600.
		int		_serial;		// 0 = OFF; 1 = ON; Default - 0.
};

// When calling the ACTLab and AAL objects, look outside this file.

extern ACTLabClass ACTLab;
extern AALClass AAL;

#endif
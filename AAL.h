// If statement to make sure the AAL Library is not
// included more than once.

#ifndef AAL_h
#define AAL_h

// Include the Arduino Library - to access the standard
// types and constants of the Arduino language. Also include
// any other libraries needed by this one.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Ethernet.h>
#include <LiquidCrystal.h>
#include <ACTLab.h>

// Declare (and spec out) the AALClass class.
class AALClass {
	// Public methods and properties.
	public:
		// Constructor method.
		AALClass();
		
		// Public methods.
		// AAL start and other core methods:
		void	setup();
		void	start(int setupRequested = 1);
		void	rigLoop();
		void	modeInternet();
		void	modeRig();
		void	modeRigSetLCD();
		int		modeRigExperiments(int numRequested = 0);
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
		void	serialMessages(int arg);
		void	serialPrint(char str[]);
		void	serialPrintln(char str[]);
		// LCD:
		void	LCDPrint(char row1[], char row2[]);
		// Data submissions:
		void	submit(double time, double reference, double input, double output);
		void	submitSerialSet(int arg);
		void	submitServerSet(int arg);
		void	submitSerial(double time, double reference, double input, double output);
		void	submitServer(double time, double reference, double input, double output);        // !!!!!
		// Reference signals:
		double	ref_step(double time, double start, double end, double amplitude);
		double	ref_sine(double time, double start, double end, double amplitude, double frequency);
		double	ref_scale(double time, double start, double end, double bias,
						  double amplitude, double frequencyStart, double frequencyEnd);
		// Control assistive methods:
		void	cProperties_reset();
		void	cProperties_set(String cm, float Kp, float Ki, float Kd, float d);
		void	cProperties_pushYE(float y, float e);
		void	cProperties_pushU(float u);
		void	cLoop_start();
		void	cLoop_end();
		// Control methods:
		double	c_p ();
		double	c_pd ();
		double	c_pi ();
		double	c_pdfb ();
		// Experiment methods:
		bool	exp_step(double start, double end, double amplitude);
		bool	exp_scale(double start, double end, double bias, double amplitude,
						  double frequencyStart, double frequencyEnd);
		bool	exp_p_step(double start, double end, double amplitude, double Kp, double delta);
		bool	exp_step_c(double start, double end, double amplitude, String cm, double Kp, double Ki, double Kd, double delta);
		
		// Public properties.
	
	// Private methods and properties.
	private:
		// Private methods.
		
		// Private properties.
	// Core method properties:
		bool	_LCD_initialModeSet;
		bool	_LCD_modeInternet;
		int		_numOfExperiments;
		int		_currentExpNum;
		bool	_btnPreviousPressed;
		bool	_btnNextPressed;
		bool	_btnGoPressed;
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
		int		_submitServer;	// 0 = OFF; 1 = ON; Default - 0.
		// Others:
		int		_baud;			// Default - 9600.
		int		_serial;		// 0 = OFF; 1 = ON; Default - 0.
		// Control properties:
		String	_cm;
		float	_Kp;
		float	_Ki;
		float	_Kd;
		float	_u;
		float	_u1;
		float	_y;
		float	_y1;
		float	_e;
		float	_e1;
		float	_d;
		float	_d1;
		float	_tLoopStart;
		float	_tLoop;
};

// When calling the ACTLab and AAL objects, look outside this file.

extern ACTLabClass ACTLab;
extern AALClass AAL;

#endif
#ifndef _RCR_PID_HH_
#define _RCR_PID_HH_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "globals.hh"

class Pid {
 public:
	Pid(volatile int*, int*, int*, float, float, float, float, int, int);
	void Compute();

 private:
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter
	float kd;                  // * (D)erivative Tuning Parameter
	float n;

	volatile int *myInput; 
	int *myOutput;  
	int *mySetpoint; 

	float ITerm, lastError, lastDTerm=0;
	unsigned long lastTime;
	int outMin, outMax;
};

#endif // _RCR_PID_HH_

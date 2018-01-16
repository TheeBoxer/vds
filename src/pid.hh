#ifndef _RCR_PID_HH_
#define _RCR_PID_HH_

namespace rcr {
namespace vds {

class Pid {
 public:
	Pid(volatile int*, int*, int*, float, float, float, float, int, int);
	void Compute();

 private:
  // Proportional Tuning Parameter
	float kp;

  // Integral Tuning Parameter
  float ki;

  // Derivative Tuning Parameter
  float kd;

	float n;

	volatile int *myInput; 
	int *myOutput;  
	int *mySetpoint; 

	float ITerm, lastError, lastDTerm=0;
	unsigned long lastTime;
	int outMin, outMax;
};

} // namespace vds
} // namespace rcr

#endif // _RCR_PID_HH_

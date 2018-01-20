#include "daq.hh"

#include "Adafruit_BMP280.h"

namespace rcr {
namespace vds {

void DaqController::init(bool bnoToo) {}

/**************************************************************************/
/*!
@brief  Gathers data from the desired source (Sensors or file). Dependent on 
TEST_MODE
Author: Jacob & Ben
*/
/**************************************************************************/
bool DaqController::getRawState(VehicleState* rawState, bool testMode) {
	bool returnVal;
	if (testMode) {                                                  //If file is in test mode, retrieve sensor data from data file with past flight data
		if (!flight_log.readCSV(rawState)) {
			out << "end of flight\n";
			returnVal = false;
		}
		else {
			returnVal = true;
		}
	}
	else {
		rawState->alt = bmp280.readAltitude(SEALVL_PRESS) - padAlt;
		if (timeOverflow) {
			rawState->time = millis() * 100;
		}
		else {
			rawState->time = micros()/10;
		}

		if (rawState->time > 420000000) {
			timeOverflow = true;
		}

		rawState->accel = get_vertical_accel();
		returnVal = true;
	}

	rawState->vel = calculateVelocity(*rawState);
	return returnVal;
}

float DaqController::get_vertical_accel() {
	auto gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  auto linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  double
    g_x = 0.,
    g_y = 0.,
    g_z = 0.,
    l_x = 0.,
    l_y = 0.,
    l_z = 0.;

	g_x = gravity.x();
  g_y = gravity.y();
	g_z = gravity.z();

	l_x = linear.x();
	l_y = linear.y();
	l_z = linear.z();

  auto& state = flight_log.supStat;
	state.rollAxisGrav = g_x;
	state.yawAxisGrav = g_y;
	state.pitchAxisGrav = g_z;

	state.rollAxisLin = l_x;
	state.yawAxisLin = l_y;
  state.pitchAxisLin = l_z;

	float res = linear.dot(gravity) / 9.81;
	return res;
}

/**************************************************************************/
/*!
@brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
Author: Jacob & Ben
- Algorithm developed by Ben Stringer, function written by Jacob Cassady
*/
/**************************************************************************/
float DaqController::calculateVelocity(VehicleState rawState) {
	float lhs = 0, sumTimes = 0, sumTimes2 = 0, sumAlt = 0, sumAltTimes = 0;
	float rhs = 0, numer = 0, denom = 0, velocity = 0, pastTime = 0, newTime = 0;

	// shift new readings into arrays   
	for (int i = BUFF_N; i > 0; i--) {
    copyState(&pastRawStates[i], &pastRawStates[i - 1]);
	}
	rawState.buff_t = 0;
	copyState(&pastRawStates[0], &rawState);

	// time relative to the current moment
	for (int i = BUFF_N; i > 0; i--) {
		pastTime = (float)pastRawStates[i - 1].time;
		newTime = (float)rawState.time;
		pastRawStates[i - 1].buff_t = (pastTime - newTime) / (float)TIME_DIVISOR;   //Calculates buff_t values for pastRawStates array (sec)
	}

	// calculate LHS sums for vel equation
	for (unsigned int i = 0; i < BUFF_N; i++) {
		sumTimes += (float)(pastRawStates[i].buff_t);
		sumTimes2 += (float)((pastRawStates[i].buff_t) * (pastRawStates[i].buff_t));
		sumAlt += pastRawStates[i].alt;
		sumAltTimes += ((float)pastRawStates[i].buff_t * pastRawStates[i].alt);
	}

	// calculate LHS
	numer = ((sumTimes * sumAlt) - (BUFF_N * sumAltTimes));
	denom = ((sumTimes*sumTimes) - (BUFF_N * sumTimes2));
	lhs = numer / denom;

	// calculate RHS
	for (unsigned int i = 0; i <= (BUFF_N / 2); i++) {
		rhs += 0.5 * (pastRawStates[i].accel + pastRawStates[i + 1].accel) * (pastRawStates[i].buff_t - pastRawStates[i + 1].buff_t);
	}

	flight_log.supStat.leftVel = lhs;
  flight_log.supStat.rightVel = rhs;

	velocity = lhs + rhs;

	if (isnan(velocity)) {
		flight_log.logError(NAN_VEL);
		velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
	}
	if ((velocity > MAX_EXP_VEL) || (velocity < MIN_EXP_VEL)) {           //logs error if velocity value is egregiously too high or low.
		flight_log.logError(NONNOM_VEL);
		velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
	}
	return velocity;
}

/**************************************************************************/
/*!
@brief  sets the altitude of the launch pad
Author: Ben
*/
/**************************************************************************/
void DaqController::setPadAlt() {
	padAlt = bmp280.readAltitude(SEALVL_PRESS);
}

} // namespace vds
} // namespace rcr

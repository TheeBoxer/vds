#include "rcr_classes.hh"

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void DaqController::init(bool bnoToo) {}

/**************************************************************************/
/*!
@brief  Gathers data from the desired source (Sensors or file). Dependent on 
TEST_MODE
Author: Jacob & Ben
*/
/**************************************************************************/
bool DaqController::getRawState(struct stateStruct* rawState, bool testMode) {
	bool returnVal;
	if (testMode) {                                                  //If file is in test mode, retrieve sensor data from data file with past flight data
		if (!log.readCSV(rawState)) {
			Serial.println("end of flight");
			delay(1000);
			returnVal = false;
		}
		else {
			delay(MOTORTEST_DELAY_MS);
			returnVal = true;
		}
	}
	else {
		//get raw altitude
#if !BMP280
		rawState->alt = altitude_plz() - padAlt;
#else
		rawState->alt = bmp280.readAltitude(SEALVL_PRESS) - padAlt;
#endif
		if (timeOverflow) {
			rawState->time = millis() * 100;             //Retrieves time from millis() function, stores within rawState
		}
		else {
			rawState->time = micros()/10;
		}

		if (rawState->time > 420000000) {
			timeOverflow = true;
		}

		//get raw acceleration  
		rawState->accel = get_vertical_accel();                          //Retrieves acceleration from bno055 sensor, stores within rawState
		returnVal = true;
	}

	rawState->vel = calculateVelocity(*rawState);                 //Calculates velocity using algorithm.  Takes prior acceleration and velocity values from pastRawStates

#if DEBUG_RAWSTATE
	Serial.println();
	Serial.println("RAW STATE--------------------");
	GUI.printState(*rawState, "raw state");                            //If in DEBUG_RAWSTATE mode, prints raw state data for evaluation.
#endif
	return returnVal;
} // END getRawState()

float DaqController::get_vertical_accel() {
	imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
	decltype(gravity.x()) g_x = 0, g_y = 0, g_z = 0, l_x = 0, l_y = 0, l_z = 0;

	g_x = gravity.x();
  g_y = gravity.y();
	g_z = gravity.z();

	l_x = linear.x();
	l_y = linear.y();
	l_z = linear.z();

	float result = linear.dot(gravity) / 9.81;

	log.supStat.rollAxisGrav = g_x;
	log.supStat.yawAxisGrav = g_y;
	log.supStat.pitchAxisGrav = g_z;

	log.supStat.rollAxisLin = l_x;
	log.supStat.yawAxisLin = l_y;
	log.supStat.pitchAxisLin = l_z;

	return result;
}

// "for testing purposes"
float DaqController::get_vertical_accel(imu::Vector<3> gravity, imu::Vector<3> linear) {
	float linearDotGravity = 0, theta = 0, defOfProduct = 0,  verticalAcceleration = 0, magL = 0, magG = 0;
	float xG = 0, yG = 0, zG = 0, xL = 0, yL = 0, zL = 0;

	xG = gravity.x();
	yG = gravity.y();
	zG = gravity.z();

	xL = linear.x();
	yL = linear.y();
	zL = linear.z();

	linearDotGravity = (xG*xL) + (yG*yL) + (zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors

	magL = pow(((xL*xL) + (yL*yL) + (zL*zL)), 0.5);                                      //Calculates magnitude of linear acceleration vector.
	magG = pow(((xG*xG) + (yG*yG) + (zG*zG)), 0.5);                                      //Calculates magnitude of acceleration from gravity vector.

	defOfProduct = linearDotGravity / (magL*magG);                                  //Calculates the cosine value using the definition of a dot product.

	theta = acos(defOfProduct);                                                     //Calculates theta using the arc cosine of the previously calculated vosine value.
	theta = (theta * 180) / PI;                                                         //Converts theta from radians to degress.

	verticalAcceleration = linearDotGravity / magG;                                 //Finds the acceleration in the direction of gravity.

																					/* Display the used acceleration from gravity*/
	Serial.print("Gravity Used <x,y,z>: ");
	Serial.print("<");
	Serial.print(xG);                                //
	Serial.print(",");
	Serial.print(yG);                                //
	Serial.print(",");
	Serial.print(zG);                                //
	Serial.println(">;");

	/* Display the calculated theta*/
	Serial.print("Theta: ");
	Serial.println(theta);

	/* Display the calculated vertical acceleration*/
	Serial.print("verticalAcceleration: ");
	Serial.print(verticalAcceleration);

	Serial.println("");

	return verticalAcceleration;                                                    //Returns calculated vertical acceleration.
} // END get_vertical_accel();  OVERLOADED VERSION


/**************************************************************************/
/*!
@brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
Author: Jacob & Ben
- Algorithm developed by Ben Stringer, function written by Jacob Cassady
*/
/**************************************************************************/
float DaqController::calculateVelocity(struct stateStruct rawState) {
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

	log.supStat.leftVel = lhs;
  log.supStat.rightVel = rhs;

	velocity = lhs + rhs;

	if isnan(velocity) {
		log.logError(NAN_VEL);
		velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
	}
	if ((velocity > MAX_EXP_VEL) || (velocity < MIN_EXP_VEL)) {           //logs error if velocity value is egregiously too high or low.
		log.logError(NONNOM_VEL);
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
void DaqController::setPadAlt(void) {
	padAlt = bmp280.readAltitude(SEALVL_PRESS);
}

DaqController daq_controller;

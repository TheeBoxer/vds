#include "drag_inducers.hh"

#include "globals.hh"
#include "limit_switches.hh"
#include "../rcr_maths.h"
#include "rcr_util.hh"

#include <chrono>
#include <cmath>
#include <cstdint>

namespace rcr {
namespace vds {

void DragInducers::init() {
	//setup motor pins
	pinMode(rcr::vds::digital_io::Pin::MOTOR_A, OUTPUT);
	pinMode(rcr::vds::digital_io::Pin::MOTOR_B, OUTPUT);
	pinMode(rcr::vds::digital_io::Pin::MOTOR_PWM, OUTPUT);
	//setup encoder pins
	pinMode(rcr::vds::digital_io::Pin::ENC_A, INPUT);
	pinMode(rcr::vds::digital_io::Pin::ENC_B, INPUT);
	//setup limit switch pins
	pinMode(rcr::vds::digital_io::Pin::LIM_OUT, INPUT);
	pinMode(rcr::vds::digital_io::Pin::LIM_IN, INPUT);
}

/**************************************************************************/
/*!
@brief  Prints status info on the limit switches, encoder position, etc
Author: Ben
*/
/**************************************************************************/
void DragInducers::dragBladesCheck() {
	out << "\r\n-----Drag Blades Check----\n";	
  out << "encMin: ";
  Serial.println(encMin);
  out << "encMax: ";
  Serial.println(encMax);
  out << "encPos: ";
  Serial.println(encPos);
  out << "Inner limit pressed : ";
  Serial.println(!digitalRead(rcr::vds::digital_io::Pin::LIM_IN));
  out << "Outter limit pressed : ";
  Serial.println(!digitalRead(rcr::vds::digital_io::Pin::LIM_OUT));
}

/**************************************************************************/
/*!
@brief  Returns the encoder position that the airbrdakes should deploy based on how
fast the vehicle is moving and how fast the SPP thinks it should be moving
Author: Ben
*/
/**************************************************************************/
int DragInducers::airBrakesGoToEncPos(float vehVel, float sppVel)
{
	float returnVal;
	returnVal = -(sppVel - vehVel) * AIRBRAKES_GAIN;
	if (returnVal >= 100) returnVal = 100;
	else if (returnVal <= 0) returnVal = 0;
	return (int)rcr::maths::map((long)returnVal, 0, 100, encMin, encMax);
}

/**************************************************************************/
/*!
@brief  Makes the DC motor spin with a given speed and direction
Also ensures that the drag blades don't extend past their limits.
Author: Ben
*/
/**************************************************************************/
void DragInducers::motorDo(BladeDirection direction, uint8_t speed) {
  bool fully_closed = m_.send(blades::limit_switches::IsInnerSwitchEnabled{}); 
  bool fully_open = m_.send(blades::limit_switches::IsOuterSwitchEnabled{});

	if (direction == BladeDirection::Out) {
		digitalWrite(rcr::vds::digital_io::Pin::MOTOR_A, kHigh);
		digitalWrite(rcr::vds::digital_io::Pin::MOTOR_B, kLow);
	}
	else {
		digitalWrite(rcr::vds::digital_io::Pin::MOTOR_A, kLow);
		digitalWrite(rcr::vds::digital_io::Pin::MOTOR_B, kHigh);
	}
	
	//flight_log.supStat.encPos = encPos;
	//flight_log.supStat.encPosCmd = encPosCmd;
	//flight_log.supStat.limit_in = fully_closed;
	//flight_log.supStat.limit_out = fully_open;
	//flight_log.supStat.encMax = encMax;
	//flight_log.supStat.encMin = encMin;
	//flight_log.supStat.mtrSpdCmd = mtrSpdCmd;

	if (!fully_closed && (direction == BladeDirection::In)) {
		analogWrite(rcr::vds::digital_io::Pin::MOTOR_PWM, 0);
		int range = std::abs(encMax - encMin);
		encPos = 0;	
		encMin = 0;
		encMax = range;
	}
	else if (!fully_open && (direction == BladeDirection::Out)) {
		analogWrite(rcr::vds::digital_io::Pin::MOTOR_PWM, 0);
		int range = std::abs(encMax - encMin);
		encPos = range;
		encMin = 0;
		encMax = range;
	}
	else {
		analogWrite(rcr::vds::digital_io::Pin::MOTOR_PWM, speed);
	}
	if ((encMax - encMin) < 9 * ENC_RANGE / 10) {
		encMin = 0;
		encMax = ENC_RANGE;
		flight_log.logError(ENC_RANGE_ERROR);
	}
}

void DragInducers::motorDont() {
	digitalWrite(rcr::vds::digital_io::Pin::MOTOR_A, kLow);
	digitalWrite(rcr::vds::digital_io::Pin::MOTOR_B, kLow);
	analogWrite(rcr::vds::digital_io::Pin::MOTOR_PWM, 0);
}

/**************************************************************************/
/*!
@brief  This function returns whether or not the dragblades are at the given
encoder position. If false, motorGoTo() calculates the speed and direction required
for the dragblades to reach their target. To return true, the blades must be at 
their target for at least SETPOINT_INAROW succesive calls of motorGoTo()
Author: Ben
*/
/**************************************************************************/
bool DragInducers::motorGoTo(int goTo)
{
	static uint8_t count = 0;
	encPosCmd = goTo;
	motorPID.Compute();
	if (mtrSpdCmd >= 0) {
		drag_inducers.motorDo(OUTWARD, mtrSpdCmd);
	}
	else if (mtrSpdCmd < 0) {
		drag_inducers.motorDo(INWARD, -1 * mtrSpdCmd);
	}
	if (std::abs(drag_inducers.encPos - encPosCmd) <= SETPOINT_TOLERANCE) {
		count++;
	}
	else {
		count = 0;
	}
	if (count >= SETPOINT_INAROW) {
		count = 0;		
		return true;
	}
	else {
		return false;
	}
}

/**************************************************************************/
/*!
@brief  First calibrates the drag blades by finding the limit switches. 
When a limit switch is hit, remember what encoder position it was hit at. 
Then exercise motorGoTo by extending and retracting the blades in 1/4 turns
Author: Ben
*/
/**************************************************************************/
void DragInducers::motorTest() {
	int timer = 0;
	flight_log.sd.remove(MOTOR_FILENAME);                                 //Removes prior error file

	File data = flight_log.sd.open(MOTOR_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		out << "Data file unable to initiated - motorTest\n";
		disk_initialized = false;
		return;
	}
	else {                                             //Adds unique header depending on if VDS is in test or flight mode
		data.println("times, encPos, encPosCmd, fully_open, fully_closed, encMax, encMin, mtrSpdCmd");
		data.close();                                               //Closes data file after use.
	}
	timer = millis();
	while (digitalRead(rcr::vds::digital_io::Pin::LIM_OUT)) {
		motorDo(OUTWARD, DEADZONE_MAX);
		if ((Serial.available() > 0) || ((millis() - timer) >4000)) {
			motorDont();
			return;
		}
	}
	timer = millis();
	while (digitalRead(rcr::vds::digital_io::Pin::LIM_IN)) {
		motorDo(INWARD, DEADZONE_MAX);
		if ((Serial.available() > 0) || ((millis() - timer) >4000)) {
			motorDont();
			return;
		}
	}	
	drag_inducers_initialized = true;

  using namespace std::chrono_literals;
  using rcr::util::sleep_for;
  auto delay = 300ms;

	motorGoToPersistent(0);
	motorDont();
  sleep_for(delay);

  for (uint16_t percent = 0; percent < 100;) {
    motorGoToPersistent(percent += 25);
    motorGoToPersistent(percent);
    motorDont();
    sleep_for(delay);
  }



	motorGoToPersistent(25);
	motorGoToPersistent(50);
	motorDont();
  sleep_for(delay);
	motorGoToPersistent(50);
	motorGoToPersistent(75);
	motorDont();
  sleep_for(delay);
	motorGoToPersistent(75);
	motorGoToPersistent(100);
	motorDont();
  sleep_for(delay);

	motorGoToPersistent(100);
	motorGoToPersistent(75);
	motorDont();
  sleep_for(delay);

	motorGoToPersistent(75);
	motorGoToPersistent(50);
	motorDont();
  sleep_for(delay);

	motorGoToPersistent(50);
	motorGoToPersistent(25);
	motorDont();
  sleep_for(delay);

	motorGoToPersistent(25);
	motorGoToPersistent(0);
	motorDont();
	motorGoToPersistent(0);
	motorDont();
}

/**************************************************************************/
/*!
@brief  This is a quick exercise meant to spin the motor in different ways.
Data collected from this function was used to build a transfer function for the DC motor
WARNING: Do not execute this function if the motor is installed in the VDS as it will try
to spin past its physical limits and the motor will stall.
Author: Ben
*/
/**************************************************************************/
void DragInducers::motorExercise()
{
	int deadZoneSpeed = 63;
	unsigned long t = 0;
	unsigned long t0 = 0;
	bool dir = OUTWARD;
	float derp;
	uint8_t spd = 0;
	flight_log.sd.remove("motorExercise.dat");
	File myFile = flight_log.sd.open("motorExercise.dat", FILE_WRITE);
	myFile.println("times,spd,dir");
	myFile.close();

	t0 = micros();
	while (t<8000000) {
		myFile = flight_log.sd.open("motorExercise.dat", FILE_WRITE);
		t = micros() - t0;
		if (t < 1000000) {
			dir = OUTWARD;
			spd = 255;
		}
		else if (t < 2000000) {
			dir = OUTWARD;
			derp = (float)(t - 1000000) / 1000000;
			spd = derp * 255;
			//out << "derp = ";
			//Serial.println(derp);
		}
		else if (t < 3000000) {
			dir = OUTWARD;
			spd = 0;
		}
		else if (t < 4000000) {
			dir = OUTWARD;
			spd = deadZoneSpeed;
		}
		else if (t < 5000000) {
			dir = OUTWARD;
			derp = (float)(t - 4000000) / 1000000;
			spd = deadZoneSpeed + derp * (255 - deadZoneSpeed);
			//out << "derp = ";
			//Serial.println(derp);
		}
		else if (t < 6000000) {
			dir = INWARD;
			spd = 255;
		}
		else if (t < 7000000) {
			dir = OUTWARD;
			spd = 0;
		}
		//Serial.print(t);
		//out << ",\t";
		//Serial.println(spd);
		drag_inducers.motorDo(dir, spd);
		myFile.printf("%lu,%u,%d,%d", t, spd, dir, drag_inducers.encPos);
		myFile.println("");
		myFile.close();
	}
}

/**************************************************************************/
/*!
@brief  Moves the blades in and out until a serial command is recieved
Author: Ben
*/
/**************************************************************************/
void DragInducers::powerTest(util::IoStreams& s) {
  using namespace std::chrono_literals;
  const auto half_sec = 500ms;

	while (s.in.gcount() == 0) {
		while (!motorGoTo(encMin)) {
      rcr::util::sleep_for(kMotorTestDelay);
		}
		while (!motorGoTo(encMax)) {
      rcr::util::sleep_for(kMotorTestDelay);
		}
	}

  rcr::util::clear_input(s);
	while (!Serial.available()) {
		while ( !motorGoTo(encMin)) {
      rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
		while (!motorGoTo(rcr::maths::map(33, 0, 100, encMin, encMax))) {
      rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
		while (!motorGoTo(rcr::maths::map(66, 0, 100, encMin, encMax))) {
			rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
		while ( !motorGoTo(encMax)) {
			rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
		while (!motorGoTo(rcr::maths::map(66, 0, 100, encMin, encMax))) {
			rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
		while (!motorGoTo(rcr::maths::map(33, 0, 100, encMin, encMax))) {
			rcr::util::sleep_for(kMotorTestDelay);
		}
		motorDont();
		rcr::util::sleep_for(half_sec);
	}
}

/**************************************************************************/
/*!
@brief  A function that will not return until the motor reaches its destination
Also takes an argument as a percent and not as an encoder position.
Also records data on the sd card.
Author: Ben
*/
/**************************************************************************/
void DragInducers::motorGoToPersistent(int goToPercent) {
	while (!motorGoTo(rcr::maths::map(goToPercent, 0, 100, encMin, encMax))) {
		rcr::util::sleep_for(kMotorTestDelay);
	}
}

DragInducers drag_inducers{};

} // namespace vds
} // namespace rcr

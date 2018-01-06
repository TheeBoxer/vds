#include "Adafruit_BMP280.h"
#include "daq.hh"
#include "drag_inducers.hh"
#include "flight_log.hh"
#include "globals.hh"
#include "gui.hh"
#include "matrix.hh"
#include "pid.hh"
#include "rcr_util.hh"
#include "vds_maths.hh"

#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

namespace rcr {
namespace vds {

namespace {
  const bool* const must_be_initialized[] { &bmp_initialized, &bno_initialized, &drag_inducers_initialized, &disk_initialized };

  void d() { drag_inducers.dragBladesCheck(); }
  void f(bool &test_mode, bool &fullBrakesTest, bool &retflag) {
    gui.flush_input();
    Serial.println("------Choose Flight Mode Settings-----");

    Serial.println("test mode?");
    {
      auto auth = rcr::util::get_authorization(Serial);
      flight_log.newFlight(auth);
      test_mode = auth;
    }

    Serial.println("full-brake test mode?");
    fullBrakesTest = rcr::util::get_authorization(Serial);

    if (!test_mode) {
      for (auto& initialized : must_be_initialized) {
        if (!*initialized && !test_mode) {
          Serial.println("Cannot enter flight mode. A sensor or sd card is not initialized.");
          flight_log.logError(SENSOR_UNIT);
          return;
        }
      }
    }

    Serial.println("entering flight mode");
    Serial.print("Test Mode: ");
    Serial.println(test_mode ? "ON" : "OFF");
    Serial.print("Full-brakes Test: ");
    Serial.println(fullBrakesTest ? "ON" : "OFF");

    if (!test_mode) daq_controller.setPadAlt();
    flightMode(test_mode, fullBrakesTest);
  }
  void i() {
    Serial.println("Inching Inward");
    drag_inducers.motorDo(INWARD, DEADZONE_MAX + 15);
    delay(250);
    drag_inducers.motorDont();
  }
  void m() {
    gui.flush_input();
    Serial.println("\n\n----- Calibrate Motor -----;");
    drag_inducers.motorTest();
  }
  void o() {
    Serial.println("Inching Outward");
    drag_inducers.motorDo(OUTWARD, DEADZONE_MAX + 15);
    delay(250);
    drag_inducers.motorDont();
  }
  void p() {
    Serial.println("Power test");
    gui.flush_input();
    drag_inducers.powerTest();
    drag_inducers.motorDont();
  }
  void r() {
    gui.flush_input();
    gui.rocketMenu();
  }
  void s() {
    flight_log.init();
    daq_controller.init(false);
    gui.init();
    drag_inducers.dragBladesCheck();
  }
  void switch_default() {
    Serial.println("Unkown code received - main menu");
    flight_log.logError(INVALID_MENU);
  }

  /**************************************************************************/
  /*!
  @brief  Launch and test sequence.
  Author: Jacob & Ben
  */
  /**************************************************************************/
  void flightMode(bool testMode, bool fullBrakesTest) {
    VehicleState rawState, filteredState;
    int airBrakesEncPos_val = 0;
    float vSPP_val = 0;
    Serial.println("asdfasdf");
    gui.flush_input();
    while ((Serial.available() == 0) && daq_controller.getRawState(&rawState, testMode)) {
      vSPP_val = rcr::vds::maths::vSPP(rawState.alt, rawState.vel);
      airBrakesEncPos_val = drag_inducers.airBrakesGoToEncPos(rawState.vel, vSPP_val);
      if (!fullBrakesTest) {
        drag_inducers.motorGoTo(airBrakesEncPos_val);
      }
      else {
        if ((rawState.accel < 0) && (rawState.alt > 150) && (rawState.vel > 0)) {
          drag_inducers.motorGoTo(drag_inducers.encMax);
        }
        else {
          drag_inducers.motorGoTo(drag_inducers.encMin);
        }
      }
      //kalman(0, rawState, &filteredState);                   //feeds raw state into kalman filter and retrieves new filtered state.

      //LOG DATA
      flight_log.supStat.vSPP = vSPP_val;
      flight_log.logData(testMode);
      if (!fullBrakesTest) {
        // call motorGoTo again to make sure the blades didn't pass their setpoint 
        drag_inducers.motorGoTo(airBrakesEncPos_val);
      }
      else {
        if ((rawState.accel < 0) && (rawState.alt > 150) && (rawState.vel > 0)) {
          drag_inducers.motorGoTo(drag_inducers.encMax);
        }
        else {
          drag_inducers.motorGoTo(drag_inducers.encMin);
        }
      }
    }
    Serial.println("End of flight mode. Returning drag blades...");
    gui.flush_input();
    while (digitalRead(LIM_IN) && (Serial.available() == 0)) {
      delay(MOTORTEST_DELAY_MS);
      drag_inducers.motorDo(INWARD, DEADZONE_MAX + 10);
    }
    drag_inducers.motorDont();
  }

  // This is an ISR! it is called when the pin belonging to ENC_A sees a rising 
  // edge. This functions purpose is to keep track of the encoder's position.
  // Author: Jacob & Ben
  void doEncoder() {
    // If pinA and pinB are both high or both low, it is spinning forward. If 
    // they're different, it's going backward.
    if (digitalRead(ENC_A) == digitalRead(ENC_B))
      --drag_inducers.encPos;
    else
      ++drag_inducers.encPos;
  }
} // namespace

void setup() {
	bool begin = false;

	Serial.begin(38400);

  Serial.println("Welcome to VDS");
  Serial.println("Press 's' to start");

	while (!begin)
		if (Serial.available() > 0)
			if (Serial.read() == 's') begin = true;

	gui.flush_input();
	gui.printTitle();

	gui.init();
	flight_log.init();
	daq_controller.init(true);
	drag_inducers.init();

	attachInterrupt(digitalPinToInterrupt(ENC_A), doEncoder, RISING);
	drag_inducers.dragBladesCheck();
	gui.printMenu();
}

void loop() {
	char response;
	bool testMode = false;
	bool fullBrakesTest = false;
	if (Serial.available() > 0) {
		switch (Serial.read()) {
      case 'S': {
        s();
        break;
      }
      case 'D': {
        d();
        break;
      }
      case 'P': {
        p();
        break;
      }
      case 'I': {
        i();
        break;
      }
      case 'O': {
        o();
        break;
      }
      case 'R': {
        r();
        break;
      }
      case 'C': {
        Serial.println("\n\n----- Calibrate BNO055 -----;");
        break;
      }
      case 'A': {
        Serial.println("\n\n----- Testing Accelerometer -----;");
        break;
      }
      case 'M': {
        m();
        break;
      }
      case 'B': {
        Serial.println("\n\n----- Testing Barometric Pressure Sensor -----;");
        break;
      }
		  case 'F': {
        bool retflag;
        f(response, testMode, fullBrakesTest, retflag);
        if (retflag) return;
			  break;
      }
      default: {
        switch_default();
        break;
      }
		}
		gui.flush_input();
		gui.printMenu();
	}
}

} // namespace vds
} // namespace rcr

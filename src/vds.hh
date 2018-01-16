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

using namespace ::rcr::vds;

namespace rcr {
namespace vds {

namespace {
  DaqController daq_controller{};
  Gui gui{};
  const bool* const must_be_initialized[] {
    &::rcr::vds::bmp_initialized,
    &::rcr::vds::bno_initialized,
    &::rcr::vds::drag_inducers_initialized,
    &::rcr::vds::disk_initialized,
  };

  void flightMode(bool testMode, bool fullBrakesTest);

  void d() { ::rcr::vds::drag_inducers.dragBladesCheck(); }
  void f(bool &test_mode, bool &fullBrakesTest) {
    rcr::util::clear_input(Serial);
    out << "------Choose Flight Mode Settings-----\n";

    out << "test mode?\n";
    {
      auto auth = rcr::util::get_authorization(Serial);
      ::rcr::vds::flight_log.newFlight(auth);
      test_mode = auth;
    }

    out << "full-brake test mode?\n";
    fullBrakesTest = rcr::util::get_authorization(Serial);

    if (!test_mode) {
      for (auto& initialized : must_be_initialized) {
        if (!*initialized && !test_mode) {
          out << "Cannot enter flight mode. A sensor or sd card is not initialized.\n";
          ::rcr::vds::flight_log.logError(SENSOR_UNIT);
          return;
        }
      }
    }

    out << "entering flight mode\n";
    out << "Test Mode: ";
    Serial.println(test_mode ? "ON" : "OFF");
    out << "Full-brakes Test: ";
    Serial.println(fullBrakesTest ? "ON" : "OFF");

    if (!test_mode) ::rcr::vds::daq_controller.setPadAlt();
    flightMode(test_mode, fullBrakesTest);
  }
  void i() {
    out << "Inching Inward\n";
    ::rcr::vds::drag_inducers.motorDo(INWARD, DEADZONE_MAX + 15);
    ::rcr::vds::drag_inducers.motorDont();
  }
  void m() {
    rcr::util::clear_input(Serial);
    out << "\n\n----- Calibrate Motor -----;\n";
    ::rcr::vds::drag_inducers.motorTest();
  }
  void o() {
    out << "Inching Outward\n";
    ::rcr::vds::drag_inducers.motorDo(OUTWARD, DEADZONE_MAX + 15);
    delay(250);
    ::rcr::vds::drag_inducers.motorDont();
  }
  void p() {
    out << "Power test\n";
    rcr::util::clear_input(Serial);
    ::rcr::vds::drag_inducers.powerTest();
    ::rcr::vds::drag_inducers.motorDont();
  }
  void r() {
    rcr::util::clear_input(Serial);
    gui.rocketMenu();
  }
  void s() {
    ::rcr::vds::flight_log.init();
    ::rcr::vds::daq_controller.init(false);
    gui.init();
    ::rcr::vds::drag_inducers.dragBladesCheck();
  }
  void switch_default() {
    out << "Unkown code received - main menu\n";
    ::rcr::vds::flight_log.logError(INVALID_MENU);
  }

  /**************************************************************************/
  /*!
  @brief  Launch and test sequence.
  Author: Jacob & Ben
  */
  /**************************************************************************/
  void flightMode(bool testMode, bool fullBrakesTest) {
    VehicleState rawState{}, filteredState{};
    int airBrakesEncPos_val = 0;
    out << "asdfasdf\n";
    rcr::util::clear_input(Serial);
    while ((Serial.available() == 0) && daq_controller.getRawState(&rawState, testMode)) {
      auto vSPP_val = rcr::vds::maths::spp_velocity(rawState.alt, rawState.vel);
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

      // log data
      flight_log.supStat.spp_velocity = vSPP_val;
      flight_log.logData(testMode);
      if (!fullBrakesTest) {
        // call motorGoTo again to make sure the blades didn't pass their setpoint 
        drag_inducers.motorGoTo(airBrakesEncPos_val);
      }
      else {
        if ((rawState.accel < 0) && (rawState.alt > 150) && (rawState.vel > 0)) {
          rcr::vds::drag_inducers.motorGoTo(rcr::vds::drag_inducers.encMax);
        }
        else {
          drag_inducers.motorGoTo(drag_inducers.encMin);
        }
      }
    }
    out << "End of flight mode. Returning drag blades...\n";
    rcr::util::clear_input(Serial);
    while (digitalRead(rcr::vds::digital_io::Pin::LIM_IN) && (Serial.available() == 0)) {
      rcr::util::sleep_for(kMotorTestDelay);
      drag_inducers.motorDo(INWARD, DEADZONE_MAX + 10);
    }
    drag_inducers.motorDont();
  }

  // This is an ISR! it is called when the pin belonging to rcr::vds::digital_io::Pin::ENC_A sees a rising 
  // edge. This functions purpose is to keep track of the encoder's position.
  // Author: Jacob & Ben
  void doEncoder() {
    // If pinA and pinB are both high or both low, it is spinning forward. If 
    // they're different, it's going backward.
    if (digitalRead(rcr::vds::digital_io::Pin::ENC_A) == digitalRead(rcr::vds::digital_io::Pin::ENC_B))
      --rcr::vds::drag_inducers.encPos;
    else
      ++rcr::vds::drag_inducers.encPos;
  }
} // namespace

inline void
setup() {
	bool begin = false;

  out << "Welcome to VDS\n";
  out << "Press 's' to start\n";

	while (!begin)
		if (Serial.available() > 0)
			if (Serial.read() == 's')
        begin = true;
	
  rcr::util::clear_input(Serial);
  gui.printTitle();

  gui.init();
  rcr::vds::flight_log.init();
  rcr::vds::daq_controller.init(true);
  rcr::vds::drag_inducers.init();

	attachInterrupt(digitalPinToInterrupt(rcr::vds::digital_io::Pin::ENC_A), doEncoder, RISING);
  ::rcr::vds::drag_inducers.dragBladesCheck();
  gui.printMenu();
}

inline void
loop() {
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
        out << "\n\n----- Calibrate BNO055 -----;\n";
        break;
      }
      case 'A': {
        out << "\n\n----- Testing Accelerometer -----;\n";
        break;
      }
      case 'M': {
        m();
        break;
      }
      case 'B': {
        out << "\n\n----- Testing Barometric Pressure Sensor -----;\n";
        break;
      }
		  case 'F': {
        f(testMode, fullBrakesTest);
			  break;
      }
      default: {
        switch_default();
        break;
      }
		}
		rcr::util::clear_input(Serial);
		gui.printMenu();
	}
}

} // namespace vds
} // namespace rcr

#include "gui.hh"

#include "globals.hh"
#include "rcr_util.hh"

#include <math.h>
#include <string>

namespace rcr {
namespace vds {

/**************************************************************************/
/*!
@brief  Gets the current vehicle and loads it
Author: Ben
*/
/**************************************************************************/
void Gui::init() {
	out << "\r\n---Initializing rocket settings---\n";
	currentRocket = readUint8_t(3 * ROCKETSTRUCT_STORSIZE);
	if ((currentRocket > 3) || (currentRocket < 1) || isnan(currentRocket)) {
		out << "An incompatible rocket ID # was found on the EEPROM\r\nIs it possible that this is a new Teensy?\r\nSetting rocket ID # to 1\r\n\n";
		currentRocket = 1;
		writeUint8_t(currentRocket, 3 * ROCKETSTRUCT_STORSIZE);
	}
	loadRocket(currentRocket);
}

/**************************************************************************/
/*!
@brief  Prints one state and it's location in the pastRawStates array
Author: Jacob
*/
/**************************************************************************/
void Gui::printState(VehicleState state, int label) {
  out << "\n";
  out << label;
	out << ") alt = ";
  out << state.alt;
	out << ", vel = ";
  out << state.vel;
	out << ", accel = ";
  out << state.accel;
	out << ", time = ";
  out << state.time;
	out << ", buff_t = ";
  out << state.buff_t;
	out << ");\n";
}


  /**************************************************************************/
  /*!
  @brief  prints the state struct
  Author: Ben
  */
  /**************************************************************************/
void Gui::printState(VehicleState state, const char* label) {
  out << "\n";
  out << label;
	out << "alt =   ";
	Serial.println(state.alt, 3);
	out << "vel =   ";
	Serial.println(state.vel, 4);
	out << "accel = ";
	Serial.println(state.accel, 3);
	out << "t =     ";
	Serial.println(state.time, 6);
}


  /**************************************************************************/
  /*!
  @brief  Prints all pastRawState values.
  Author: Jacob
  */
  /**************************************************************************/
void Gui::printPastStates(VehicleState* states) {
	for (int i = 0; i < BUFF_N; i++) {
		printState(states[i], i);
	}
}


  /**************************************************************************/
  /*!
  @brief  Prints out the title sequence
  Author: Ben
  */
  /**************************************************************************/
void Gui::printTitle() {
  out << "| VARIABLE DRAG SYSTEM |";
}


  /**************************************************************************/
  /*!
  @brief  *HIDDEN* Menu Function.  Prints menu options.
  Author: Jacob
  */
  /**************************************************************************/
void Gui::printMenu() {
  out << "BMP:\n";
  out << report(bmp_initialized);
  out << "BNO:\n";
  out << report(bno_initialized);
	out << "SD:\n";
  out << report(disk_initialized);
	out << "Drag Inducers:\n";
  out << report(drag_inducers_initialized);

#if LIMITSWITCHES_DETATCHED
	out << "WARNING! LIMITSWITCHES_DETATCHED MODE IS ON!\n";
#endif
	delay(50);
	out << "\n--------- Menu -----------;\n";
	out << "'S' - (S)ystem Check\n";
	out << "'D' - (D)rag Blades Check\n";
	out << "'C' - (C)alibrate BNO055\n";
	out << "'R' - Edit (R)ockets\n";
	out << "'I' - Inch (I)nward\n";
	out << "'O' - Inch (O)utward\n";
	out << "'A' - (A)ccelerometer Test\n";
	out << "'B' - (B)arometric Pressure Sensor Test\n";
	out << "'M' - (M)otor Calibration & Test\n";
	out << "'F' - (F)light Mode\n";
}

/**************************************************************************/
/*!
@brief  Loads the vehicle indicated by whichOne from its designated spot in EEPROM
Author: Ben
*/
/**************************************************************************/
bool Gui::loadRocket(uint8_t whichOne) {
	vehicle.dryMass = readFloat(0 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.propMass = readFloat(4 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.Cd_r = readFloat(8 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.Cd_b = readFloat(12 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.Ar = readFloat(16 + (whichOne - 1)* ROCKETSTRUCT_STORSIZE);
	vehicle.Ab = readFloat(20 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.avgMotorThrust = readFloat(24 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.targetAlt = readFloat(28 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.interVel = readFloat(32 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	//TODO: vehicle.name = readString(36 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	vehicle.Cmin = (vehicle.Cd_r*vehicle.Ar*RHO / 2 / vehicle.dryMass);
	vehicle.Cmax = (vehicle.Cd_b*vehicle.Ab*RHO / 2 / vehicle.dryMass);
	vehicle.Cspp = (vehicle.Cmax + vehicle.Cmin) / 2;
	vehicle.interAlt = (vehicle.targetAlt - log(std::sqrt((400 * vehicle.Cmin*(vehicle.interVel*vehicle.interVel)) / 981 + 4) / 2) / vehicle.Cmin);

	printRocket();

	//add if here to check vehicle values against max nominal values
	if ((vehicle.dryMass < MAX_EXP_DRYMASS) && (vehicle.dryMass > 0) && 
		(vehicle.propMass < MAX_EXP_PROPMASS) && (vehicle.propMass > 0) && 
		(vehicle.Cd_r < MAX_EXP_CD_R) && (vehicle.Cd_r > 0) && 
		(vehicle.Cd_b < MAX_EXP_CD_B) && (vehicle.Cd_b > 0) && 
		(vehicle.Ar < MAX_EXP_AR) && (vehicle.Ar > 0) && 
		(vehicle.Ab < MAX_EXP_AB) && (vehicle.Ab > 0) &&
		(vehicle.avgMotorThrust < MAX_EXP_AVGMOTORTHRUST) && (vehicle.avgMotorThrust > 0) &&
		(vehicle.targetAlt < MAX_EXP_TARGETALT) && (vehicle.targetAlt > 0) &&
		(vehicle.interVel < MAX_EXP_INTERVEL) && (vehicle.interVel > 0) &&
		(vehicle.interAlt < MAX_EXP_INTERALT) && (vehicle.interVel > 0)) {
		return true;
	}
	else {
		out << "WARNING: NON-NOMINAL VALUES DETECTED IN ROCKET SETTINGS\n";
		return false;
	}
}

/**************************************************************************/
/*!
@brief  Saves the vehicle indicated by whichOne to the EEPROM in its designated spot
Author: Ben
*/
/**************************************************************************/
void Gui::saveRocket(uint8_t whichOne) {
	writeFloat(vehicle.dryMass, 0 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.propMass, 4 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.Cd_r, 8 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.Cd_b, 12 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.Ar, 16 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.Ab, 20 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.avgMotorThrust, 24 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.targetAlt, 28 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeFloat(vehicle.interVel, 32 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	writeString(vehicle.name, 36 + (whichOne - 1) * ROCKETSTRUCT_STORSIZE);
	eeprom_write_block((void*)&whichOne, (unsigned char*)(3 * ROCKETSTRUCT_STORSIZE), 1);
}

/**************************************************************************/
/*!
@brief  Prints out the settings of the currently selected vehicle
Author: Ben
*/
/**************************************************************************/
void Gui::printRocket() {
	//out << "Selected Vehicle = ";
	//Serial.println(vehicle.name);
	//delay(50);
	//Serial.println("Selected Vehicle # = %d\r\n", currentRocket);
	//delay(50);
	//Serial.println("dryMass = %f\r\n", vehicle.dryMass);
	//delay(50);
	//Serial.println("propMass = %f\r\n", vehicle.propMass);
	//delay(50);
	//Serial.println("Cd_r = %f\r\n", vehicle.Cd_r);
	//delay(50);
	//Serial.println("Cd_b = %f\r\n", vehicle.Cd_b);
	//delay(50);
	//Serial.println("Ar = %f\r\n", vehicle.Ar);
	//delay(50);
	//Serial.println("Ab = %f\r\n", vehicle.Ab);
	//delay(50);
	//Serial.println("avgMotorThrust = %d\r\n", vehicle.avgMotorThrust);
	//delay(50);
	//Serial.println("targetAlt = %d\r\n", vehicle.targetAlt);
	//delay(50);
	//Serial.println("interVel = %d\r\n", vehicle.interVel);
	//delay(50);
	//Serial.println("interAlt = %d\r\n", vehicle.interAlt);
}

/**************************************************************************/
/*!
@brief  Code for navigating the vehicle submenu
Author: Ben
*/
/**************************************************************************/
void Gui::rocketMenu() {
	bool exit = false;
	int tempVar;
	printRocketMenu();
	while (!exit) {
		if (Serial.available() > 0) {
			switch (Serial.read()) {
			case 'e'://edit vehicle
				editRocket();
				saveRocket(currentRocket);
				loadRocket(currentRocket);
				break;
			case 's'://switch rockets
				out << "------Switch Rockets-----\n";
				out << "Type a number 1-3\n";
        rcr::util::clear_input(Serial);
				while (!Serial.available()) {
					//wait
				}
				tempVar = Serial.parseInt();
				if ((tempVar > 3) || (tempVar < 1)) {
					out << " Type a number between 1 & 3\n";
				}
				else {
					currentRocket = tempVar;
				}
				eeprom_write_block((void *)&currentRocket, (unsigned char *)(3 * ROCKETSTRUCT_STORSIZE), 1);
				loadRocket(currentRocket);
				break;
			case 'x':
				exit = true;
				return;
				break;
			default:
				out << "unknown code received - rocket submenu\n";
				break;
			}
      rcr::util::clear_input(Serial);
			printRocketMenu();
		}
	}
}

/**************************************************************************/
/*!
@brief  Asks for user input to change the stored vehicle settings
Author: Ben
*/
/**************************************************************************/
void Gui::editRocket() {
	int eqlIndex;
	String myVariable;
	out << "------Editing Rockets------\n";
	printRocket();
	out << "Enter the variable you want to change in the following format:\r\nvariableName=value;\r\n\n";
	rcr::util::clear_input(out);
  std::string input;
  in >> input;

  String myString = Serial.readStringUntil(';');
	out << "String received: ";
	Serial.println(myString);
	eqlIndex = myString.indexOf('=');
	myVariable = myString.substring(0, eqlIndex);
	out << "variable parsed: ";
	Serial.println(myVariable);
	if (myVariable.equals("dryMass")) {
		vehicle.dryMass = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("propMass")) {
		vehicle.propMass = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Cd_r")) {
		vehicle.Cd_r = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Cd_b")) {
		vehicle.Cd_b = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Ar")) {
		vehicle.Ar = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Ab")) {
		vehicle.Ab = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("avgMotorThrust")) {
		vehicle.avgMotorThrust = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("targetAlt")) {
		vehicle.targetAlt = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("interVel")) {
		vehicle.interVel = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("name")) {
		vehicle.name = myString.substring(eqlIndex + 1).c_str();
		Serial.println(vehicle.name);
	}
	else {
		out << "Bad string received\n";
	}
}

/**************************************************************************/
/*!
@brief  Displays vehicle submenu
Author: Ben
*/
/**************************************************************************/
void Gui::printRocketMenu() {
	out << "\r\n------Rocket SubMenu-------\r\n\n";
	delay(100);
	out << "'e' - (e)dit rockets\n";
	delay(100);
	out << "'s' - (s)witch rockets\n";
	delay(100);
	out << "'x' - e(x)it rocket submenu\n";
}

/**************************************************************************/
/*!
@brief  All read/write EEPROM functions below. Read/writes different variable
types to the EEPROM
Author: Ben
*/
/**************************************************************************/
float Gui::readFloat(int address) {
	float out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}

uint8_t Gui::readUint8_t(int address) {
	uint8_t out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 1);
	return out;
}

void Gui::writeUint8_t(uint8_t value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 1);
}

void Gui::writeFloat(float value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 4);
}

void Gui::writeString(String value, int address) {
	unsigned char buf[4];
	value.getBytes(buf, 4);
	eeprom_write_block((void *)&buf, (unsigned char *)address, 4);
}

String Gui::readString(int address) {
	char out[32];
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}

// helper functions:

inline constexpr auto
report(bool x) -> const char* {
  return x ? "OK" : "uninitialized";
}

} // namespace vds
} // namespace rcr

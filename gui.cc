#include "gui.hh"

namespace rcr {
namespace vds {

/**************************************************************************/
/*!
@brief  Gets the current vehicle and loads it
Author: Ben
*/
/**************************************************************************/
void Gui::init() {
	Serial.println("\r\n---Initializing rocket settings---");
	currentRocket = readUint8_t(3 * ROCKETSTRUCT_STORSIZE);
	if ((currentRocket > 3) || (currentRocket < 1) || isnan(currentRocket)) {
		Serial.println("An incompatible rocket ID # was found on the EEPROM\r\nIs it possible that this is a new Teensy?\r\nSetting rocket ID # to 1\r\n");
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
  Serial.println();
	Serial.println(label);
	Serial.print(") alt = ");
	Serial.print(state.alt, 4);
	Serial.print(", vel = ");
	Serial.print(state.vel, 4);
	Serial.print(", accel = ");
	Serial.print(state.accel, 4);
	Serial.print(", time = ");
	Serial.print(state.time);
	Serial.print(", buff_t = ");
	Serial.print(state.buff_t, 4);
	Serial.println(");");
}


  /**************************************************************************/
  /*!
  @brief  prints the state struct
  Author: Ben
  */
  /**************************************************************************/
void Gui::printState(VehicleState state, const char* label) {
  Serial.println();
	Serial.println(label);
	Serial.print("alt =   ");
	Serial.println(state.alt, 3);
	Serial.print("vel =   ");
	Serial.println(state.vel, 4);
	Serial.print("accel = ");
	Serial.println(state.accel, 3);
	Serial.print("t =     ");
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
  //remember backslahses have to be double backslashed to print correctly
  Serial.println(F("             __      _______   _____  __      _____    __ "));
  delay(100);
  Serial.println(F("             \\ \\    / /  __ \\ / ____| \\ \\    / /__ \\   /_ |"));
  delay(100);
  Serial.println(F("              \\ \\  / /| |  | | (___    \\ \\  / /   ) |   | |"));
  delay(100);
  Serial.println(F("               \\ \\/ / | |  | |\\___ \\    \\ \\/ /   / /    | |"));
  delay(100);
  Serial.println(F("                \\  /  | |__| |____) |    \\  /   / /_   _| |"));
  delay(100);
  Serial.println(F("                 \\/   |_____/|_____/      \\/   |____| (_)_|"));
  delay(100);
  Serial.println("");
  Serial.println("             River City Rocketry's Variable Drag System");
  delay(100);
  Serial.println(" \t\t\t Full Scale Test Flights");
  delay(200);
  Serial.print(F("Software written by Jacob Cassady, "));
  delay(100);
  Serial.println(F("Ben Stringer, Lydia Sharp, and Denny Joy."));
  delay(100);
  Serial.println(F("With help from libraries written by Adafruit Industries."));
  delay(100);
  Serial.println(F("Mechanical hardware developed by Justin Johnson."));
  delay(100);
  Serial.println(F("Electrical hardware developed by Kenny Dang, Kristian Meyer, and Alora Mazarakis."));
  Serial.println("");
}


  /**************************************************************************/
  /*!
  @brief  *HIDDEN* Menu Function.  Prints menu options.
  Author: Jacob
  */
  /**************************************************************************/
void Gui::printMenu() {
  Serial.println("BMP:");
  Serial.println(report(bmp_initialized));
  Serial.println("BNO:");
  Serial.println(report(bno_initialized));
	Serial.println("SD:");
  Serial.println(report(disk_initialized));
	Serial.println("Drag Inducers:");
  Serial.println(report(drag_inducers_initialized));

#if LIMITSWITCHES_DETATCHED
	Serial.println("WARNING! LIMITSWITCHES_DETATCHED MODE IS ON!");
#endif
	delay(50);
	Serial.println("\n--------- Menu -----------;");
	Serial.println("'S' - (S)ystem Check");
	Serial.println("'D' - (D)rag Blades Check");
	Serial.println("'C' - (C)alibrate BNO055");
	Serial.println("'R' - Edit (R)ockets");
	Serial.println("'I' - Inch (I)nward");
	Serial.println("'O' - Inch (O)utward");
	Serial.println("'A' - (A)ccelerometer Test");
	Serial.println("'B' - (B)arometric Pressure Sensor Test");
	Serial.println("'M' - (M)otor Calibration & Test");
	Serial.println("'F' - (F)light Mode");
}


void Gui::flush_input() {
	while (Serial.available()) Serial.read();
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
	vehicle.interAlt = (vehicle.targetAlt - log(sqrt((400 * vehicle.Cmin*(vehicle.interVel*vehicle.interVel)) / 981 + 4) / 2) / vehicle.Cmin);

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
		Serial.println("WARNING: NON-NOMINAL VALUES DETECTED IN ROCKET SETTINGS");
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
	//Serial.print("Selected Rocket = ");
	//Serial.println(vehicle.name);
	//delay(50);
	//Serial.println("Selected Rocket # = %d\r\n", currentRocket);
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
				Serial.println("------Switch Rockets-----");
				Serial.println("Type a number 1-3");
        rcr::util::clear_input(Serial);
				while (!(Serial.available() > 0)) {
					//wait
				}
				tempVar = Serial.parseInt();
				if ((tempVar > 3) || (tempVar < 1)) {
					Serial.println(" Type a number between 1 & 3");
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
				Serial.println("unknown code received - rocket submenu");
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
	Serial.println("------Editing Rockets------");
	printRocket();
	Serial.println("Enter the variable you want to change in the following format:\r\nvariableName=value;\r\n");
	rcr::util::clear_input(Serial);
	while (!(Serial.available() > 0)) {}

  String myString = Serial.readStringUntil(';');
	Serial.print("String received: ");
	Serial.println(myString);
	eqlIndex = myString.indexOf('=');
	myVariable = myString.substring(0, eqlIndex);
	Serial.print("variable parsed: ");
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
		Serial.println("Bad string received");
	}
}

/**************************************************************************/
/*!
@brief  Displays vehicle submenu
Author: Ben
*/
/**************************************************************************/
void Gui::printRocketMenu() {
	Serial.println("\r\n------Rocket SubMenu-------\r\n");
	delay(100);
	Serial.println("'e' - (e)dit rockets");
	delay(100);
	Serial.println("'s' - (s)witch rockets");
	delay(100);
	Serial.println("'x' - e(x)it rocket submenu");
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

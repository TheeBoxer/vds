#ifndef _RCR_VDS_H_
#define _RCR_VDS_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Adafruit_BMP280.h"
#include "globals.hh" 
#include "RCR_Bmp180.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SdFat.h>
#include "SPI.h"
#include "pid.hh"
#include <eeprom.h>

struct stateStruct {
	float alt;                                                    //The most recent altitude reading from Adafruit BMP180 sensor           (m)
	float vel;                                                    //The most recent velocity derived from calculateVelocity() function     (m/s)
	float accel;                                                  //The most recent acceleration reading from Adafruit BNO055 sensor       (m/s^2)
	unsigned long time;                                           //Time since the program began                                           (sec*10^-5)
	float buff_t;                                                 //The time relative to the present moment. (used in calculateVelocity()) (s)
};

class DaqController {
 public:
	void init(bool bnoToo);
	void setPadAlt(void);
  bool getRawState(struct stateStruct* rawState, bool testMode);

 protected:
  Adafruit_BNO055 bno{};                        //stores BNO055 object
  Adafruit_BMP280 bmp280{};
	float padAlt;
	bool timeOverflow = false;
	float lastAlt;
	float altitude_plz();
	float calculateVelocity(struct stateStruct rawState);           //Calculates velocity using alt from bmp180 and accel from BNO055.
	float get_vertical_accel();										//Returns the vertical acceleration as a floating point value.
	float get_vertical_accel(imu::Vector<3> gravity, imu::Vector<3> linear);
	
  // deep copy
  void copyState(struct stateStruct* destination, struct stateStruct* original);

	stateStruct pastRawStates[BUFF_N];                       //Stores past BUFF_N state structures
};

extern DaqController daq_controller;

#endif // _RCR_VDS_H_

#ifndef _DATALOG_h
#define _DATALOG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

struct stateToLogStruct {
	unsigned long time;
	float alt;
	float vel;
	float leftVel;
	float rightVel;
	float accel;
	float rollAxisGrav;
	float yawAxisGrav;
	float pitchAxisGrav;
	float rollAxisLin;
	float yawAxisLin;
	float pitchAxisLin;
	float rollAxisGyro;
	float yawAxisGyro;
	float pitchAxisGyro;
	float roll;
	float yaw;
	float pitch;
	float alt_k;
	float vel_k;
	float accel_k;
	float vSPP;
	int16_t encPos;
	int16_t encPosCmd;
	bool limit_out;
	bool limit_in;
	int16_t encMax;
	int16_t encMin;
	int mtrSpdCmd;
};

class FlightLog {
 public:
	void printTestFileNames();
	void init();
	void* sd;
	void logData(bool testMode);
	stateToLogStruct supStat;
	void logError(String error);				 //Stores error to VDSv2Errors.dat.
	void newFlight(bool flightMode);						//Initiates files and variables for a new flight.
	bool readCSV(struct stateStruct* destination);
 protected:
  File data;                                                      //Stores file object
  int pos = 0;
  uint32_t testFileSize;
};

extern FlightLog log;

#endif

#ifndef _GUI_h
#define _GUI_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



class GUIClass
{
protected:
	void editRocket();
	float readFloat(int address);
	void writeFloat(float value, int address);
	String readString(int address);
	uint8_t readUint8_t(int address);
	void writeUint8_t(uint8_t value, int address);
	void writeString(String value, int address);
	bool loadRocket(uint8_t whichOne);
	void saveRocket(uint8_t whichOne);
	void printRocket();
	void printRocketMenu();
	uint8_t currentRocket = 1;

public:
	void init();
	void rocketMenu();
	void printPastStates(struct stateStruct*);                      //Prints all pastRawState values.
	void printState(struct stateStruct, int);                       //Prints one state and it's location in the pastRawStates array.
	void printState(struct stateStruct, String);                       //Prints one state and it's location in the pastRawStates array.
	void printTitle(void);                                          //Prints out the title sequence.
	void printMenu(void);
	void eatYourBreakfast();
};

extern GUIClass GUI;

#endif // _GUI_h

/*
  AISFCAlphaBuild Main File
  This program is a stripped down and cleaned version of the AISFCProto code
  This program is intended to act as a clean reference for future expansion

  Created: 22nd May 2023
  Last Update: 25th May 2023
  Created By: Michael Haggart
  For: StarthAIS
  Updated by: Michael Haggart
			  #Add New Names Here
*/
//
//


#include "AISFCLED.h"
#include "AISFCAccelerometer.h"
#include "AISFCBarometer.h"
#include "AISFCCore.h"
#include "AISFCDataLogging.h"
#include "AISFCGPS.h"
//
//
#define drogueIgGate 22
#define drogueIgDrain 23

#define mainIgGate 24
#define mainIgDrain 25

/*~~INDIVIDUAL LED PINS FOR TESTING: UNCOMMENT AS NEEDED*/
//#define blue1   22
//#define blue2   23

//#define white1  24
//#define white2  25

//#define green1  26
//#define green2  27

//#define red1    28
//#define red2    29

//#define yellow1 30
//#define yellow2 31
//
//
/*
	Mosfet 1 gate - 11
	Mosfet 1 drain - 12
	Mosfet 2 drain - 13
	Mosfet 2 gate - 14
*/
#define ledAltIndicator 8
#define ledStatusIndicator 9
//
//
/*~~Global Variables~~*/
float timeSinceActive{};
flightStatus currentFS = flightStatus::preLaunch;
flightStatus prevFS{};
bool apogeeCheck_Flag = false;
bool motorCheck_Flag = false;
bool drogueDep_Flag = false;
bool mainDep_Flag = false;
bool stationary_Flag = false;
float time_update = 0;
/*~~End Of Global Variables~~*/
//
//
/*~~Accelerometer Components~~*/
Adafruit_MPU6050 AISFC_Accel;
Adafruit_Sensor *ASIFCAccel_temp, *ASIFCAccel_accel, *ASIFCAccel_gyro;
float accel_x{}, accel_y{}, accel_z{};
const int AISFCAccel_Address1 = 0x69;  //IF we use a second accelerometer, address would be 0x68
float AISFCAccel_Mag{};
int accelSampleCount{};
/*~~End Of Accelerometer Components~~*/
//
//
/*~~Barometer Components~~*/
AISFCbaro AISFC_Baro;
float highest_Alt{};
int baroSampleCount{};
float baroPressure{}, zero_Alt{}, current_Alt{}, baroAltFeet{};
/*~~End Of Barometer Components~~*/
//
//
/*~~LED Indicators~~*/
Adafruit_NeoPixel AISFCAltIndicator(AISFCLED::NUM_LEDS, ledAltIndicator, NEO_RGBW);
Adafruit_NeoPixel AISFCStatusIndicator(AISFCLED::NUM_LEDS, ledStatusIndicator, NEO_RGBW);
/*~~End Of LED Indicators~~*/
//
//
/*~~GPS Components~~*/
SFE_UBLOX_GNSS AISFCgps;
long lat_mdeg{}, long_mdeg{}, gnss_alt{};
long launch_Lat{}, launch_Long{};
long heading_from_launch{}, distance_from_launch{};
long last_Time = 0;
uint16_t gps_Year{};
uint8_t gps_Month{}, gps_Day{}, gps_Hour{}, gps_Minute{}, gps_Second{};
/*~~End Of GPS Components~~*/
//
//
/*~~Function Defs~~*/
bool activateHardware();
void fsAction(flightStatus currentFS);
bool stationary_Check(AISFCbaro AISFC_Baro, Adafruit_MPU6050 AISFC_Accel);
/*~~End Of Function Defs~~*/
//
//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  digitalWrite(drogueIgGate, HIGH);  //<- gate HIGH & drain LOW = no activation: for testing this should be blue1
  digitalWrite(drogueIgDrain, LOW);

  digitalWrite(mainIgGate, HIGH);  //<- gate HIGH & drain LOW = no activation: for testing this should be white1
  digitalWrite(mainIgDrain, LOW);
  
  Serial.println("AISFC V1.0");
  Serial.println("Last Update: 31-05-2023");
  Serial.println("Check for latest updates: https://github.com/EanElectric/AISFC");

  if (!activateHardware()) {
    Serial.println("Not all hardware activated successfully, check terminal log for failed device");
  } else {
    Serial.println("Hardware Activated Successfully");
  }

  AISFC_Accel.getAccelerometerSensor();
  Serial.println("Startup Complete. Starting main loop...");
}
//
//
void loop() {
  // put your main code here, to run repeatedly:
  timeSinceActive = millis();  // <- get time since activation

  /*~~Accelerometer Actions~~*/
  AISFCAccelerometer::get_xya(AISFCAccel_Address1, accel_x, accel_y, accel_z);           // <- get the xyz accelerations
  AISFCAccel_Mag = AISFCAccelerometer::absoluteAcceleration(accel_x, accel_y, accel_z);  // <- create absolute acceleration
  if (motorCheck_Flag == false)                                                          // <- if we haven't turned the motor on, check this
  {
    motorCheck_Flag = motorCheckFunction(accelSampleCount, AISFCAccel_Mag);  // <- check if rocket is accelerating due to motorburn //I want to gang this to thermistor but prob too late now
  }

  /*~~Barometer Actions~~*/
  current_Alt = AISFC_Baro.curAlt(zero_Alt);  // <- update current altitude
  if (highest_Alt < current_Alt)              // <- if the highest altitude is lower than the current altitude, update highest alt
  {
    highest_Alt = current_Alt;
  }
  if (apogeeCheck_Flag == false)  // <- if we haven't hit the apogee, check if reached
  {
    apogeeCheck_Flag = descendingCheck(baroSampleCount, highest_Alt, current_Alt);  // <- check if apogee has been hit
  }

  //Final Action of Current Loop, set current flight status, and if that's changed, update action
  currentFS = stateCheckFunc(currentFS, timeSinceActive, apogeeCheck_Flag, drogueDep_Flag, mainDep_Flag, motorCheck_Flag, highest_Alt, current_Alt);
  if (currentFS != prevFS) {
    fsAction(currentFS);
  }

  /*~~GPS Actions~~*/
  if (AISFCGPS::initGPS(AISFCgps)) {                                   // <- Check if GPS was activated
    if (AISFCGPS::updateGPS(AISFCgps, lat_mdeg, long_mdeg, gnss_alt))  // <- if GPS has a fresh update, re-calculate distance and bearing from launch. [Leon]: If GPS was not active then updateGPS() would restart the whole sketch
    {
      AISFCGPS::distance_bearingGPS(launch_Lat, launch_Long, lat_mdeg, long_mdeg);
      AISFCGPS::getTimeGPS(AISFCgps, gps_Year, gps_Month, gps_Day, gps_Hour, gps_Minute, gps_Second);
    }
  }

  //End of current Loop, prepare for next loop
  prevFS = currentFS;

  if((timeSinceActive - time_update) > 1000) {  // <- Prints stats every second
    Serial.print("Time (ms): ");
    Serial.println(timeSinceActive);
    Serial.print("Acceleration (unit???): ");
    Serial.println(AISFCAccel_Mag);
    Serial.print("Altitude (unit???): ");
    Serial.println(current_Alt);
    time_update += 1000;
  }

}
//
//
bool activateHardware() {
  //AISFCAccel
  bool accel_Flag{}, baro_Flag{}, altInd_Flag{}, statusInd_Flag{}, gps_Flag{};
  for (int i = 0; i < 4; i++)  //i < n, where n = number of components used
  {
    switch (i) {
      case 0:
        //Accelerometer activation
        if (!AISFC_Accel.begin(AISFCAccel_Address1)) {
          Serial.println("Accelerometer failed to activate");
          accel_Flag = false;
        } else {
          Serial.println("Accelerometer Activated");
          accel_Flag = true;
        }
        continue;
      case 1:
        //barometer activation
        if (!AISFC_Baro.begin()) {
          Serial.println("Barometer failed to activate");
          baro_Flag = false;
        } else {
          Serial.println("Barometer Activated");
          zero_Alt = AISFC_Baro.zeroAlt();
          Serial.print("Zeroed Altitude: ");
          Serial.println(zero_Alt);
          baro_Flag = true;
        }
        continue;
      case 2:
        //LED activation
        if (!AISFCLED::ledInit(AISFCAltIndicator)) {
          Serial.println("Altitude Indicator failed to activate");
          altInd_Flag = false;
        } else {
          Serial.println("Altitude Indicator Activated");
          altInd_Flag = true;
        }
        if (!AISFCLED::ledInit(ledStatusIndicator)) {
          Serial.println("Status Indicator failed to activate");
          statusInd_Flag = false;
        } else {
          Serial.println("Status Indicator Activated");
          statusInd_Flag = true;
        }
        continue;
      case 3:
        //GPS Activation
        if (!AISFCGPS::initGPS(AISFCgps)) {
          Serial.println("GPS failed to activate");
          gps_Flag = false;
        } else {
          AISFCGPS::zeroLaunchSiteGPS(AISFCgps, launch_Lat, launch_Long);
          Serial.println("GPS Activated");
          gps_Flag = true;
        }
        continue;
      case 4:
        //telecoms actication
        continue;
      default:
        continue;
    }
  }

  if (accel_Flag == true && baro_Flag == true && altInd_Flag == true && statusInd_Flag == true && gps_Flag == true) {
    Serial.println("All Hardware Activated Successfully");
    return true;
  } else if (accel_Flag == true && baro_Flag == true && altInd_Flag == true && statusInd_Flag == true && gps_Flag == false) {
    Serial.println("Only Accelerometer, Barometer, Alt Indcator, and Status Indicator Activated Successfully...");
    Serial.println("Skipping GPS Functionality");
    return true;
  } else {
    return false;
  }
}
//
//
void fsAction(flightStatus currentFS) {
  //bool drogueDep_Flag = false;
  //bool mainDep_Flag = false;
  switch (currentFS) {
    case 0:  //preLaunch,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cYellow);
      break;
    case 1:  //Boost,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cGreen);
      break;
    case 2:  //Coast,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cCyan);
      break;
    case 3:  //Apogee,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cWhite);
      digitalWrite(drogueIgGate, LOW);
      digitalWrite(drogueIgDrain, HIGH);
      drogueDep_Flag = true;
      break;
    case 4:  //Drogue,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cBlue);
      digitalWrite(drogueIgGate, HIGH);
      digitalWrite(drogueIgDrain, LOW);
      break;
    case 5:  //Main,
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cMagenta);
      digitalWrite(mainIgGate, LOW);
      digitalWrite(mainIgDrain, HIGH);
      mainDep_Flag = true;
      break;
    case 6:  //Landed
      AISFCStatusIndicator.setPixelColor(0, AISFCLED::cYellow);
      digitalWrite(mainIgGate, HIGH);
      digitalWrite(mainIgDrain, LOW);
      break;
  }
}

bool stationary_Check(AISFCbaro AISFC_Baro, Adafruit_MPU6050 AISFC_Accel) {
  AISFC_Baro.readAltitude();
}

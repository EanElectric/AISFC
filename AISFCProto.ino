/*
  AISFCAlphaBuild Main File
  This program is a stripped down and cleaned version of the AISFCProto code
  This program is intended to act as a clean reference for future expansion

  Created: 22nd May 2023
  Last Update: 02th May 2023
  Created By: Michael Haggart
  For: StarthAIS
  Updated by: Michael Haggart
			  Leon Yip
			  Preben Rasmussen
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
#include "RTClib.h"
//
//
#include "MemoryFree.h"
#include "pgmStrToRAM.h"
//
//
#define drogueIgGate 42
#define drogueIgDrain 43

#define mainIgGate 32
#define mainIgDrain 33

#define buzzer 31

#define CSpin 53
#define SCKpin 52
#define MoSipin 51
#define MiSopin 50

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
bool RTC_Flag = false;
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
SFE_UBLOX_GNSS AISFC_gps;
long lat_mdeg{}, long_mdeg{}, gnss_alt{};
long launch_Lat{}, launch_Long{};
long heading_from_launch{}, distance_from_launch{};
long last_Time = 0;
uint16_t gps_Year{};
uint8_t gps_Month{}, gps_Day{}, gps_Hour{}, gps_Minute{}, gps_Second{};
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
/*~~End Of GPS Components~~*/
//
//
/*~~RTC Clock~~*/
RTC_PCF8523 AISFC_RTC;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

/*~~End of RTC Clock~~*/
//
//
/*~~Function Defs~~*/
bool activateHardware();
void fsAction(flightStatus currentFS);
bool stationary_Check(AISFCbaro AISFC_Baro, Adafruit_MPU6050 AISFC_Accel);
File testFile;
/*~~End Of Function Defs~~*/
//
//
void setup() {
  // put your setup code here, to run once:
  pinMode(buzzer, OUTPUT);
  tone(buzzer, 500);
  delay(250);
  noTone(buzzer);
  Serial.begin(115200);
  Wire.begin();
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
  /*~~ SD CARD Prototype~~*/
  SD.begin(CSpin);
  testFile = SD.open("Test.txt", FILE_WRITE);

  Serial.println("Startup Complete. Starting main loop...");
  tone(buzzer, 1000);
  delay(250);
  noTone(buzzer);
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
  Serial.println("Accel done");
  /*~~Barometer Actions~~*/
  current_Alt = AISFC_Baro.curAlt(zero_Alt);  // <- update current altitude
  baroPressure = AISFC_Baro.readPressure();
  Serial.println("Baro done");
  if (highest_Alt < current_Alt)  // <- if the highest altitude is lower than the current altitude, update highest alt
  {
    highest_Alt = current_Alt;
  } else {
    highest_Alt = highest_Alt;
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
  Serial.println("Current state check done");
  /*~~GPS Actions~~*/
  // AISFC_gps.checkUblox(); 
  // Serial.println("Ublox check done");
  // if (nmea.isValid()) 
  // {
  //   lat_mdeg = nmea.getLatitude();
  //   long_mdeg = nmea.getLongitude();
  //   Serial.println("lat long check done");
  // } 
  // else 
  // {
  //   Serial.println("Waiting for fresh GPS data");
  //   nmea.clear();
  // }
  //AISFCGPS::distance_bearingGPS(launch_Lat, launch_Long, lat_mdeg, long_mdeg);
  //AISFCGPS::getTimeGPS(AISFC_gps, gps_Year, gps_Month, gps_Day, gps_Hour, gps_Minute, gps_Second);
  //Serial.println("time check done");
  //End of current Loop, prepare for next loop
  prevFS = currentFS;
  Serial.println("Cur state check updated");
  // if ((timeSinceActive - time_update) > 1000) {  // <- Prints stats every second
  // 	Serial.print("Time (ms): ");
  // 	Serial.println(timeSinceActive);
  // 	Serial.print("Acceleration (g): ");
  // 	Serial.println(AISFCAccel_Mag);
  // 	Serial.print("Altitude (m (AGL)???): ");
  // 	Serial.println(current_Alt);
  // 	Serial.print("Highest Altitude (m (AGL)???): ");
  // 	Serial.println(highest_Alt);

  // 	Serial.print("X accel: ");
  // 	Serial.println(accel_x);
  // 	Serial.print("Y accel: ");
  // 	Serial.println(accel_y);
  // 	Serial.print("Z accel: ");
  // 	Serial.println(accel_z);

  // 	Serial.print("Motor Active: ");
  // 	Serial.println(motorCheck_Flag);
  // 	Serial.print("Apogee Check: ");
  // 	Serial.println(apogeeCheck_Flag);
  // 	Serial.print("Drogue Deploy: ");
  // 	Serial.println(drogueDep_Flag);
  // 	Serial.print("Main Deploy: ");
  // 	Serial.println(mainDep_Flag);

  // 	Serial.print("Flight Status: ");
  // 	Serial.println(currentFS);
  // 	time_update += 1000;
  // }
  Serial.print(timeSinceActive);
  Serial.print(", ");
    Serial.print(baroPressure);
  Serial.print(", ");
    Serial.print(current_Alt);
  Serial.print(", ");
    Serial.print(accel_x);
  Serial.print(", ");
    Serial.print(accel_y);
  Serial.print(", ");
    Serial.print(accel_z);
  Serial.print(", ");
      Serial.print(lat_mdeg);
  Serial.print(", ");
      Serial.println(long_mdeg);
  // String log = loggedData(timeSinceActive, baroPressure, current_Alt, accel_x, accel_y, accel_z, lat_mdeg, long_mdeg);
  // //testFile.println(log);
  // Serial.println(log);
  //nmea.clear();
  //Serial.println(freeMemory());  // print how much RAM is available in bytes.
  //delay(2000);
  //freeMemory();
}
//
//
bool activateHardware() {
  //AISFCAccel
  bool accel_Flag{}, baro_Flag{}, altInd_Flag{}, statusInd_Flag{}, gps_Flag{};
  for (int i = 0; i < 3; i++)  //i < n, where n = number of components used
  {
    switch (i) {
      case 0:
        //Accelerometer activation
        if (!AISFC_Accel.begin(AISFCAccel_Address1)) {
          Serial.println("Accelerometer failed to activate");
          accel_Flag = false;
        } else {
          Serial.println("Accelerometer Activated");
          AISFC_Accel.setAccelerometerRange(MPU6050_RANGE_16_G);
          AISFC_Accel.setGyroRange(MPU6050_RANGE_250_DEG);
          AISFC_Accel.setFilterBandwidth(MPU6050_BAND_21_HZ);
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
        if (AISFC_gps.begin() == false) {
          Serial.println("GPS failed to activate");
          gps_Flag = false;
        }

        AISFC_gps.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);     //Set the I2C port to output both NMEA and UBX messages
        AISFC_gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);     //Save (only) the communications port settings to flash and BBR
        AISFC_gps.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);  // Make sure the library is passing all NMEA messages to processNMEA
        AISFC_gps.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);  // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it
        AISFC_gps.setMeasurementRate(500);
        AISFC_gps.setI2CpollingWait(25);
        AISFC_gps.checkUblox();
        if (nmea.isValid() == false) {
          Serial.println("Waiting for GPS Link...");
          delay(1000);
          AISFC_gps.checkUblox();
        }
        // AISFCGPS::zeroLaunchSiteGPS(AISFC_gps, nmea, launch_Lat, launch_Long);
        // Serial.println("GPS Activated");
        // Serial.print("Launch site - Lat: ");
        // Serial.print(launch_Lat / 1000000., 6);
        // Serial.print(" - Long: ");
        // Serial.println(launch_Long / 1000000., 6);
        gps_Flag = true;
        continue;
      case 4:
        //telecoms actication
        continue;
      case 5:
        //RTC clock activation
        if (!AISFC_RTC.begin()) {
          Serial.println("RTC failed to activate");
          RTC_Flag = false;
        }
        if (!AISFC_RTC.initialized() || AISFC_RTC.lostPower()) {
          Serial.println("RTC is NOT initialized, let's set the time!");
          AISFC_RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
      default:
        continue;
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

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

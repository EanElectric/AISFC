#include "AISFCGPS.h"
#include "AISFCAccelerometer.h"
#include "AISFCBarometer.h"

#include <SPI.h>
#include <SD.h>

Adafruit_MPU6050 mpu1;
Adafruit_Sensor* mpu1_temp, * mpu1_accel, * mpu1_gyro;
const int mpu1_address = 0x69;

File dataLog;
SFE_UBLOX_GNSS NEO_M9;

AISFCbaro baro;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Test bed for Arduino Flight Computer");
    if (!activateHardware())
    {
        Serial.println("Not all hardware activated successfully, check terminal log for failed device");
        while (1)
        {
            delay(10);
        }
    }
    if (activateHardware())
    {
        Serial.println("Hardware activated successfully");
    }
    AISFCAccelerometer::getSensors(mpu1, mpu1_accel, mpu1_gyro, mpu1_temp);

}

void loop() {
    AISFCAccelerometer::printSensors(mpu1_address);
    // put your main code here, to run repeatedly:

}

bool activateHardware()
{
    bool accelFlag01{}, SDFlag{}, gpsFlag{};
    if (!mpu1.begin(mpu1_address))
    {
        Serial.println("Accelerometer 1 failed to activate");
        accelFlag01 = false;
    }
    if (mpu1.begin(mpu1_address))
    {
        Serial.println("Accelerometer 1 activated");
        accelFlag01 = true;
    }

    if (!SD.begin(4))
    {
        Serial.println("SD data logger failed to activate");
        SDFlag = false;
    }
    if (SD.begin(4))
    {
        Serial.println("SD data logger activated");
        if (dataLog)
        {
            dataLog.println("Flight Log");
        }
        SDFlag = true;
    }
    if (!NEO_M9.begin())
    {
        Serial.println("GPS failed to activate");
        gpsFlag = false;
    }
    if (NEO_M9.begin())
    {
        Serial.println("GPS activated");
        startGPS(NEO_M9);
    }
    


    if (accelFlag01 == true && SDFlag == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}
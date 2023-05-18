// AISFCAccelerometer.h

#ifndef AISFCAccelerometer
#define AISFCAccelerometer

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>                                  //Needed for I2C to GY-521

int16_t accelerometer_x, accelerometer_y, accelerometer_z;  // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                             // variables for gyro raw data
int16_t temperature;                                        // variables for temperature data
sensors_event_t ac, gy, te;


char temp_str[7];                                              // temporary variable used in convert function

char* convert_int16_to_str(int16_t i);   // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
void getSensors(Adafruit_MPU6050 mpu, Adafruit_Sensor* accelSens, Adafruit_Sensor* gyroSens, Adafruit_Sensor* tempSens);
void printSensors(Adafruit_MPU6050 mpu);
void setRange(Adafruit_MPU6050 mpu);
//int setFilLScaleAccelRange(MPU6050_ACCEL_FS_16);



char* convert_int16_to_str(int16_t i)   // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
{
	sprintf(temp_str, "%6d", i);           //"string print format" (to char array temp_str, signed decimal thats 6 chars long, from i)
	return temp_str;
}

void getSensors(Adafruit_MPU6050 mpu, Adafruit_Sensor* accelSens, Adafruit_Sensor* gyroSens, Adafruit_Sensor* tempSens)
{
	accelSens = mpu.getAccelerometerSensor();
	accelSens->printSensorDetails();
	gyroSens = mpu.getGyroSensor();
	gyroSens->printSensorDetails();
	tempSens = mpu.getTemperatureSensor();
	tempSens->printSensorDetails();
}

void setRange(Adafruit_MPU6050 mpu)
{
	mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
	mpu.setGyroRange(MPU6050_RANGE_250_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void printSensors(const int address)
{
	Wire.beginTransmission(address);
	Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
	Wire.endTransmission(false);              // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
	Wire.requestFrom(address, 7 * 2, true);  // request a total of 7*2=14 registers
	accelerometer_x = Wire.read() << 8 | Wire.read();  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
	accelerometer_y = Wire.read() << 8 | Wire.read();  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
	accelerometer_z = Wire.read() << 8 | Wire.read();  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
	temperature = Wire.read() << 8 | Wire.read();      // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
	gyro_x = Wire.read() << 8 | Wire.read();           // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
	gyro_y = Wire.read() << 8 | Wire.read();           // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
	gyro_z = Wire.read() << 8 | Wire.read();           // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
	Serial.print("aX = ");
	Serial.println(convert_int16_to_str(accelerometer_x));
	Serial.println(ac.acceleration.x);
	Serial.print(" | aY = ");
	Serial.println(convert_int16_to_str(accelerometer_y));
	Serial.println(ac.acceleration.y);
	Serial.print(" | aZ = ");
	Serial.println(convert_int16_to_str(accelerometer_z));
	Serial.println(ac.acceleration.z);
}


#endif

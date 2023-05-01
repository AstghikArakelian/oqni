/*
 * BMX055.c
 *
 *  Created on: Apr 8, 2023
 *      Author: armen
 */

#include "BMX055.h"
#include "Math.h"
#include <math.h>
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef cur_i2c;

/* BMX055_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: August 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic BMX-055 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is intended specifically for the BMX055+MS5637 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption od 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the BMX055+MS5637 Mini Add-On board for Teensy 3.1.

 Hardware setup:
 BMX055 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND

 Note: The BMX055 is an I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */


#ifndef PI

#define PI       3.14159265358979323846264338328      // Pi
#endif


// Specify sensor full scale
//uint8_t OSR    = ADC_8192;         // set pressure amd temperature oversample rate
uint8_t Gscale;// = GFS_125DPS;       // set gyro full scale
uint8_t GODRBW;// = G_200Hz23Hz;      // set gyro ODR and bandwidth
uint8_t Ascale;// = AFS_2G;           // set accel full scale
uint8_t ACCBW;//  = 0x08 | ABW_16Hz;  // Choose bandwidth for accelerometer, need bit 3 = 1 to enable bandwidth choice in enum
uint8_t Mmode;//  = Regular;          // Choose magnetometer operation mode
uint8_t MODR;//   = MODR_10Hz;        // set magnetometer data rate
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
signed char   dig_x1;
signed char   dig_y1;
signed char   dig_x2;
signed char   dig_y2;
uint16_t      dig_z1;
int16_t       dig_z2;
int16_t       dig_z3;
int16_t       dig_z4;
unsigned char dig_xy1;
signed char   dig_xy2;
uint16_t      dig_xyz1;

// Pin definitions
// The BMX055 has three sensors, and two interrupt pins per device!
int intACC1;//   =  8;  // These are fixed on the BMX055 Mini Add-On for Teensy 3.1
int intACC2;//   =  9;
int intGYRO1;//  = 11;
int intGYRO2;//  = 10;
int intMAG1;//   = 12;
int intDRDYM;//  = 15;
int myLed;//     = 13;  // LED on the Teensy 3.1

// BMX055 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
float gyroBias[3], accelBias[3], magBias[3];  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test



// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
const float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
const float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void BMX055_init_globals()
{
	Gscale = GFS_125DPS;
	GODRBW = G_200Hz23Hz;      // set gyro ODR and bandwidth
	Ascale = AFS_2G;           // set accel full scale
	ACCBW  = 0x08 | ABW_16Hz;  // Choose bandwidth for accelerometer, need bit 3 = 1 to enable bandwidth choice in enum
	Mmode  = Regular;          // Choose magnetometer operation mode
	MODR   = MODR_10Hz;

	intACC1   =  8;  // These are fixed on the BMX055 Mini Add-On for Teensy 3.1
	intACC2   =  9;
	intGYRO1  = 11;
	intGYRO2  = 10;
	intMAG1   = 12;
	intDRDYM  = 15;
	myLed     = 13;  // LED on the Teensy 3.1

	// Bias corrections for gyro, accelerometer, mag
	for (int i=0; i < 3; i++)
	{
		gyroBias[i] = 0;
		accelBias[i] = 0;
		magBias[3] = 0;
	}


}

bool BMX055_setup()
{

//  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
//  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
//  delay(5000);
//  Serial.begin(38400);
//
//  // Set up the interrupt pin, its set as active high, push-pull
//  pinMode(intACC1,  INPUT);
//  pinMode(intACC2,  INPUT);
//  pinMode(intGYRO1, INPUT);
//  pinMode(intGYRO2, INPUT);
//  pinMode(intMAG1,  INPUT);
//  pinMode(intDRDYM, INPUT);
//
//  pinMode(myLed, OUTPUT);
//  digitalWrite(myLed, HIGH);
//
//  display.begin(); // Initialize the display
//  display.setContrast(58); // Set the contrast
//
//// Start device display with ID of sensor
//  display.clearDisplay();
//  display.setTextSize(2);
//  display.setCursor(0,0); display.print("BMX055");
//  display.setTextSize(1);
//  display.setCursor(0, 20); display.print("9-DOF 16-bit");
//  display.setCursor(0, 30); display.print("motion sensor");
//  display.setCursor(20,40); display.print("1 mg LSB");
//  display.display();
//  delay(1000);
//
//// Set up for data display
//  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
//  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
//  display.clearDisplay();   // clears the screen and buffer

  // Read the BMX-055 WHO_AM_I registers, this is a good test of communication
//  Serial.println("BMX055 accelerometer...");

  uint8_t c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_WHOAMI);  // Read ACC WHO_AM_I register for BMX055

//  Serial.print("BMX055 ACC"); Serial.print(" I AM 0x"); Serial.print(c, HEX); Serial.print(" I should be 0x"); Serial.println(0xFA, HEX);
//  display.setCursor(20,0); display.print("BMX055 ACC");
//  display.setCursor(0,10); display.print("I AM");
//  display.setCursor(0,20); display.print("0x"); display.print(c, HEX);
//  display.setCursor(0,30); display.print("I Should Be");
//  display.setCursor(0,40); display.print("0x"); display.print(0xFA, HEX);
//  display.display();
  HAL_Delay(1000);

//  display.clearDisplay();   // clears the screen and buffer
//  Serial.println("BMX055 gyroscope...");

  uint8_t d = readByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_WHOAMI);  // Read GYRO WHO_AM_I register for BMX055

//  Serial.print("BMX055 GYRO"); Serial.print(" I AM 0x"); Serial.print(d, HEX); Serial.print(" I should be 0x"); Serial.println(0x0F, HEX);
//  display.setCursor(0, 0); display.print("BMX055 GYRO");
//  display.setCursor(0,10); display.print("I AM");
//  display.setCursor(0,20); display.print("0x"); display.print(d, HEX);
//  display.setCursor(0,30); display.print("I Should Be");
//  display.setCursor(0,40); display.print("0x"); display.print(0x0F, HEX);
//  display.display();
  HAL_Delay(1000);

//  Serial.println("BMX055 magnetometer...");

  writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer first thing
  HAL_Delay(100);
  uint8_t e = readByte(BMX055_MAG_ADDRESS, BMX055_MAG_WHOAMI);  // Read MAG WHO_AM_I register for BMX055

//  Serial.print("BMX055 MAG"); Serial.print(" I AM 0x"); Serial.print(e, HEX); Serial.print(" I should be 0x"); Serial.println(0x32, HEX);
//  display.clearDisplay();   // clears the screen and buffer
//  display.setCursor(0, 0); display.print("BMX055 MAG");
//  display.setCursor(0,10); display.print("I AM");
//  display.setCursor(0,20); display.print("0x"); display.print(e, HEX);
//  display.setCursor(0,30); display.print("I Should Be");
//  display.setCursor(0,40); display.print("0x"); display.print(0x32, HEX);
//  display.display();
  HAL_Delay(1000);

  if ((c == 0xFA) && (d == 0x0F) && (e == 0x32)) // WHO_AM_I should always be ACC = 0xFA, GYRO = 0x0F, MAG = 0x32
  {
//  Serial.println("BMX055 is online...");
//  display.clearDisplay();   // clears the screen and buffer
//  display.setCursor(0, 0); display.print("BMX055 online");
//  display.setCursor(0,10); display.print("configuring");
//  display.display();
//  delay(1000);

  initBMX055();

//  Serial.println("BMX055 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
//  display.clearDisplay();   // clears the screen and buffer
//  display.setCursor(0, 0); display.print("BMX055 online");
//  display.setCursor(0,10); display.print("initialized");
//  display.display();
//  delay(1000);

  HAL_Delay(1000);

  // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   // magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
   mRes = 1./1.6;
   trimBMX055();  // read the magnetometer calibration data

//   display.clearDisplay();
//   display.setCursor(0, 0); display.print("BMX055 Res");
//   display.setCursor(0,10); display.print("ACC ");  display.setCursor(50,10); display.print(1000.*aRes, 2);
//   display.setCursor(0,20); display.print("GYRO "); display.setCursor(50,20); display.print(1000.*gRes, 2);
////   display.setCursor(0,30); display.print("MAG ");  display.setCursor(50,30); display.print((int)dig_x1);
//   display.display();
   HAL_Delay(1000);


  fastcompaccelBMX055(accelBias);
//  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
//  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

  magcalBMX055(magBias);
//  Serial.println("mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);

  return true;
  }
  else
  {
//    Serial.print("Could not connect to BMX055: 0x");
//    Serial.println(c, HEX);

    //while(1) ; // Loop forever if communication doesn't happen
	  return false;
  }
}

void loop()
{
  // If intPin goes high, all data registers have new data
//  if (digitalRead(intACC2)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes; // + accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes; // + accelBias[1];
    az = (float)accelCount[2]*aRes; // + accelBias[2];
 // }
//  if (digitalRead(intGYRO2)) {  // On interrupt, read data
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;
    gz = (float)gyroCount[2]*gRes;
 // }
 // if (digitalRead(intDRDYM)) {  // On interrupt, read data
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    mx = (float)magCount[0]*mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magBias[1];
    mz = (float)magCount[2]*mRes - magBias[2];
 //}

//  Now = micros();
//  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
//  lastUpdate = Now;
//
//  sum += deltat; // sum for averaging filter update rate
//  sumCount++;
//
//  // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
//  // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
//  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
//  // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
//  // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
//  // This is ok by aircraft orientation standards!
//  // Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  -my,  mx, mz);
////  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz);
//
//    // Serial print and/or display at 0.5 s rate independent of data rates
//    delt_t = millis() - count;
//    if (delt_t > 500) { // update LCD once per half-second independent of read rate
//
////    if(SerialDebug) {
////    Serial.print("ax = "); Serial.print((int)1000*ax);
////    Serial.print(" ay = "); Serial.print((int)1000*ay);
////    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
////    Serial.print("gx = "); Serial.print( gx, 2);
////    Serial.print(" gy = "); Serial.print( gy, 2);
////    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
////    Serial.print("mx = "); Serial.print( (int)mx);
////    Serial.print(" my = "); Serial.print( (int)my);
////    Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");
////
////    Serial.print("q0 = "); Serial.print(q[0]);
////    Serial.print(" qx = "); Serial.print(q[1]);
////    Serial.print(" qy = "); Serial.print(q[2]);
////    Serial.print(" qz = "); Serial.println(q[3]);
////    }
//
//
//  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
//  // In this coordinate system, the positive z-axis is down toward Earth.
//  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
//  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
//  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
//  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
//  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
//  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
//  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
//    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
//    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//    pitch *= 180.0f / PI;
//    yaw   *= 180.0f / PI;
//    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//    roll  *= 180.0f / PI;
//
//    // Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis
//    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
//    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
//    // points toward the right of the device.
//    //
//
////    if(SerialDebug) {
////    Serial.print("Yaw, Pitch, Roll: ");
////    Serial.print(yaw, 2);
////    Serial.print(", ");
////    Serial.print(pitch, 2);
////    Serial.print(", ");
////    Serial.println(roll, 2);
////
////    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
////    }
//
////    display.clearDisplay();
////
////    display.setCursor(0, 0); display.print(" x   y   z ");
////
////    display.setCursor(0,  8); display.print((int)(1000*ax));
////    display.setCursor(24, 8); display.print((int)(1000*ay));
////    display.setCursor(48, 8); display.print((int)(1000*az));
////    display.setCursor(72, 8); display.print("mg");
//
//    tempCount = readACCTempData();  // Read the gyro adc values
//    temperature = ((float) tempCount) / 2.0 + 23.0; // Gyro chip temperature in degrees Centigrade
////    display.setCursor(64, 0); display.print(9.*temperature/5. + 32., 0); display.print("F");
////
////    display.setCursor(0,  16); display.print((int)(gx));
////    display.setCursor(24, 16); display.print((int)(gy));
////    display.setCursor(48, 16); display.print((int)(gz));
////    display.setCursor(66, 16); display.print("o/s");
////
////    display.setCursor(0,  24); display.print((int)(mx));
////    display.setCursor(24, 24); display.print((int)(my));
////    display.setCursor(48, 24); display.print((int)(mz));
////    display.setCursor(72, 24); display.print("mG");
////
////    display.setCursor(0,  32); display.print((int)(yaw));
////    display.setCursor(24, 32); display.print((int)(pitch));
////    display.setCursor(48, 32); display.print((int)(roll));
////    display.setCursor(66, 32); display.print("ypr");
//
//    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
//    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
//    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
//    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
//    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
//    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
//    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
//    // This filter update rate should be fast enough to maintain accurate platform orientation for
//    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
//    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
//    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
////    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft");
////    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0);
////    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz");
////    display.display();
////
////    digitalWrite(myLed, !digitalRead(myLed));
//    count = millis();
//    sumCount = 0;
//    sum = 0;
//    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
    case GFS_125DPS:
          gRes = 124.87/32768.0; // per data sheet, not exactly 125!?
          break;
    case GFS_250DPS:
          gRes = 249.75/32768.0;
          break;
    case GFS_500DPS:
          gRes = 499.5/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 999.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 1998.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
        // BMX055 ACC data is signed 12 bit
    case AFS_2G:
          aRes = 2.0/2048.0;
          break;
    case AFS_4G:
          aRes = 4.0/2048.0;
          break;
    case AFS_8G:
          aRes = 8.0/2048.0;
          break;
    case AFS_16G:
          aRes = 16.0/2048.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array
  if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) {  // Check that all 3 axes have new data
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
  }
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readMagData(int16_t * magData)
{
  int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
  uint16_t data_r = 0;
  uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
  readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
    if(rawData[6] & 0x01) { // Check if data ready status bit is set
    mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
    mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
    mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
    data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance

   // calculate temperature compensated 16-bit magnetic fields
   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
   magData[0] = ((int16_t)((((int32_t)mdata_x) *
				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
			     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
			   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
			(((int16_t)dig_x1) << 3);

   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
   magData[1] = ((int16_t)((((int32_t)mdata_y) *
				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
			     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
		           ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
			(((int16_t)dig_y1) << 3);
   magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
	((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
    }
  }

int16_t readACCTempData()
{
  uint8_t c =  readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
  return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
}

void trimBMX055()  // get trim values for magnetometer sensitivity
{
  uint8_t rawData[2];  //placeholder for 2-byte trim data
  dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
  dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
  dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
  dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
  dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
  dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
  dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
  dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
  dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
  dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
  dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
}


void initBMX055()
{
   // start with all sensors in default mode with all registers reset
   writeByte(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
   HAL_Delay(1000); // Wait for all registers to reset

   // Configure accelerometer
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_EN_1, 0x10);           // Enable ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_OUT_CTRL, 0x04);       // Set interrupts push-pull, active high for INT1 and INT2
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x02);        // Define INT1 (intACC1) as ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x80);          // Define INT2 (intACC2) as ACC data ready interrupt

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_BGW_SPI3_WDT, 0x06);       // Set watchdog timer for 50 ms

 // Configure Gyro
 // start by resetting gyro, better not since it ends up in sleep mode?!
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
// delay(100);
 // Three power modes, 0x00 Normal,
 // set bit 7 to 1 for suspend mode, set bit 5 to 1 for deep suspend mode
 // sleep duration in fast-power up from suspend mode is set by bits 1 - 3
 // 000 for 2 ms, 111 for 20 ms, etc.
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x00);  // set GYRO normal mode
//  set GYRO sleep duration for fast power-up mode to 20 ms, for duty cycle of 50%
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM1, 0x0E);
 // set bit 7 to 1 for fast-power-up mode,  gyro goes quickly to normal mode upon wake up
// can set external wake-up interrupts on bits 5 and 4
// auto-sleep wake duration set in bits 2-0, 001 4 ms, 111 40 ms
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x00);  // set GYRO normal mode
// set gyro to fast wake up mode, will sleep for 20 ms then run normally for 20 ms
// and collect data for an effective ODR of 50 Hz, other duty cycles are possible but there
// is a minimum wake duration determined by the bandwidth duration, e.g.,  > 10 ms for 23Hz gyro bandwidth
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM2, 0x87);

 writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
 writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_0, 0x80);  // enable data ready interrupt
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_1, 0x04);  // select push-pull, active high interrupts
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_MAP_1, 0x80); // select INT3 (intGYRO1) as GYRO data ready interrupt

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SPI3_WDT, 0x06); // Enable watchdog timer for I2C with 50 ms window


// Configure magnetometer
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
HAL_Delay(100);
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
HAL_Delay(100);

writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode
//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3 | 0x02); // Forced mode

//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_INT_EN_2, 0x84); // Enable data ready pin interrupt, active high

// Set up four standard configurations for the magnetometer
  switch (Mmode)
  {
    case lowPower:
         // Low-power
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
          break;
    case Regular:
          // Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
          break;
    case enhancedRegular:
          // Enhanced Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
          break;
    case highAccuracy:
          // High Accuracy
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
          break;
  }
}

void fastcompaccelBMX055(float * dest1)
{
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

  uint8_t c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}

  int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
  int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
  int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

  dest1[0] = (float) compx/128.; // accleration bias in g
  dest1[1] = (float) compy/128.; // accleration bias in g
  dest1[2] = (float) compz/128.; // accleration bias in g
}


void magcalBMX055(float * dest1)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

//  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  HAL_Delay(4000);

   sample_count = 128;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readMagData(mag_temp);
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    HAL_Delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;
    dest1[2] = (float) mag_bias[2]*mRes;

 /* //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 */
//   Serial.println("Mag Calibration done!");
}

// I2C communication with the MS5637 is a little different from that with the MPU9250 and most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers






// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
//	Wire.beginTransmission(address);  // Initialize the Tx buffer
//	Wire.write(subAddress);           // Put slave register address in Tx buffer
//	Wire.write(data);                 // Put data in Tx buffer
//	Wire.endTransmission();           // Send the Tx buffer


	uint8_t buffer[2];
	buffer[0]=subAddress;
	buffer[1]=data;

	HAL_I2C_Master_Transmit(&cur_i2c, address<<1, buffer, 2, 10);
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
//	uint8_t data; // `data` will store the register data
//	Wire.beginTransmission(address);         // Initialize the Tx buffer
//	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
//	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
////	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
////	Wire.requestFrom(address, 1);  // Read one byte from slave register address
//	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
//	data = Wire.read();                      // Fill Rx buffer with result
//	return data;                             // Return data read from slave register

	uint8_t cmd_size = 1;
	uint8_t count = 1;
	uint8_t data = 0;
//	HAL_I2C_Mem_Read(&cur_i2c, address<< 1, subAddress, cmd_size, &data, count, 10);
//	return data;

uint16_t addr = address;

	HAL_I2C_Master_Transmit	(&cur_i2c, addr << 1, &subAddress, cmd_size, 10);

	HAL_I2C_Master_Receive (&cur_i2c, addr<< 1 | 1, &data, count, 10);

	return data;

}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
//	Wire.beginTransmission(address);   // Initialize the Tx buffer
//	Wire.write(subAddress);            // Put slave register address in Tx buffer
//	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
////	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
//	uint8_t i = 0;
////        Wire.requestFrom(address, count);  // Read bytes from slave register address
//        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
//	while (Wire.available()) {
//        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer

	uint8_t cmd_size = 1;
	HAL_I2C_Mem_Read(&cur_i2c, address<< 1, subAddress, cmd_size, dest, count, 10);
}




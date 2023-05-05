
#ifndef __BMX055__
#define __BMX055__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

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
////#include "Wire.h"
//#include <i2c_t3.h>
//#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

//// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet http://www.meas-spec.com/downloads/MS5637-02BA03.pdf
//#define MS5637_RESET      0x1E
//#define NS5637_CONVERT_D1 0x40
//#define NS5637_CONVERT_D2 0x50
//#define MS5637_ADC_READ   0x00

// BMX055 data sheet http://ae-bst.resource.bosch.com/media/products/dokumente/bmx055/BST-BMX055-DS000-01v2.pdf
// The BMX055 is a conglomeration of three separate motion sensors packaged together but
// addressed and communicated with separately by design
// Accelerometer registers
#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64
#define BMM050_DIG_Y2             0x65
#define BMM050_DIG_Z2_LSB         0x68
#define BMM050_DIG_Z2_MSB         0x69
#define BMM050_DIG_Z1_LSB         0x6A
#define BMM050_DIG_Z1_MSB         0x6B
#define BMM050_DIG_XYZ1_LSB       0x6C
#define BMM050_DIG_XYZ1_MSB       0x6D
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70
#define BMM050_DIG_XY1            0x71

// Using the Teensy Mini Add-On board, SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
#define BMX055_ACC_ADDRESS  0x18   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x68   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x10   // Address of BMX055 magnetometer


//#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
// define X055 ACC full scale options
#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

enum ACCBW {    // define BMX055 accelerometer bandwidths
  ABW_8Hz,      // 7.81 Hz,  64 ms update time
  ABW_16Hz,     // 15.63 Hz, 32 ms update time
  ABW_31Hz,     // 31.25 Hz, 16 ms update time
  ABW_63Hz,     // 62.5  Hz,  8 ms update time
  ABW_125Hz,    // 125   Hz,  4 ms update time
  ABW_250Hz,    // 250   Hz,  2 ms update time
  ABW_500Hz,    // 500   Hz,  1 ms update time
  ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

enum GscaleType {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODRBW {
  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
  G_2000Hz230Hz,
  G_1000Hz116Hz,
  G_400Hz47Hz,
  G_200Hz23Hz,
  G_100Hz12Hz,
  G_200Hz64Hz,
  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
  MODR_10Hz = 0,   // 10 Hz ODR
  MODR_2Hz     ,   // 2 Hz ODR
  MODR_6Hz     ,   // 6 Hz ODR
  MODR_8Hz     ,   // 8 Hz ODR
  MODR_15Hz    ,   // 15 Hz ODR
  MODR_20Hz    ,   // 20 Hz ODR
  MODR_25Hz    ,   // 25 Hz ODR
  MODR_30Hz        // 30 Hz ODR
};

enum Mmode {
  lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
  Regular             ,   // rms noise ~0.6 microTesla, 0.5 mA power
  enhancedRegular     ,   // rms noise ~0.5 microTesla, 0.8 mA power
  highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
};

//// MS5637 pressure sensor sample rates
//#define ADC_256  0x00 // define pressure and temperature conversion rates
//#define ADC_512  0x02
//#define ADC_1024 0x04
//#define ADC_2048 0x06
//#define ADC_4096 0x08
//#define ADC_8192 0x0A
//#define ADC_D1   0x40
//#define ADC_D2   0x50



// Specify sensor full scale
//uint8_t OSR    = ADC_8192;         // set pressure amd temperature oversample rate
extern uint8_t Gscale;// = GFS_125DPS;       // set gyro full scale
extern uint8_t GODRBW;// = G_200Hz23Hz;      // set gyro ODR and bandwidth
extern uint8_t Ascale;// = AFS_2G;           // set accel full scale
extern uint8_t ACCBW;//  = 0x08 | ABW_16Hz;  // Choose bandwidth for accelerometer, need bit 3 = 1 to enable bandwidth choice in enum
extern uint8_t Mmode;//  = Regular;          // Choose magnetometer operation mode
extern uint8_t MODR;//   = MODR_10Hz;        // set magnetometer data rate
extern float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
extern signed char   dig_x1;
extern signed char   dig_y1;
extern signed char   dig_x2;
extern signed char   dig_y2;
extern uint16_t      dig_z1;
extern int16_t       dig_z2;
extern int16_t       dig_z3;
extern int16_t       dig_z4;
extern unsigned char dig_xy1;
extern signed char   dig_xy2;
extern uint16_t      dig_xyz1;

// Pin definitions
// The BMX055 has three sensors, and two interrupt pins per device!
extern int intACC1;//   =  8;  // These are fixed on the BMX055 Mini Add-On for Teensy 3.1
extern int intACC2;//   =  9;
extern int intGYRO1;//  = 11;
extern int intGYRO2;//  = 10;
extern int intMAG1;//   = 12;
extern int intDRDYM;//  = 15;
extern int myLed;//     = 13;  // LED on the Teensy 3.1

// BMX055 variables
extern int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
extern int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
extern int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
extern float gyroBias[3], accelBias[3], magBias[3];  // Bias corrections for gyro, accelerometer, mag
extern int16_t tempCount;            // temperature raw count output
extern float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
extern float SelfTest[6];            // holds results of gyro and accelerometer self test



//
//// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
//float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//// There is a tradeoff in the beta parameter between accuracy and response speed.
//// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
//// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
//// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
//// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
//// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
//// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
//// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
//#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
//#define Ki 0.0f
//
//uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
//float pitch, yaw, roll;
//float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
//uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
//uint32_t Now = 0;                         // used to calculate integration interval
//
//float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
//float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void BMX055_init_globals();
bool BMX055_setup();


void loop();

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getGres();

void getAres();


void readAccelData(int16_t * destination);


void readGyroData(int16_t * destination);

void readMagData(int16_t * magData);

int16_t readACCTempData();

void trimBMX055();  // get trim values for magnetometer sensitivity

void initBMX055();

void fastcompaccelBMX055(float * dest1);


void magcalBMX055(float * dest1);






// I2C read/write functions for the MPU9250 and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

uint8_t readByte(uint8_t address, uint8_t subAddress);

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

#ifdef __cplusplus
}
#endif

#endif /*BMX055_H_*/

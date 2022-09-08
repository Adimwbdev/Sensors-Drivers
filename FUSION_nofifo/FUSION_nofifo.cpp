  Author: A. Dimitrakopoulos
  Date: 2019
  
//	This is a library for use with the SparkFun LSM6DS3 intertial module 
//	in conjunction with the SparkFun MAG3110 Magnetometer for use with an oil-spill Drifter
//	in order to measure waves.
//      In this library internal functions are used to fuse the two sensors (without using the Buffer)

#include "FUSION_nofifo.h"
#include "stdint.h"
#include "Wire.h"
#include "SparkFun_MAG3110.h"

#define   SENSORS_GRAVITY_STANDARD  9.80665F



// This function sets the I2C connection


void LSM6DS3::beginSens(void)
{

Wire.begin();
//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

}


// This function reads 8-bit registers


thereturn_t LSM6DS3::readRegister(uint8_t sens_addr,uint8_t addr, uint8_t* dabuff, int nob)


{

thereturn_t caution = ERROR;
Wire.beginTransmission(sens_addr);
Wire.write(addr); //Send the Register Address
Wire.endTransmission();
// //End the communication sequence.
Wire.beginTransmission(sens_addr);
Wire.requestFrom(sens_addr, nob);
//// Ask the device for data
if (Wire.available())
{
////  Wait for a response from device
*dabuff = Wire.read();
caution = ALLGOOD;
}


return caution;

}

// This function writes 8-bit registers


thereturn_t LSM6DS3::writeRegister(uint8_t sens_addr, uint8_t addr, uint8_t wtr)


{

thereturn_t caution = ALLGOOD;
Wire.beginTransmission(sens_addr);
 //initiate the send sequence
Wire.write(addr);
 //the register address to write
Wire.write(wtr);
 //the data to be written
if( Wire.endTransmission() != 0) {

caution = ERROR;
}

return caution;

}


//// This function reads 16-bit registers (it composes 16-bit from two 8-bit data)


thereturn_t LSM6DS3::readRegister16(uint8_t sens_addr,uint8_t addr, uint16_t* dabuff, int nob)


{

nob = 2;
uint8_t lsb;
uint8_t msb;
thereturn_t caution = ERROR;
Wire.beginTransmission(sens_addr);
Wire.write(addr); //Send the Register Address
Wire.endTransmission();
// //End the communication sequence.
Wire.beginTransmission(sens_addr);
Wire.requestFrom(sens_addr, nob);
//// Ask the device for data
if (Wire.available())
{
////  Wait for a response from device
lsb = Wire.read();
msb = Wire.read();
caution = ALLGOOD;

}
*dabuff = (lsb|(msb<<8));
}


// This function resets the Sensor


thereturn_t LSM6DS3::reset(void)


{

thereturn_t caution;
uint8_t temp;
writeRegister(ACCEL_ADDR, CTRL2_G, 0x00);
writeRegister(ACCEL_ADDR, CTRL6_C, 0x10);
writeRegister(ACCEL_ADDR, CTRL3_C, 0x05);
readRegister(ACCEL_ADDR, CTRL3_C, &temp, 1);
while (temp != 0x04)
{
caution = ERROR;
}
caution = ALLGOOD;

return caution;

}


// This function sets the scaling factors of the internal Sensors (accelerometer and the gyro)


thereturn_t LSM6DS3::scaleset(uint8_t* buff)                // Draft


{

thereturn_t caution;

uint8_t dabuff;
//Accel Scale
readRegister(ACCEL_ADDR, CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff|=0x00); //FS +- 2
readRegister(ACCEL_ADDR, CTRL1_XL, buff, 1);

//Gyro Scale

readRegister(ACCEL_ADDR, CTRL2_G, &dabuff, 1);
caution = writeRegister(ACCEL_ADDR, CTRL2_G, dabuff|=0x00); //FS +- 250
readRegister(ACCEL_ADDR, CTRL2_G, buff, 1);

return caution;

}


// This function calculates the 16-bit raw data of the accelerometer


axis3bit16_t LSM6DS3::accel_data(void)


{

axis3bit16_t accel;
uint16_t dabuff;
readRegister16(ACCEL_ADDR, OUTX_L_XL, &dabuff, 2);
accel.i16bit[0] = dabuff;
readRegister16(ACCEL_ADDR, OUTY_L_XL, &dabuff, 2);
accel.i16bit[1] = dabuff;
readRegister16(ACCEL_ADDR, OUTZ_L_XL, &dabuff, 2);
accel.i16bit[2] = dabuff;

return accel;

}


// This function calculates the SI accelerometer data from the raw data


float LSM6DS3::calcSI( int16_t input)


{

uint16_t accelRange = 2;


float output = (float)input * 0.061 * (accelRange >> 1)*GRAVITY/1000;

return output;

}


// This function calculates the 16-bit raw data of the gyro


axis3bit16_t LSM6DS3::gyro_data(void)


{

axis3bit16_t gyro;
uint16_t dabuff;

readRegister16(ACCEL_ADDR, OUTX_L_G, &dabuff, 2);
gyro.i16bit[0] = dabuff;
readRegister16(ACCEL_ADDR, OUTY_L_G, &dabuff, 2);
gyro.i16bit[1] = dabuff;
readRegister16(ACCEL_ADDR, OUTZ_L_G, &dabuff, 2);
gyro.i16bit[2] = dabuff;

return gyro;

}


// This function calculates the SI gyro data from the raw data


float LSM6DS3::calcGyro( int16_t input )


{

uint8_t gyroRangeDivisor;
//	uint8_t gyroRangeDivisor = settings.gyroRange / 125;
//	if ( settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
//	}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;  //Gia FS = 250

return output;

}



//*From this point and down all the Functions Consern The external HUB Functionality
    



// This function enables the pull-up resistors for the external sensors


thereturn_t THEHUB::pullup_en(void)


{

thereturn_t caution;
uint8_t buff;
caution = ALLGOOD;

readRegister(ACCEL_ADDR, MASTER_CONFIG, &buff, 1);
if(writeRegister(ACCEL_ADDR, MASTER_CONFIG, buff|=0x08)!=1)
caution = ERROR;

return caution;

}


// This function disables the pull-up resistors for the external sensors


thereturn_t THEHUB::pullup_dis(void)


{

thereturn_t caution;
uint8_t buff;
caution = ALLGOOD;

readRegister(ACCEL_ADDR, MASTER_CONFIG, &buff, 1);
if(writeRegister(ACCEL_ADDR, MASTER_CONFIG, buff&=~(0x08))!=1)
caution = ERROR;

return caution;

}


// This function enables the passthrough feature so that you can intialize the aux sensors directly


thereturn_t THEHUB::passthrough(uint8_t* buff)


{

thereturn_t caution;
caution = ERROR;

//readRegister(ACCEL_ADDR, WHO_AM_I, &buff, 1);
//writeRegister(ACCEL_ADDR, CTRL1_XL, 0x43);
//
//caution = readRegister(ACCEL_ADDR, CTRL1_XL, &buff, 1);
pullup_en();
writeRegister(ACCEL_ADDR, MASTER_CONFIG, 0x04); //Enable the passthrough bit in Master_Config
writeRegister(MAG_ADDR, MAG3110_CTRL_REG1, 0x01); //Setting up the 2nd Accelerometer
readRegister(MAG_ADDR, MAG3110_WHO_AM_I, buff, 1);
caution = writeRegister(ACCEL_ADDR, MASTER_CONFIG, 0x00); //Disable the passthrough bit in Master_Config

return caution;

}


// This function sets aux-sensors the old-fashioned way


thereturn_t THEHUB::extrn_conf(uint8_t* buff, uint8_t* check)


{

thereturn_t caution;
thereturn_t caution1;
int count = 0;
uint8_t dabuff;
uint8_t drdy;
uint8_t endop;
axis3bit16_t data_raw_acceleration;
caution1 = ERROR;

//Disable accelerometer
readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff&=~(0xF0)); //Disabling ODR-> disabling accel
readRegister(ACCEL_ADDR,CTRL1_XL, buff, 1);

//Enable external Functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff|=0x80); //Enabling the embedded functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, buff, 1);


//Setting the Address of the slave;
writeRegister(ACCEL_ADDR, SLV0_ADD, MAG_ADDR<<1); //Setting the address of the slave Magnetometer (0x0E) with the rw_0 bit ->0 (write operation)
readRegister(ACCEL_ADDR,SLV0_ADD, buff, 1);


//Setting the register of the slave to be modified
writeRegister(ACCEL_ADDR, SLV0_SUBADD, MAG3110_CTRL_REG1); //Selecting the CTRL_REG1  of the Magnetometer to modify
readRegister(ACCEL_ADDR,SLV0_SUBADD, buff, 1);


//Writing data to the slave (LSM303DLHC)
writeRegister(ACCEL_ADDR, DATAWRITE_SRC_MODE_SUB_SLV0, 0x01); //Put magnetometer in active mode 80 Hz OSR = 1

//Disable external Functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
caution = writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff&=~(1<<7));
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, buff, 1);

//Enable I2C Master
readRegister(ACCEL_ADDR,CTRL10_C, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL10_C, dabuff|=0x04); //Enable embedded functionalities
readRegister(ACCEL_ADDR,CTRL10_C, buff, 1);

//Sensor Hub Master Enable
readRegister(ACCEL_ADDR,MASTER_CONFIG, &dabuff, 1);
writeRegister(ACCEL_ADDR, MASTER_CONFIG, dabuff|=0x01); //Enable embedded functionalities
readRegister(ACCEL_ADDR,MASTER_CONFIG, buff, 1);


//Enable pull-up
pullup_en();

//Enable accelerometer to trigger Sensor Hub operation
readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff|=0x43); //Enable Sensor with ODR->104Hz
readRegister(ACCEL_ADDR,CTRL1_XL, buff, 1);

  /*
   * Wait Sensor Hub operation flag set
   */
readRegister(ACCEL_ADDR, OUTX_L_XL, data_raw_acceleration.u8bit, 6);
do {
 readRegister(ACCEL_ADDR, STATUS_REG, &drdy, 1);
count = count + 1;
  } while (!drdy&(1<<0));

  do
  {
  readRegister(ACCEL_ADDR, FUNC_SRC, &endop, 1);
  } while (!endop&(1<<0));
*check = endop;

//  Disable accelerometer
  readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
  writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff&=~(0xF0)); //Disabling ODR-> disabling accel


  //Sensor Hub Master Disable
  readRegister(ACCEL_ADDR,MASTER_CONFIG, &dabuff, 1);
  writeRegister(ACCEL_ADDR, MASTER_CONFIG, dabuff&=0x00); //Disable embedded functionalities
  readRegister(ACCEL_ADDR,MASTER_CONFIG, buff, 1);

  //Disable pull-up
  pullup_dis();


caution1 = ALLGOOD;

return caution1;

}


// This function reads sets up the sensor so that he can read data from the aux sensors


thereturn_t THEHUB::extrn_read(uint8_t* buff)


{
uint8_t dabuff;
thereturn_t caution;
caution = ERROR;


//Enable external Functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff|=0x80); //Enabling the embedded functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, buff, 1);

//Setting the Address of the slave;
writeRegister(ACCEL_ADDR, SLV0_ADD, (MAG_ADDR<<1)|0x01); //Setting the address of the slave Magnetometer (0x0E) with the rw_0 bit ->1 (read operation)
readRegister(ACCEL_ADDR,SLV0_ADD, buff, 1);

//Setting the register of the slave to be read
writeRegister(ACCEL_ADDR, SLV0_SUBADD, MAG3110_OUT_X_MSB); //Selecting the   OUT_X_MSB  of the Magnetometer to read continiously
readRegister(ACCEL_ADDR,SLV0_SUBADD, buff, 1);

//Setting the total number of continious operations so that the whole accelerometer would be read
//Also setting up the number of auxiliary sensors that are connected to the hub --> default value 00
readRegister(ACCEL_ADDR, SLAVE0_CONFIG, &dabuff, 1);
writeRegister(ACCEL_ADDR, SLAVE0_CONFIG, dabuff|=0x06); // 6-operations because-->  2-bytes for every axis, one auxiliary sensor

//Disable external Functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff&=~(1<<7));
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, buff, 1);

//Enable I2C Master
readRegister(ACCEL_ADDR,CTRL10_C, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL10_C, dabuff|=0x04); //Enable embedded functionalities
readRegister(ACCEL_ADDR,CTRL10_C, buff, 1);

//Sensor Hub Master Enable
readRegister(ACCEL_ADDR,MASTER_CONFIG, &dabuff, 1);     // Na dokimasw an einai aparaititi i energopoisi tis pull-up
writeRegister(ACCEL_ADDR, MASTER_CONFIG, dabuff|=0x01); //Enable embedded functionalities
readRegister(ACCEL_ADDR,MASTER_CONFIG, buff, 1);


//Enable pull-up
pullup_en();

//Enable accelerometer to trigger Sensor Hub operation
readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff|=0x43); //Enable Accel with ODR->104Hz
readRegister(ACCEL_ADDR,CTRL1_XL, buff, 1);

//Enable Gyro
readRegister(ACCEL_ADDR, CTRL2_G,  &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL2_G, dabuff|=0x42); //Enable Gyro with ODR->104Hz, FS->125 dps


caution = ALLGOOD;

return caution;

}


// This function reads the raw data from the aux sensor
//(in this case the magnetometer)

axis3bit16_t THEHUB::mag_data(void)


{
axis3bit16_t mag;
uint16_t dabuff;

readRegister16(ACCEL_ADDR, SENSORHUB1_REG, &dabuff, 2);
mag.i16bit[0] = dabuff;
readRegister16(ACCEL_ADDR, SENSORHUB3_REG, &dabuff, 2);
mag.i16bit[1] = dabuff;
readRegister16(ACCEL_ADDR, SENSORHUB5_REG, &dabuff, 2);
mag.i16bit[2] = dabuff;


return mag;

}











  
//	This is a library for use with the SparkFun LSM6DS3 intertial module 
//	in conjunction with the SparkFun MAG3110 Magnetometer for use with an oil-spill Drifter
//	in order to measure waves.
//      In this library internal functions and FIFO buffer are used to fuse the two sensors

#include "HubDub.h"
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

thereturn_t LSM6DS3::BDU(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, CTRL3_C, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL3_C, dabuff|=0x40);

return caution;

}




// This function sets the scaling factors of the internal Sensors (accelerometer and the gyro)


thereturn_t LSM6DS3::accel_scaleset(uint8_t* buff)


{

thereturn_t caution;

uint8_t dabuff;
//Accel Scale
readRegister(ACCEL_ADDR, CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff|=0x00); //FS +- 2
readRegister(ACCEL_ADDR, CTRL1_XL, buff, 1);

caution = ALLGOOD;

return caution;

}


thereturn_t LSM6DS3::gyro_scaleset(uint8_t* buff)


{

thereturn_t caution;

uint8_t dabuff;

//Gyro Scale

readRegister(ACCEL_ADDR, CTRL2_G, &dabuff, 1);
caution = writeRegister(ACCEL_ADDR, CTRL2_G, dabuff|=0x02); //FS +- 125
readRegister(ACCEL_ADDR, CTRL2_G, buff, 1);

caution = ALLGOOD;

return caution;

}


thereturn_t LSM6DS3::accel_ODRset(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;
//Set accelerometer ODR and bandwidth
readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff|=0x13); //Enable Accel with ODR->13Hz bandwidth -->50Hz
readRegister(ACCEL_ADDR,CTRL1_XL, buff, 1);

return caution;

}

thereturn_t LSM6DS3::gyro_ODRset(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

//Set Gyro ODR
readRegister(ACCEL_ADDR, CTRL2_G,  &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL2_G, dabuff|=0x12); //Enable Gyro with ODR->13Hz, FS->125dps
readRegister(ACCEL_ADDR, CTRL2_G,  buff, 1);

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
//	if ( gyroRange == 245 ) {
//		gyroRangeDivisor = 2;
//	}
// if Gyrorange == 125
// gyroRangeDivisor = 1;
gyroRangeDivisor = 1;
	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;  //Gia FS = 125

return output;

}

//This function disables the Accelerometer


thereturn_t LSM6DS3::accel_dis(void)
{
thereturn_t caution;
uint8_t dabuff;
readRegister(ACCEL_ADDR,CTRL1_XL, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL1_XL, dabuff&=~(0xF0)); //Disabling ODR-> disabling accel

caution = ALLGOOD;

return caution;
}


//*From this point and down all the Functions Concern The external HUB Functionality
 


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
writeRegister(MAG_ADDR, MAG3110_CTRL_REG1, 0xF9); //Setting up magnetometer ODR --> 10Hz OS-->128
readRegister(MAG_ADDR, MAG3110_CTRL_REG1 , buff, 1);
writeRegister(ACCEL_ADDR, MASTER_CONFIG, 0x00); //Disable the passthrough bit in Master_Config
//pullup_dis();
return caution;

}


// This function sets aux-sensors the old-fashioned way


thereturn_t THEHUB::extrn_conf(uint8_t* buff, uint8_t* check)


{

thereturn_t caution;
int count = 0;
uint8_t dabuff;
uint8_t drdy;
uint8_t endop;
uint8_t check1;
//*check = dabuff;
axis3bit16_t data_raw_acceleration;

//Disable accelerometer
accel_dis();

//Enable external Functions
func_cfg_en(&dabuff);

//Setting the Address of the slave;
slv_0add(MAG_ADDR, 0); // 0:for writing,  1:for reading

//Setting the register of the slave to be modified
slv_0subadd(MAG3110_CTRL_REG1); //CTRL_REG1 of the magnetometer

//Writing data to the slave (MAG3110)
slv_wrt(0x18); //Setting up magnetometer ODR --> 10Hz OS-->128

//Disable external Functions
func_cfg_dis();

//Enable I2C Master
i2c_mast_en(&dabuff);

//Sensor Hub Master Enable
hub_mast_en();

//Enable pull-up
pullup_en();

//Enable accelerometer to trigger Sensor Hub operation
accel_ODRset(&dabuff); //By setting the odr we enable the sensors

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

//Disable accelerometer
accel_dis();


//Sensor Hub Master Disable
hub_mast_dis();

//Disable pull-up
pullup_dis();


caution = ALLGOOD;

return caution;

}


// This function reads sets up the sensor so that he can read data from the aux sensors


thereturn_t THEHUB::extrn_read(uint8_t* buff)


{
uint8_t dabuff;
thereturn_t caution;
caution = ERROR;
uint8_t check;

//Enable external Functions
func_cfg_en(&dabuff);

//Setting the Address of the slave;
slv_0add(MAG_ADDR, 1); //Slave MAG3110 with read enabled rw_0 -->1

//Setting the register of the slave to be read
slv_0subadd(MAG3110_OUT_X_MSB);

//Setting the total number of continious operations so that the whole accelerometer would be read
//Also setting up the number of auxiliary sensors that are connected to the hub --> default value 00
slv_0conf(&dabuff);

//Disable external Functions
func_cfg_dis();

//Enable I2C Master
i2c_mast_en(&dabuff);

//Sensor Hub Master Enable
hub_mast_en();

//Enable pull-up
pullup_en();

//Enable accelerometer to trigger Sensor Hub operation
accel_ODRset(&dabuff); //Enable Accel with ODR->13Hz bandwidth -->50Hz

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


thereturn_t THEHUB::func_cfg_en(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

//Enable external Functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff|=0x80); //Enabling the embedded functions
readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, buff, 1);

caution = ALLGOOD;

return caution;

}

thereturn_t THEHUB::func_cfg_dis(void)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR,FUNC_CFG_ACCESS, &dabuff, 1);
writeRegister(ACCEL_ADDR, FUNC_CFG_ACCESS, dabuff&=~(1<<7));

caution = ALLGOOD;

return caution;

}


thereturn_t THEHUB::i2c_mast_en(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR,CTRL10_C, &dabuff, 1);
writeRegister(ACCEL_ADDR, CTRL10_C, dabuff|=0x04); //Enable embedded functionalities
readRegister(ACCEL_ADDR,CTRL10_C, buff, 1);

caution = ALLGOOD;

return caution;

}

thereturn_t THEHUB::hub_mast_en(void)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR,MASTER_CONFIG, &dabuff, 1);
writeRegister(ACCEL_ADDR, MASTER_CONFIG, dabuff|=0x01); //Enable embedded functionalities

caution = ALLGOOD;

return caution;

}

thereturn_t THEHUB::hub_mast_dis(void)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR,MASTER_CONFIG, &dabuff, 1);
writeRegister(ACCEL_ADDR, MASTER_CONFIG, dabuff&=0x00); //Disable embedded functionalities

return caution;

}


thereturn_t THEHUB::slv_0add(uint8_t addr, int rw_0)

{

thereturn_t caution;

//Setting the Address of the slave for writing;
if (rw_0 == 0)
{
writeRegister(ACCEL_ADDR, SLV0_ADD, addr<<1); //Setting the address of the slave Magnetometer (0x0E) with the rw_0 bit ->0 (write operation)
}

//Setting the Address of the slave for reading;
else if (rw_0 == 1)
{
addr = addr<<1;
writeRegister(ACCEL_ADDR, SLV0_ADD, addr|=0x01); //Setting the address of the slave Magnetometer (0x0E) with the rw_0 bit ->0 (read operation)
}

caution = ALLGOOD;

return caution;

}

thereturn_t THEHUB::slv_0subadd(uint8_t addr)
{
thereturn_t caution;
writeRegister(ACCEL_ADDR, SLV0_SUBADD, addr);

return caution;

}

thereturn_t THEHUB::slv_wrt(uint8_t wtr)
{
thereturn_t caution;

writeRegister(ACCEL_ADDR, DATAWRITE_SRC_MODE_SUB_SLV0, wtr);

return caution;

}

thereturn_t THEHUB::slv_0conf(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, SLAVE0_CONFIG, &dabuff, 1);
writeRegister(ACCEL_ADDR, SLAVE0_CONFIG, dabuff|=0x06); // 6-operations because-->  2-bytes for every axis, one auxiliary sensor
readRegister(ACCEL_ADDR, SLAVE0_CONFIG, buff, 1);

caution = ALLGOOD;

return caution;

}

//From this point there are the functions that are responsible for the FIFO buffer





thereturn_t THEHUB::fifo_conf(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

magfifoconfig(&dabuff);

//Setting the total number of continious operations so that the whole accelerometer would be read
//Also setting up the number of auxiliary sensors that are connected to the hub --> default value 00
//slv_0conf(buff);

//accel_scaleset(buff);
//gyro_scaleset(buff);
////BDU(buff);
//fifo_watermark(buff, 180);
//fifo_mode(buff);
//fifodec_accel(buff);
//fifodec_gyro(buff);
//fifodec_mag(buff);
//func_cfg_en(buff);
////readRegister(ACCEL_ADDR, FIFO_CTRL5, &dabuff, 1);
//i2c_mast_en(buff);
//fifo_odr(buff);
//accel_ODRset(buff);
//gyro_ODRset(buff);
//
//*buff = dabuff;

accel_scaleset(buff);

gyro_scaleset(buff);

fifo_watermark(buff, 1800);

fifo_mode(buff);

fifodec_accel(buff);

fifodec_gyro(&dabuff);

fifodec_mag(buff);

i2c_mast_en(buff);

hub_mast_en();

fifo_odr(buff);

accel_ODRset(buff);

gyro_ODRset(buff);


caution = ALLGOOD;

return caution;

}

uint8_t THEHUB::fifo_getstatus(void)
{
uint8_t result;

readRegister(ACCEL_ADDR, FIFO_STATUS2, &result,1);

return result;
}

thereturn_t THEHUB::fifo_clear(void)
{
thereturn_t caution;
uint8_t dabuff;

//readRegister(ACCEL_ADDR, FIFO_STATUS2, &dabuff,1);
while((fifo_getstatus()&(1<<4)) == 0)
{

fifo_read();
readRegister(ACCEL_ADDR, FIFO_STATUS2, &dabuff,1);

};

caution = ALLGOOD;

return caution;

}


//int length_of_pattern = 18;
//int threshold = 10*lenght_of_pattern;
thereturn_t THEHUB::fifo_watermark(uint8_t* buff, uint16_t threshold)


{
thereturn_t caution;
uint8_t lsb;
uint8_t msb;

lsb = threshold & 0x00FF;
msb = (threshold & 0x0F00)>>8;

writeRegister(ACCEL_ADDR, FIFO_CTRL1, lsb);
//readRegister(ACCEL_ADDR, FIFO_CTRL1,buff,1);
writeRegister(ACCEL_ADDR, FIFO_CTRL2, msb);
readRegister(ACCEL_ADDR, FIFO_CTRL2,buff,1);

caution = ALLGOOD;

return caution;

}


thereturn_t THEHUB::fifo_mode(uint8_t* buff)


{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, FIFO_CTRL5, &dabuff, 1);
writeRegister(ACCEL_ADDR, FIFO_CTRL5, dabuff|=0x06); //Continuous mode
readRegister(ACCEL_ADDR, FIFO_CTRL5, buff, 1);

caution = ALLGOOD;

return caution;

}


thereturn_t THEHUB::fifo_odr(uint8_t* buff)


{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, FIFO_CTRL5, &dabuff, 1);
writeRegister(ACCEL_ADDR, FIFO_CTRL5, dabuff|=0x10); //FIFO ODR --> 26Hz
readRegister(ACCEL_ADDR, FIFO_CTRL5, buff, 1);

caution = ALLGOOD;

return caution;

}


thereturn_t THEHUB::fifodec_accel(uint8_t* buff)


{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, FIFO_CTRL3, &dabuff, 1);
writeRegister(ACCEL_ADDR, FIFO_CTRL3, dabuff|=0x01); //Accel decimation --> 0
readRegister(ACCEL_ADDR, FIFO_CTRL3, buff, 1);

caution = ALLGOOD;

return caution;

}



thereturn_t THEHUB::fifodec_gyro(uint8_t* buff)


{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, FIFO_CTRL3, &dabuff, 1);
writeRegister(ACCEL_ADDR, FIFO_CTRL3, dabuff|=0x08); //Gyro decimation --> 0
readRegister(ACCEL_ADDR, FIFO_CTRL3, buff, 1);

caution = ALLGOOD;

return caution;

}


thereturn_t THEHUB::fifodec_mag(uint8_t* buff)


{
thereturn_t caution;
uint8_t dabuff;

readRegister(ACCEL_ADDR, FIFO_CTRL4, &dabuff, 1);
writeRegister(ACCEL_ADDR, FIFO_CTRL4, dabuff|=0x01); //Mag decimation --> 0
readRegister(ACCEL_ADDR, FIFO_CTRL4, buff, 1);

caution = ALLGOOD;

return caution;

}


uint8_t THEHUB::fifo_wtrm_flag(void)
{

uint8_t result;

readRegister(ACCEL_ADDR, FIFO_STATUS2, &result, 1);

return result;
}



uint16_t THEHUB::fifo_read(void)
{

uint16_t value = 0;
uint8_t temp = 0;

readRegister(ACCEL_ADDR, FIFO_DATA_OUT_L, &temp, 1);
value = temp;
readRegister(ACCEL_ADDR, FIFO_DATA_OUT_H, &temp, 1);
value |= ((uint16_t)temp<<8);

return value;

}

thereturn_t THEHUB::fifodefault(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

accel_scaleset(buff);

gyro_scaleset(buff);

accel_ODRset(buff);

gyro_ODRset(buff);

fifo_watermark(buff, 1800);

fifodec_accel(buff);

fifodec_gyro(buff);

fifo_mode(buff);

fifo_odr(buff);

//readRegister(ACCEL_ADDR, CTRL9_XL, &dabuff, 1);

caution = ALLGOOD;

*buff = dabuff;

return caution;
}

thereturn_t THEHUB::magfifoconfig(uint8_t* buff)
{
thereturn_t caution;
uint8_t dabuff;

passthrough(&dabuff);

//Enable external Functions
func_cfg_en(buff);

//Setting the Address of the slave;
slv_0add(MAG_ADDR, 1); //Slave MAG3110 with read enabled rw_0 -->1
readRegister(ACCEL_ADDR, SLV0_ADD, buff, 1);

//Setting the register of the slave to be read
slv_0subadd(MAG3110_OUT_X_MSB);

slv_0conf(buff);

//Disable external Functions
func_cfg_dis();

*buff = dabuff;
//extrn_read(buff);




return caution;

}

//	This is a library for use with the SparkFun LSM6DS3 intertial module 
//	in conjunction with the SparkFun MAG3110 Magnetometer but used in series connected with the I2C Protocol
//
//

#include "LSM6MAG5.h"
#include "stdint.h"
//#include <iostream>
#include "Wire.h"

//using namespace std;

//  LSM6DS3 initial;
// H function pou diavazei ta registers
unsigned char LSM6DS3::readReg(uint8_t regAddr,int nob) {

//uint8_t lsb;
//uint8_t msb;
unsigned char data=0;
Wire.beginTransmission(SENS_ADDR);
Wire.write(regAddr); //Send the Register Address
Wire.endTransmission();
 //End the communication sequence.
Wire.beginTransmission(SENS_ADDR);
Wire.requestFrom(SENS_ADDR, nob);
// Ask the device for data
if(Wire.available()){
//  Wait for a response from device
data = Wire.read();
//  read the data

}

Wire.endTransmission(); //End the communication sequence
    return data;
}

// H function pou diavazei ta registers gia dedomena 2-byte -> tis metriseis

int16_t LSM6DS3::readReg16(uint8_t regAddr,int nob) {

uint8_t lsb;
uint8_t msb;
int16_t data=0;
Wire.beginTransmission(SENS_ADDR);
Wire.write(regAddr); //Send the Register Address
Wire.endTransmission();
 //End the communication sequence.
Wire.beginTransmission(SENS_ADDR);
Wire.requestFrom(SENS_ADDR, nob);
// Ask the device for data

if(Wire.available()){

//  Wait for a response from device
lsb = Wire.read();
msb = Wire.read();
data = (lsb|(msb<<8));
//  read the data
    }



Wire.endTransmission(); //End the communication sequence
    return data;
}

// H function pou grafei sta registers (8-bit)

void LSM6DS3::writeReg(uint8_t regAddr,uint8_t WhatoWrite){

Wire.beginTransmission(SENS_ADDR);
 //initiate the send sequence
Wire.write(regAddr);
 //the register address to write
Wire.write(WhatoWrite);
 //the data to be written
Wire.endTransmission();


}

// O constructor mou

LSM6DS3::LSM6DS3()
{
//Accelerometer Settings
custom.accelenable = 1;
custom.accelODR = 104;
custom.accelBW = 50;
custom.accelDecim = 0;
custom.accelFS = 2;

//Gyroscope Settings
custom.gyroenable = 1;
custom.gyroODR = 104;
custom.gyroDecim = 0;
custom.gyroFS = 250;


//Magnetometer
custom.extenable = 0;

//FIFO settings
custom.fifoODR = 104;
custom.fifoThreshold = 2000;

//custom.fifomode = 6; //Continuous -> to grafw kateutheian


}

// Accelerometer
//
//
//Setting the ODR of the Accelerometer

uint8_t Sensor::accelinit(void){
 custom.accelset = 0x00;
switch (custom.accelODR){

     case 13 :
     custom.accelset |=0x10;
        break;

     case 26 :
     custom.accelset |=0x20;
        break;

     case 52 :
     custom.accelset |=0x30;
        break;

    case 104 :
    custom.accelset |=0x40;
        break;

    case 208 :
    custom.accelset |=0x50;
        break;

    case 416 :
    custom.accelset |=0x60;
        break;

    case 833 :
    custom.accelset |=0x70;
        break;

    case 1660:
    custom.accelset |=0x80;
        break;

    case 3330:
    custom.accelset |=0x90;
        break;

    case 6660:
    custom.accelset |=0xA0;
        break;
    }

// Setting the BW of the accelerometer
switch (custom.accelBW){

     case 50 :
     custom.accelset |=0x03;
        break;

     case 100 :
     custom.accelset |=0x02;
        break;

     case 200 :
     custom.accelset |=0x01;
        break;

     case 400 :
     custom.accelset |=0x00;
        break;

     }

//Setting the Scale of the Accelerometer
switch (custom.accelFS){
    case 2:
    custom.accelset |=0x00;
        break;

    case 4:
    custom.accelset |=0x08;
        break;

    case 8:
    custom.accelset |=0x0C;
        break;

    case 16:
    custom.accelset |=0x04;
        break;
    }
    return  custom.accelset;
}

// Gyroscope
//
//
//Setting the ODR of the Gyroscope
uint8_t Sensor::gyroinit(void){

 custom.gyroset = 0;
switch (custom.gyroODR){

     case 13 :
     custom.gyroset |=0x10;
        break;

     case 26 :
     custom.gyroset |=0x20;
        break;

     case 52 :
     custom.gyroset |=0x30;
        break;

     case 104 :
     custom.gyroset |=0x40;
        break;

     case 208 :
     custom.gyroset |=0x50;
        break;

     case 416 :
     custom.gyroset |=0x60;
        break;

     case 833 :
     custom.gyroset |=0x70;
        break;

     case 1660:
     custom.gyroset |=0x80;
        break;

     case 3330:
     custom.gyroset |=0x90;
        break;

     case 6660:
     custom.gyroset |=0xA0;
        break;
    }

// Setting the scale of the gyroscope
 switch (custom.gyroFS){
      case 250:
      custom.gyroset |=0x00;
          break;

      case 500:
      custom.gyroset |=0x04;
          break;

      case 1000:
      custom.gyroset |=0x08;
          break;

      case 2000:
      custom.gyroset |=0x0C;
          break;


    }
    return  custom.gyroset;
}

// FIFO Buffer
//
//
//Setting the ODR of the FIFO

//FIFO_CTRL5
uint8_t Sensor::fifoinit(void){
 custom.fifoset = 0x00;
switch (custom.fifoODR){

     case 12 :
     custom.fifoset |=0x08;
        break;

     case 26 :
     custom.fifoset |=0x10;
        break;

     case 52 :
     custom.fifoset |=0x18;
        break;

    case 104 :
    custom.fifoset |=0x20;
        break;

    case 208 :
    custom.fifoset |=0x28;
        break;

    case 416 :
    custom.fifoset |=0x30;
        break;

    case 833 :
    custom.fifoset |=0x38;
        break;

    case 1660:
    custom.fifoset |=0x40;
        break;

    case 3330:
    custom.fifoset |=0x48;
        break;

    case 6660:
    custom.fifoset |=0x50;
        break;
    }
    //fifomode -> exw dialeksei continuous mode kateutheian giati mallon mono  auto tha xrisimopoiisw
    custom.fifoset |=0x06;

    uint8_t fifoenabler;
    uint8_t fifoenablerext = 0x00;
    uint8_t lsb;
    uint8_t msb;
    uint8_t forceset;

    if (custom.accelenable==1){
    fifoenabler|=0x01;
    }
    else if (custom.accelenable==0){
    fifoenabler|=0x00;
    }
    if (custom.gyroenable==1){
    fifoenabler|=0x08;
    }
    else if (custom.gyroenable==0){
    fifoenabler|=0x00;
    }
    if (custom.extenable==1){
    fifoenablerext|=0x08;
    }
    else if (custom.extenable==0){
    fifoenablerext|=0x00;
    }
    forceset = 0x00; //Prosoxi na min ksexasw na to ftiakswwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
    lsb=custom.fifoThreshold & 0x00FF;
    msb=(custom.fifoThreshold & 0x0F00) >> 8;
    writeReg(FIFO_CTRL1, lsb);
    writeReg(FIFO_CTRL2, msb);
    writeReg(FIFO_CTRL3,fifoenabler); //fifoenabler: enables accel or gyro or both
    writeReg(FIFO_CTRL4,fifoenablerext); //fifoenablerext: enables external sensor
    //writeReg(FIFO_CTRL4,forceset);
return custom.fifoset;
//return fifoenablerext;
}

// Einai ta duo status gia to fifo buffer enwmena kai ta xrisimopoiw san flags gia to pote eina adeio gemato klp
int16_t Sensor::fifostatus(void){
int16_t status;
uint8_t lsb;
uint8_t msb;
lsb=readReg(FIFO_STATUS1,1);
msb=readReg(FIFO_STATUS2,1);
status=lsb|(msb<<8);
return status;
}

// Ennwnei duo 8-bit registers kai ftiaxnei to 16-bit pou einai kai to mikos twn metrisevn
int16_t Sensor::fiforead(void){
uint8_t lsb;
uint8_t msb;
int16_t read;

lsb=readReg(FIFO_DATA_OUT_L,1);
msb=readReg(FIFO_DATA_OUT_H,1);

read=lsb|(msb<<8);


return read;
}

// Adeiazei to buffer mou apo metriseis wste na ksekinisei apo tin arxi
bool Sensor::fifoclear(void){
bool status;
status = false;
while((fifostatus()&0x1000)==0) {
fiforead();
}
if(fifostatus()==0x1000)
status = true;
return status;

}

// Arxikopoiei ta settings aapo tous sensores
void Sensor::init(void){

writeReg(CTRL1_XL,accelinit());
writeReg(CTRL2_G,gyroinit());
writeReg(FIFO_CTRL5,fifoinit());


volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ ) //Dinei xrono gia to initialization
	{
		temp++;
	}


}

// Dedomena apo to accel
//X:
int16_t Sensor::accrawx(){
    int16_t data;
    data = readReg16(OUTX_L_XL, 2);

    return data;
}
//Y;
int16_t Sensor::accrawy(){
    int16_t data;
    data = readReg16(OUTY_L_XL, 2);

    return data;
}
//Z:
int16_t Sensor::accrawz(){
    int16_t data;
    data = readReg16(OUTZ_L_XL, 2);

    return data;
}

//Dedomena apo to gyro
//X:
int16_t Sensor::gyrorawx(){
    int16_t data;
    data = readReg16(OUTX_L_G, 2);

    return data;
}
//Y:
int16_t Sensor::gyrorawy(){
    int16_t data;
    data = readReg16(OUTY_L_G, 2);

    return data;
}
//Z:
int16_t Sensor::gyrorawz(){
    int16_t data;
    data = readReg16(OUTZ_L_G, 2);

    return data;
}

//Edw ta metatrepw se m/s^2
float Sensor::calcSI( int16_t input )
{
	float output = (float)input * 0.061 * (custom.accelFS >> 1)*GRAVITY / 1000;
	return output;
}

	float Sensor::calcGyroSI( int16_t input )
{
	uint8_t gyroRangeDivisor = custom.gyroFS / 125;
	if ( custom.gyroFS == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}


//Sensor Reset

void Sensor::reset(void){

writeReg(CTRL2_G,0x00);
writeReg(CTRL6_C,0x00);
writeReg(CTRL3_C,0x01);
}

//External Sensor Configuration




//*********************************************************************************************************
//          Pass-through efford                                                                             *
//                                                                                                          *
//*********************************************************************************************************
//
//bool Sensor::extinit(void){
//bool status=false;
//
//passenable();
////EDW tha ginei i vasiki arxikopoiisi
////maginit();
////passdisable();
//status=true;
//return status;
//}
//
//uint8_t Sensor::passenable(void){
//volatile uint8_t tempcount = 0;
//volatile uint8_t temp;
//temp = readReg(MASTER_CONFIG,1);
//writeReg(MASTER_CONFIG,temp|0x10);
//
//	for( uint16_t i = 0; i <50000; i++ ) //Dinei xrono
//	{
//		tempcount++;
//	}
//
//	for( uint16_t i = 0; i <50000; i++ ) //Dinei xrono
//	{
//		tempcount++;
//	}
//for( uint16_t i = 0; i <50000; i++ ) //Dinei xrono
//	{
//		tempcount++;
//	}
//for( uint16_t i = 0; i <50000; i++ ) //Dinei xrono
//	{
//		tempcount++;
//	}
//for( uint16_t i = 0; i <50000; i++ ) //Dinei xrono
//	{
//		tempcount++;
//	}
//
//temp=readReg(MASTER_CONFIG,1);
//writeReg(MASTER_CONFIG,0x00);
//temp=readReg(MASTER_CONFIG,1);
//writeReg(MASTER_CONFIG,temp|0x04);
////temp=readReg(MASTER_CONFIG,1);
//writeReg(MASTER_CONFIG,0x00);
//writeReg(MASTER_CONFIG,0x08);
//
//
//return temp;
//}
//
//void Sensor::passdisable(void){
//
//writeReg(MASTER_CONFIG,0x00);
//
//
//}
//
//bool Sensor::maginit(void){
//bool data;
//uint8_t temp;
//data = false;
////writeReg(0x10,0x48);
//temp = readReg(0x07,1);
//if (temp==0xC4){   //Who am i gia to Magnitometro
//data = true;
//}
//
//return data;
//}
//************************************************************************************************************


//*********************************************************************************************************
//          Classic                                                                                         *
//                                                                                                          *
//*********************************************************************************************************


bool Sensor::extinit(void){

bool data;
data = false;
writeReg(FUNC_CFG_ACCESS,0x80);
writeReg(SLV0_ADD,0x1C);
writeReg(SLV0_SUBADD,0x10);
writeReg(DATAWRITE_SRC_MODE_SUB_SLV0,0x19);
writeReg(FUNC_CFG_ACCESS,0x00);
writeReg(CTRL10_C,0x3C);
writeReg(MASTER_CONFIG,0x09);
writeReg(CTRL1_XL,0x80);
writeReg(CTRL10_C,0x38);
writeReg(MASTER_CONFIG,0x00);
writeReg(CTRL1_XL,0x00);
writeReg(FUNC_CFG_ACCESS,0x80);
writeReg(SLV0_ADD,0x1D);
writeReg(SLV0_SUBADD,0x01);
writeReg(SLAVE0_CONFIG,0x06);
writeReg(FUNC_CFG_ACCESS,0x00);
writeReg(CTRL10_C,0x3C);
writeReg(MASTER_CONFIG,0x09);
writeReg(CTRL1_XL,0x08);
data = true;
}

uint16_t Sensor::readexternX(void){
uint16_t data;
data = readReg16(MAG_OFFX_L, 2);

return data;
}

uint16_t Sensor::readexternY(void){
uint16_t data;
data = readReg16(MAG_OFFY_L, 2);

return data;
}

uint16_t Sensor::readexternZ(void){
uint16_t data;
data = readReg16(MAG_OFFZ_L, 2);

return data;
}

//************************************************************************************************************



//Debug
//Duo routines gia pio eukolo debugging

bool Sensor::readbool(uint8_t reg,uint8_t debval) {
bool status=false;
uint8_t temp;
temp = readReg(reg,1);
if(temp==debval){
status=true;
};

return status;
}

bool Sensor::readbool16(uint16_t reg,uint16_t debval) {
bool status=false;
if(reg==debval){
status=true;
};

return status;
}

bool Sensor::readmag(uint8_t reg){
writeReg(FUNC_CFG_ACCESS,0x80);
writeReg(SLV0_ADD,0x1C);
writeReg(SLV0_SUBADD,0x10);

}

uint8_t Sensor::readout(uint8_t reg){
uint8_t temp;
temp = readReg(reg,1);
return temp;
}


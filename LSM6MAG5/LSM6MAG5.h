/*
 * LSM6MAG.h
 *
 *  Created on: Nov 10, 2018
 *      Author: adim
 */

#ifndef LSM6MAG5_H_
#define LSM6MAG5_H_

#include "stdint.h"

// REGISTERS ADDRESSES (Hex)

#define FUNC_CFG_ACCESS					0X01
#define SENSOR_SYNC_TIME_FRAME			0X04
#define FIFO_CTRL1						0X06
#define FIFO_CTRL2						0X07
#define FIFO_CTRL3						0X08
#define FIFO_CTRL4						0X09
#define FIFO_CTRL5						0X0A
#define ORIENT_CFG_G					0X0B
#define INT1_CTRL						0X0D
#define INT2_CTRL						0X0E
#define WHO_AM_I						0X0F
#define CTRL1_XL						0X10
#define CTRL2_G							0X11
#define CTRL3_C							0X12
#define CTRL4_C							0X13
#define CTRL5_C							0X14
#define CTRL6_C							0X15
#define CTRL7_G							0X16
#define CTRL8_XL						0X17
#define CTRL9_XL						0X18
#define CTRL10_C						0X19
#define MASTER_CONFIG					0X1A
#define WAKE_UP_SRC						0X1B
#define TAP_SRC							0X1C
#define D6D_SRC							0X1D
#define STATUS_REG						0X1E
#define OUT_TEMP_L						0X20
#define OUT_TEMP						0X21
#define OUTX_L_G						0X22
#define OUTX_H_G						0X23
#define OUTY_L_G						0X24
#define OUTY_H_G						0X25
#define OUTZ_L_G						0X26
#define OUTZ_H_G						0X27
#define OUTX_L_XL						0X28
#define OUTX_H_XL						0X29
#define OUTY_L_XL						0X2A
#define OUTY_H_XL						0X2B
#define OUTZ_L_XL						0X2C
#define OUTZ_H_XL						0X2D
#define SENSORHUB1_REG					0X2E
#define SENSORHUB2_REG					0X2F
#define SENSORHUB3_REG					0X30
#define SENSORHUB4_REG					0X31
#define SENSORHUB5_REG					0X32
#define SENSORHUB6_REG					0X33
#define SENSORHUB7_REG					0X34
#define SENSORHUB8_REG					0X35
#define SENSORHUB9_REG					0X36
#define SENSORHUB10_REG					0X37
#define SENSORHUB11_REG					0X38
#define SENSORHUB12_REG					0X39
#define FIFO_STATUS1					0X3A
#define FIFO_STATUS2					0X3B
#define FIFO_STATUS3					0X3C
#define FIFO_STATUS4					0X3D
#define FIFO_DATA_OUT_L					0X3E
#define FIFO_DATA_OUT_H					0X3F
#define TIMESTAMP0_REG					0X40
#define TIMESTAMP1_REG					0X41
#define TIMESTAMP2_REG					0X42
#define STEP_TIMESTAMP_L				0X49
#define STEP_TIMESTAMP_H				0X4A
#define STEP_COUNTER_L					0X4B
#define STEP_COUNTER_H					0X4C
#define SENSORHUB13_REG					0X4D
#define SENSORHUB14_REG					0X4E
#define SENSORHUB15_REG					0X4F
#define SENSORHUB16_REG					0X50
#define SENSORHUB17_REG					0X51
#define SENSORHUB18_REG					0X52
#define FUNC_SRC						0X53
#define TAP_CFG							0X58
#define TAP_THS_6D						0X59
#define INT_DUR2						0X5A
#define WAKE_UP_THS						0X5B
#define WAKE_UP_DUR						0X5C
#define FREE_FALL						0X5D
#define MD1_CFG							0X5E
#define MD2_CFG							0X5F
#define OUT_MAG_RAW_X_L					0X66
#define OUT_MAG_RAW_X_H					0X67
#define OUT_MAG_RAW_Y_L					0X68
#define OUT_MAG_RAW_Y_H					0X69
#define OUT_MAG_RAW_Z_L					0X70
#define OUT_MAG_RAW_Z_H					0X71


//EMBEDDED FUNCTIONS REGISTERS

#define SLV0_ADD						0X02
#define SLV0_SUBADD                     0X03
#define SLAVE0_CONFIG                   0X04
#define SLV1_ADD                        0X05
#define SLV1_SUBADD                     0X06
#define SLAVE1_CONFIG                   0X07
#define SLV2_ADD                        0X08
#define SLV2_SUBADD                     0X09
#define SLAVE2_CONFIG                   0X0A
#define SLV3_ADD                        0X0B
#define SLV3_SUBADD                     0X0C
#define SLAVE3_CONFIG                   0X0D
#define DATAWRITE_SRC_MODE_SUB_SLV0     0X0E
#define SM_THS                          0X13
#define STEP_COUNT_DELTA                0X15
#define MAG_SI_XX                       0X24
#define MAG_SI_XY                       0X25
#define MAG_SI_XZ                       0X26
#define MAG_SI_YX                       0X27
#define MAG_SI_YY                       0X28
#define MAG_SI_YZ                       0X29
#define MAG_SI_ZX                       0X2A
#define MAG_SI_ZY                       0X2B
#define MAG_SI_ZZ                       0X2C
#define MAG_OFFX_L                      0X2D
#define MAG_OFFX_H                      0X2E
#define MAG_OFFY_L                      0X2F
#define MAG_OFFY_H                      0X30
#define MAG_OFFZ_L                      0X31
#define MAG_OFFZ_H                      0X32

#define SENS_ADDR                       0x6B
#define GRAVITY                         9.81

struct settings
{
    //Accelerometer
    unsigned int accelenable;
    unsigned int accelODR; //Can be 13,26,52,104,208,416,833Hz,1.66kHz,3.33kHz,6,66kHz
    unsigned int accelBW; //50,100,200,400 Hz
    unsigned int accelDecim; // Can be 0,2,3,4,8,16,32
    unsigned int accelFS; //Can be 2,4,8,16g
    //Gyroscope
    unsigned int gyroenable;
    unsigned int gyroODR; //Can be 13,26,52,104,208,416,833Hz,1.66kHz    ,3.33kHz,6,66kHz
    unsigned int gyroDecim; // Can be 0,2,3,4,8,16,32
    unsigned int gyroFS;  //250,500,1000,2000 dps

    //Magnetometer
    unsigned int  extenable;



    //FIFO

    unsigned int fifoODR; //Can be 13,26,52,104,208,416,833Hz,1.66kHz,3.33kHz,6,66kHz
    unsigned int fifomode; // Can be 1. Bypass, 2. FIFO mode 3. Reserved,
                          //4. Coninuous 5. Bypass 6. Reserved 7. Continuous 8. Reserved
//    unsigned int fifoaccel; //0:off 1:on
//    unsigned int fifogyro; //0:off 1:on
    unsigned int fifoext; //0:off 1:on
    uint16_t fifoThreshold;
//    unsigned int fifomag; Gia meta....
    unsigned int fifodecim; // Can be 0,2,3,4,8,16,32
    //unsigned int externalDecim; // Can be 0,2,3,4,8,16,32

    uint8_t accelset;
    uint8_t gyroset;
    uint8_t fifoset;
};

class LSM6DS3 {
public:

unsigned char readReg(uint8_t regAddr,int nob); //regAddr: Address of the register to be read
                                              //  nob: Number Of Bytes to be read
int16_t readReg16(uint8_t, int);
void writeReg(uint8_t regAddr,uint8_t WhatoWrite); //Address of the register to be written
                                                            //WhatoWrite: what to write!!
 struct settings custom;
 LSM6DS3(); //Constructor


private:


};


class Sensor: public LSM6DS3 {


    public:
      uint8_t accelinit(void);
      uint8_t gyroinit(void);
      void init(void);

    int16_t accrawx();
    int16_t accrawy();
    int16_t accrawz();

    int16_t gyrorawx();
    int16_t gyrorawy();
    int16_t gyrorawz();

    //SI calculations
    float calcSI( int16_t input );
    float calcGyroSI( int16_t input );

    //Magnetometer
    uint8_t passenable(void);
    void passdisable(void);
    bool extinit(void);
    bool maginit(void);
    uint16_t readexternX(void);
    uint16_t readexternY(void);
    uint16_t readexternZ(void);
//    void in_the_hub(void);


    //FIFO
    uint8_t fifoinit(void);
    int16_t fifostatus(void);
    int16_t fiforead(void);
    bool fifoclear(void);

    //RESET
    void reset(void);

    //Debugging
    bool readbool(uint8_t reg,uint8_t debval);
    bool readbool16(uint16_t reg,uint16_t debval);
    bool readmag(uint8_t reg);
    uint8_t readout(uint8_t reg);
};



#endif /* LSM6MAG_H_ */


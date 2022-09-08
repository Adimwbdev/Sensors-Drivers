#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFatConfig.h>
#include <SdFat.h>
#include <sdios.h>
#include <SysCall.h>
#include <BlockDriver.h>





#include <HubDub.h>




THEHUB mySens;

//SD
//SdFatSdio sd;
SdFat sd;
File myFile;
int pinCS = 10;

void setup() {
Serial.begin(9600);
delay(1000);


uint8_t buff;
uint8_t buff2;
uint8_t buff3;
uint8_t check;


Serial.println("Teensy ok");
mySens.beginSens();

pinMode(pinCS, OUTPUT);

Serial.print("RESET OK?:");
Serial.println(mySens.reset());

//:FIFO Read

mySens.fifo_conf(&buff);
//mySens.fifodefault(&buff);
Serial.print("Is FIFO clear?:");
if(mySens.fifo_clear())
Serial.println("........ ya ma man!");
//Serial.print("FIFO status:");
//mySens.readRegister(ACCEL_ADDR, FIFO_STATUS2, &buff2,1);
//Serial.println(buff2, HEX);
//Serial.print("The checking is:");
//Serial.println(buff, HEX);

if (!sd.begin())
        {
          Serial.println("SD card Error.");
        } 

}



void loop() {

  int i=0;
  uint8_t flag;


myFile = sd.open("test.txt", FILE_WRITE);


while((mySens.fifo_wtrm_flag()&0x80)==0){  };

Serial.println("Start emptying the FIFO!");

while((mySens.fifo_wtrm_flag()&0x10) == 0)
{ 
  i=1;

//  while(i<=172){
  
if (myFile) {

//Gyro
//  myFile.print("x: ");
  myFile.print(mySens.calcGyro(mySens.fifo_read()));myFile.print(", ");
//  myFile.print("y: ");
  myFile.print(mySens.calcGyro(mySens.fifo_read()));myFile.print(", ");
//  myFile.print("z: ");
  myFile.print(mySens.calcGyro(mySens.fifo_read()));myFile.print(", ");
  
//Accel  
//  myFile.print("x: ");
  myFile.print(mySens.calcSI(mySens.fifo_read()));myFile.print(", ");
//  myFile.print("y: ");
  myFile.print(mySens.calcSI(mySens.fifo_read()));myFile.print(", ");
//  myFile.print("z: ");
  myFile.print(mySens.calcSI(mySens.fifo_read()));

////Mag
  myFile.print(", ");
//  myFile.print("x: ");
  myFile.print(mySens.fifo_read());myFile.print(", ");
//  myFile.print("y: ");
  myFile.print(mySens.fifo_read());myFile.print(", ");
//  myFile.print("z: ");
  myFile.println(mySens.fifo_read());

  delay(100);
  
//  i++;
//    };
     }

    }

myFile.close();

Serial.println("Right in the card.....");


}

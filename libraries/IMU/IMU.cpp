/*
  Created by Basel Al-Rudainy, 6 april 2013.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "IMU.h"
#include "Arduino.h"
#include "Wire.h"

IMU::IMU()
{

}

void IMU::init(){
Wire.begin(); 
//init - ADXL345
writeCmd(ADXL_addr,REG_DATA_FORMAT,0x0B); //+-16g
writeCmd(ADXL_addr,REG_PWR_CTL,0x08);  //measurement mode
writeCmd(ADXL_addr,REG_BW_RATE,0x09);  //REG_BW_RATE rate=50hz, bw=20hz
//1G = 265
//-----end init ADXL345

//init - L3G4200D
/*writeCmd(L3G4_addr,CTRL_REG1,L3G4_BW_ENAX);
writeCmd(L3G4_addr,CTRL_REG2,L3G4_LPF);
writeCmd(L3G4_addr,CTRL_REG4,MODE_2000);
writeCmd(L3G4_addr,CTRL_REG5,L3G4_HPF);*/

  writeCmd(L3G4_addr, DLPF_FS, (DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
  //Set the sample rate to 100 hz
  writeCmd(L3G4_addr, SMPLRT_DIV, 9);

//-----end init L3G4200D

//init - HMC5883
writeCmd(HMC_addr,HMC_mode_reg,HMC_contm_val);
//-----end init HMC5883
}

void IMU::getAxlData(float buff[]){
byte buffer[6];
readCmd(ADXL_addr,DATAX0,6,buffer);
buff[0]=(buffer[1]<<8) | buffer[0];
buff[1]=(buffer[3]<<8) | buffer[2];
buff[2]=(buffer[5]<<8) | buffer[4];

}
/////////////////////////////Edited by Drew///////////////////////////////
void IMU::getGyroData(float buff[]){
/*byte buffer[6];
readCmd(L3G4_addr,READALLSIX,6,buffer);
buff[0]=(float)((int)(buffer[1]<<8) | buffer[0])*SCALE_2000;
buff[1]=(float)((int)(buffer[3]<<8) | buffer[2])*SCALE_2000;
buff[2]=(float)((int)(buffer[5]<<8) | buffer[4])*SCALE_2000;*/

buff[0] = (float)readX();
buff[1] = (float)readY();
buff[2] = (float)readZ();
}

//////////////////////////////////////////////////////////////////////////

void IMU::getMagData(int buff[]){
byte buffer[6];
readCmd(HMC_addr,HMC_X_MSB,6,buffer);
buff[0]=(buffer[0]<<8) | buffer[1];
buff[2]=(buffer[2]<<8) | buffer[3];
buff[1]=(buffer[4]<<8) | buffer[5];
}

void IMU::readCmd( byte addr,byte reg,byte num,byte buff[] ){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, num);
  
  while(Wire.available()<num);
  
  for(byte i = 0;i<num;i++)
  {
	buff[i] = Wire.read();
  }
}

/*
//~~~~~~~~~~~Added by Jesse for testing?~~~~~~~~~~~~~~~~~
unsigned char MPULib::jReadCmd( char addr, char reg )
{
	byte data;

	//I'm not sure why this needs to be here
	Wire.beginTransmission(addr);
	Wire.write(reg);
	Wire.endTransmission();
	
	//ask Gyro for values
	Wire.beginTransmission(addr);
	Wire.requestFrom( addr, 1 );
	
	//wait for a response
	if( Wire.available() )
	{
		//grab the data
		data = Wire.read();
	}

	//close transmission
	Wire.endTransmission();
	
	return data;
}
*/


void IMU::writeCmd(byte addr, byte reg, byte val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

///////////////////////////////////Added by Drew///////////////////////////
unsigned char IMU::gyroRead(char address, char registerAddress)
{
  unsigned char data=0;

  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();
  
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 1);

  if(Wire.available())
  {
    data = Wire.read();
  }

  Wire.endTransmission();

  return data;
}

int IMU::readX()
{
  int data=0;
  data = gyroRead(L3G4_addr, OUT_X_H)<<8;
  data |= gyroRead(L3G4_addr, OUT_X_L);  

  return data;
}

int IMU::readY()
{
  int data=0;
  data = gyroRead(L3G4_addr, OUT_Y_H)<<8;
  data |= gyroRead(L3G4_addr, OUT_Y_L);  

  return data;
}

int IMU::readZ()
{
  int data=0;
  data = gyroRead(L3G4_addr, OUT_Z_H)<<8;
  data |= gyroRead(L3G4_addr, OUT_Z_L);  

  return data;
}

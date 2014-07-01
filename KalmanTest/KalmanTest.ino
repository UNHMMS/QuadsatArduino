#include <MPULib.h>
#include <Wire.h>
#include "Kalman.h"

#define GRAV (9.80665/250)
#define RADS (3.14159/180)

short gyroBuf[3];  // { x, y, z }
short axlBuf[3];  // { x, y, z }

float gyroFlt[3];
float axlFlt[3];

float gyroCalib[3] = {37.0, 5.0, -2.0};  // { x, y, z } offsets
float axlCalib[3] = { -9.0, 0.0, 8.0 };  // { x, y, z } offsets
float angleCalib[2] = { -1.57, -1.57 };



MPULib imu;  //IMU library pulls data from the gyro and the accelerometer

Kalman kFilterX;
Kalman kFilterY;
//Kalman kFilterZ;

double axlAngle[3]; 
float axlSum = 0.0;
float lastAngle[2] = { 0.0, 0.0 };
double kAngle[3];

unsigned long lastTime;
double timeStep;

void setup()
{
  Serial.begin( 115200 );
  imu.init();
  
  imu.getAxlData( axlBuf );
  imu.getGyroData( gyroBuf );
  
  for( int i = 0; i < 3; i++ )
  {
    axlFlt[i] = axlBuf[i];
    gyroFlt[i] = gyroBuf[i]; 
  }
  
  for( int i = 0; i < 3; i++ )
  {
    axlFlt[i] -= ( axlFlt[i] + axlCalib[i] ) * -GRAV;
    
    axlSum = pow( axlFlt[i], 2);
    
    if( ( gyroFlt[i] += gyroCalib[i] ) == 0.0 )
    {
      gyroFlt[i] = ((gyroFlt[i]) / 14.375) * RADS ;
    } 
  }
  
  axlSum = sqrt(axlSum);
  
  axlAngle[0] = ( acos(axlFlt[0]/axlSum) ) + angleCalib[0];
  axlAngle[1] = ( acos(axlFlt[1]/axlSum) ) + angleCalib[0];
  
  //set the gyro angles
  kFilterX.setAngle( axlAngle[0] ); // Set starting angle
  kFilterY.setAngle( axlAngle[1] );
  //kFilterZ.setAngle( axlAngle[2] );
  //gyroAngle[0] = axlAngle[0];
  //gyroAngle[1] = axlAngle[1];
  
  lastTime = micros();
}

void loop()
{
  //update all values
  imu.getAxlData( axlBuf );
  imu.getGyroData( gyroBuf );
  
  for( int i = 0; i < 3; i++ )
  {
    axlFlt[i] = axlBuf[i];
    gyroFlt[i] = gyroBuf[i]; 
  }
  
  /*
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( axlFlt[i] + axlCalib[i] );
    Serial.print( " " );
  }
  //Serial.println();
  */
  
  for( int i = 0; i < 3; i++ )
  {
    axlFlt[i] = ( axlFlt[i] + axlCalib[i] ) * -GRAV;
    
    axlSum = pow( axlFlt[i], 2);
    
    if( (gyroFlt[i] += gyroCalib[i]) != 0.0 )
    {
      gyroFlt[i] = ((gyroFlt[i]) / 14.375) * RADS ;
    }
  }
  
  
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( gyroFlt[i] );
    Serial.print( " " );
  }
  
  
  axlSum = sqrt(axlSum);
  
  axlAngle[0] = ( acos(axlFlt[0]/axlSum) ) + angleCalib[0];
  axlAngle[1] = ( acos(axlFlt[1]/axlSum) ) + angleCalib[0];
  //axlAngle[2] = (atan2(axlBuf[1], axlBuf[0]) + PI);
  
  /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print( axlAngle[i] );
    Serial.print( " " );
  }
  */
  
  for( int i = 0; i < 2; i++ )
  {
    if( isnan( axlAngle[i] ) )
    {
      axlAngle[i] = 0.95;
      if( lastAngle[i] < 0 )
      {
        axlAngle[i] *= -1;
      }
      
    }
    
    lastAngle[i] = axlAngle[i];
  }
  
  timeStep = (double)(micros() - lastTime) / 1000000;
  
  kAngle[0] = kFilterX.getAngle(axlAngle[0], gyroFlt[0], timeStep );
  kAngle[1] = kFilterY.getAngle(axlAngle[1], gyroFlt[1], timeStep );
  //kAngle[2] = kFilter.getAngle(axlAngle[2], gyroAngle[2], timeStep );
  
  /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print( kAngle[i] );
    Serial.print( " " );
  }
  */
  
  lastTime = micros();
  
  /*
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( axlAngle[i] );
    Serial.print( " " );
  }  
  
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( gyroAngle[i] );
    Serial.print( " " );
  }
  
  
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( kAngle[i] );
    Serial.print( " " );
  }
  */
  
  Serial.println();
  
  
}

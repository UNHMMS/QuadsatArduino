#include "Kalman.h"
#include <MPULib.h>       //Used to grab data from the IMU
#include <Wire.h>         //necessary for the MPULib library to function properly

#define GRAV (9.80665/250)
#define RADS (3.14159/180)

MPULib imu;                //this is the object that we'll access the IMU through
                           // used in IMU Reading

short gyroBuffer[3];        //gyro data gets written into this buffer
// 0:x, 1:y, 2:z

short axlBuffer[3];        //accelerometer data gets written into this buffer
// 0:x, 1:y, 2:z

float gyroFlt[3];        //gyro data gets written into this buffer
// 0:x, 1:y, 2:z

float axlFlt[3];        //accelerometer data gets written into this buffer
// 0:x, 1:y, 2:z

float axlAngle[2];        //calculated angle values using the angle data
float lastAngle[2] = { 0.0, 0.0 };
float kAngle[2];

float gyroCalib[3] =       //gyro callibration values
  {33.0, 2.0, -2.0};
// 0:x, 1:y, 2:z

float axlCalib[3] =       //gyro callibration values
  {6.0, -11.0, 21.0};
// 0:x, 1:y, 2:z

float angleCalib[2] = { -1.63, -1.52 }; //offsets for pitch and roll angles
float axlSum = 0.0;

Kalman kFilterPitch;  //kalman filter for pitch
Kalman kFilterRoll;  //kalman filter for raw
double kOut = 0;

unsigned long lastRead = 0;
double timeStep = 0.0;

void initIMU()
{
  imu.init(); 
}

// updates the buffers with new IMU values.
void updateIMU()
{
  imu.getGyroData( gyroBuffer );
  imu.getAxlData( axlBuffer );
  
  for( int i = 0; i < 3; i++ )
  {
    gyroFlt[i] = gyroBuffer[i];
    axlFlt[i] = axlBuffer[i]; 
  }
  /*
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( axlFlt[i] + axlCalib[i] );
    Serial.print( " " );
  }
  Serial.println();
  */
  
  for( int i = 0; i < 3; i++ )
  {
    axlBuffer[i] = ( axlBuffer[i] + axlCalib[i] ) * -GRAV;
    axlSum = pow( axlBuffer[i], 2);
    
    if( (gyroBuffer[i] += gyroCalib[i]) != 0.0 )
    {
      gyroBuffer[i] = ((gyroBuffer[i] + gyroCalib[i]) / 14.375) * RADS ;
    } 
  }
  
  axlSum = sqrt( axlSum );
  
  axlAngle[0] = ( ( acos(axlBuffer[0]/axlSum) ) + angleCalib[0] ) *-1;
  axlAngle[1] = ( acos(axlBuffer[1]/axlSum) ) + angleCalib[1];
  
  for( int i = 0; i < 2; i++ )
  {
    if( isnan( axlAngle[i] ) )
    {
      if( lastAngle[i] < 0 )
      {
        axlAngle[i] = -0.95;
      }
      else
      {
        axlAngle[i] = 0.95;
      }
    }
    
    lastAngle[i] = axlAngle[i];
  }
  
  /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print(axlAngle[i]);
    Serial.print( " " );
  }
  */
  
  
  //Serial.print(axlSum);
  
  timeStep = (double)( micros() - lastRead ) / 1000000;
  
  kAngle[0] = kFilterPitch.getAngle(axlAngle[0], gyroBuffer[0], timeStep);
  kAngle[1] = kFilterRoll.getAngle(axlAngle[1], gyroBuffer[1], timeStep);
 
 /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print(kAngle[i]);
    Serial.print( " " );
  }
  */
  
  lastRead = micros();
}

float getGyroRoll()
{
  return gyroBuffer[0]; 
}

float getGyroPitch()
{ 
  return gyroBuffer[1]; 
}

float getGyroYaw()
{
  return gyroBuffer[2]; 
}

float getRawRoll()
{
  return axlAngle[1];
}

float getRawPitch()
{
  return axlAngle[0];
}

float getRoll()
{
  return kAngle[1];
}

float getPitch()
{
  return kAngle[0];
}

void setKalmanRollAngle()
{
  updateIMU();
  kFilterRoll.setAngle( axlAngle[1] );
}

void setKalmanPitchAngle()
{
  updateIMU();
  kFilterPitch.setAngle( axlAngle[0] );
}

void resetFilter()
{
  kFilterRoll.reset();
  kFilterPitch.reset();
}

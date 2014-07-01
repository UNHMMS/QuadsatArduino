#include "Kalman.h"
#include <MPULib.h>       //Used to grab data from the IMU
#include <Wire.h>         //necessary for the MPULib library to function properly

/*
  Definition of Rotationgs:
  About X : ROLLING
  About Y : PITCHING
  About Z : YAWING
*/

#define GRAV (9.80665/250)
#define RADS (3.14159/180)
#define GAUSS (1090)

MPULib imu;                //this is the object that we'll access the IMU through
                           // used in IMU Reading

short gyroBuffer[3];        //gyro data gets written into this buffer
// 0:x:roll, 1:y:Pitch, 2:z:yaw

short axlBuffer[3];        //accelerometer data gets written into this buffer
// 0:x:roll, 1:y:Pitch, 2:z:yaw

short magBuffer[3];      //magnetometer data gets written into this from the sensor

float gyroFlt[3];        //gyro data gets transfered to this array from the short array
// 0:x:roll, 1:y:Pitch, 2:z:yaw

float axlFlt[3];        //accelerometer data gets transfered to this array from the short array
// 0:x:roll, 1:y:Pitch, 2:z:yaw

float magFlt[3];        //magnetometer data gets transfered to this array from the short array

float axlAngle[2];        //calculated angle values using the angle data
float lastAngle[2] = { 0.0, 0.0 };
float kAngle[3];
float rawHeading;

float gyroCalib[3] =       //gyro callibration values
  {25.0, -14.0, -17.0};
// 0:x:roll, 1:y:Pitch, 2:z:yaw

float axlCalib[3] =       //gyro callibration values
  {-5.0, -7.0, -4.0};
// 0:x:roll, 1:y:Pitch, 2:z:yaw

float yawCalib = 0.0;

float angleCalib[2] = { -1.61, -1.52 }; //offsets for pitch and roll angles
float axlSum = 0.0;

Kalman kFilterPitch;  //kalman filter for pitch
Kalman kFilterRoll;  //kalman filter for raw
Kalman kFilterYaw;  //NOT USED kalman filter for yaw
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
  imu.getMagData( magBuffer );
  
  for( int i = 0; i < 3; i++ )
  {
    gyroFlt[i] = gyroBuffer[i];
    axlFlt[i] = axlBuffer[i]; 
    magFlt[i] = magBuffer[i];
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
    axlFlt[i] = ( axlFlt[i] + axlCalib[i] ) * -GRAV;
    axlSum = pow( axlFlt[i], 2);
    
    if( (gyroFlt[i] += gyroCalib[i]) != 0.0 )
    {
      gyroFlt[i] = (gyroFlt[i] / 14.375) * RADS ;
    }
    
    magFlt[i] = magFlt[i] / GAUSS * 100;
  }
  /*
  for( int i = 0; i < 3; i++ )
  {
    Serial.print( gyroFlt[i] );
    Serial.print( " " );
  }
  Serial.println();
  */
  axlSum = sqrt( axlSum );
  
  axlAngle[0] = ( ( acos(axlFlt[0]/axlSum) ) + angleCalib[0] ) *-1; //X axis, Roll, Motors 8, 12
  axlAngle[1] = ( acos(axlFlt[1]/axlSum) ) + angleCalib[1];//Y axis, Pitch, Motors 13, 10
  
  rawHeading = atan2( magFlt[1], magFlt[0] );
  /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print( axlAngle[i] );
    Serial.print( " " );
  }
  Serial.println();
  */
  
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
  if( rawHeading < 0 )
    rawHeading += 2*PI;
    
  if( rawHeading > 2*PI )
    rawHeading -= 2*PI;
    
  if( rawHeading > PI )
    rawHeading -= 2*PI;
    */
    
  rawHeading -= yawCalib;
  //rawHeading *= PI;
  
  /*
  for( int i = 0; i < 2; i++ )
  {
    Serial.print(axlAngle[i]);
    Serial.print( " " );
  }
  */
  
  
  //Serial.print(axlSum);
  
  timeStep = (double)( micros() - lastRead ) / 1000000;
  
  kAngle[0] = kFilterRoll.getAngle(axlAngle[1], gyroFlt[0], timeStep);
  kAngle[1] = kFilterPitch.getAngle(axlAngle[0], gyroFlt[1], timeStep);
  kAngle[2] = kFilterYaw.getAngle(rawHeading, gyroFlt[2], timeStep);
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
  return gyroFlt[0]; 
}

float getGyroPitch()
{ 
  return gyroFlt[1]; 
}

float getGyroYaw()
{
  return gyroFlt[2]; 
}

float getRawRoll()
{
  return axlAngle[1];
}

float getRawPitch()
{
  return axlAngle[0];
}

float getRawYaw()
{
  return rawHeading; 
}

float getRoll()
{
  return kAngle[0];
}

float getPitch()
{
  return kAngle[1];
}

float getYaw()
{
  return kAngle[2]; 
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

void setYawDomain()
{
  yawCalib = rawHeading;
}

void resetYawDomain()
{
  yawCalib = 0.0; 
}

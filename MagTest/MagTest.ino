#include <MPULib.h>
#include <Wire.h>

#define GAUSSXY (1100)
#define GAUSSZ (980)

MPULib imu;

short magBuffer[3];
float magFlt[3];
float heading;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  imu.init();
}

void loop() 
{
  imu.getMagData( magBuffer );
  
  
  for( int i = 0; i < 3; i++ )
  {
     magFlt[i] = magBuffer[i]; 
  }
  
  magFlt[0] = magFlt[0] / GAUSSXY * 100;
  magFlt[1] = magFlt[1] / GAUSSXY * 100;
  magFlt[2] = magFlt[2] / GAUSSZ * 100;
  
  heading = atan2( magFlt[1], magFlt[0] );
  
  heading += -0.262;
  
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
    
  if( heading > PI )
    heading -= 2*PI;
    
  
  Serial.print( "Heading (Rads): " );
  Serial.println( heading );
  
  
  /*
  Serial.print( magFlt[0] );
  Serial.print( " " );
  Serial.print( magFlt[1] );
  Serial.print( " " );
  Serial.print( magFlt[2] );
  Serial.print( " " );
  Serial.println();
  */
}

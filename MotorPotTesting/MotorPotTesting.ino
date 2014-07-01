#include <Servo.h>

Servo motors[4];
int motorPins[4] = {2, 4, 6, 8};
int count = 0;
int count2 = 0;

void setup()
{
  Serial.begin( 57600 );
  
  for( int i = 0; i < 4; i++ )
  {
    motors[i].attach( motorPins[i] );
    motors[i].write( 0 );
  }
  
  Serial.println( "We've Been Setup!" );
  delay( 10000 );
  Serial.println( "GO!" );
}

void loop()
{
  for( int i = 0; i < 4; i++ )
  {
     motors[i].write( count ); 
  }
  
  if( count < 70 )
  {
    count++;
    Serial.println( count );    
  }
  
  count2++;
  
  if( count2 > 90 )
  {
     if( count2 > 94 )
     {
       Serial.println( "done" );
       
        for( int i = 0; i < 4; i++ )
        {
          motors[i].write( 0 ); 
        } 
        
        while( true ) {}
     }
     else if( count2 > 92 )
     {
       Serial.println( "Straight to 70" );
       
       for( int i = 0; i < 4; i++ )
        {
          motors[i].write( count ); 
        } 
        
        delay( 1000 );
     }
     else 
     {
        Serial.println( "temp down" );
        
        for( int i = 0; i < 4; i++ )
        {
          motors[i].write( 0 ); 
        } 
        
        delay( 1000 );
     }
  }
  
  delay( 100 );
}

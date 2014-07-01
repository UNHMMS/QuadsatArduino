// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
/*
Servo motors[4];
int motorPins[] =          //pins that the motors are tied to
  {2, 4, 6, 8};
*/

Servo motor;
int motorPin = 6;
 
int potpin = A4;  // analog pin used to connect the potentiometer
int val = 0;       // variable to store the value coming from the sensor
int oldVal = 0;

int plus = 5;
 
void setup() 
{ 
  Serial.begin( 57600 );
  
  motor.attach( motorPin );
  motor.write( 0 );
  
  pinMode( plus, OUTPUT );
  digitalWrite( plus, HIGH );
  
  /*
  for( int i = 0; i < 4; i++ )
  {
    motors[i].attach( motorPins[i] );
    motors[i].write( 0 );
  }
  */
  
  Serial.println( "We've Been Setup!" );
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  
  /*
  for( int i = 0; i < 4; i++ )
  {
    motors[i].write( val );
  }  // sets the servo position according to the scaled value
  */
  
  motor.write( val );
  //analogWrite( 6, val );
  
  if( val != oldVal )
  {
    Serial.println(val);         // print the value from the sensor
  }
  
  oldVal = val;
}

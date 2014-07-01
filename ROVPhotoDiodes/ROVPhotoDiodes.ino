/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
// const int a0 = A0;  // Analog input pin that the potentiometer is attached to
int pins[5] = {0, 1, 2, 3, 4};

int sv[5];

int outputValue = 0;        // value output to the PWM (analog out)

void setup() 
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
}

void loop() 
{
  
  if( Serial.available() && Serial.read() == '!' )
  {
    // read the analog in value:
    for( int i = 0; i < 5; i++ )
    {
       sv[i] = analogRead( pins[i] ); 
    }
    
    //write the analog reads from the 
    Serial.print("sensor = " );    
    
    for( int i = 0; i < 5; i++ )
    {
      Serial.print(sv[i]);
      Serial.print(", ");
    }
    
    Serial.print( "\n" );
  
    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(2);
  }    
}

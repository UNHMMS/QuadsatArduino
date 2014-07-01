#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo

int ftPin = A5;
int servoPin = 6;

int motorVal = 0;
int count = 0;
int ftVal = 0;

String outString = "";


void setup() 
{ 
  myservo.attach( servoPin );
  Serial.begin(115200);
  //analogReference( 2.0 );
} 
 
void loop()
{ 
  while( motorVal < 180 )
  {
    //write the value to the motor
    myservo.write( motorVal );
    
    //reset the read count to 0
    count = 0;
    
    //take ten force transducer readings
    while( count < 10 )
    {
      //read the force transducer value
      ftVal = analogRead( ftPin );

      
      //make the string that will be written to the file
      outString = String( motorVal ) + " " + String( ftVal );
      
      //write the string to the python serial read program
      Serial.println( outString );
      
      //intcrement the count
      count = count + 1;
      
      //delay the program so it doesn't overload the serial port
      delay( 100 );
    }
    
    // move to the next motor value to take readings on
    motorVal = motorVal + 1;
  }
  
  motorVal = 0;
  myservo.write( motorVal );
  
  while( true )
  {
    Serial.println( "DONE" ); 
  }
  
}
